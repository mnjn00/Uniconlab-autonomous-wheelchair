#!/usr/bin/env bash
# Bring up the existing field stack, then replace only perception/control
# adapters with the RTX-accelerated hybrid profile. The original pursuit
# startup remains the immediate rollback path.
set -euo pipefail

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  cat <<'EOF'
Usage: start_hybrid_avoidance.sh

Starts the existing ROS1 field stack with PROFILE=dwa, then installs:
  MID-360 geometry + RTX 2060 PointPillars semantics -> DWA proposal
  -> semantic stop supervisor -> raw safety gate -> terrain guard
  -> tip_guard -> wheelchair base

Environment:
  START_POINTPILLARS=true|false                   # default true
  REQUIRE_LEARNED=true|false                      # defaults to START_POINTPILLARS
  POINTPILLARS_ENV=~/.config/unicon/pointpillars.env
  POINTPILLARS_MODEL=/path/to/pointpillar.plan
  POINTPILLARS_INPUT_TOPIC=/cloud_registered_body
  POINTPILLARS_DETECTIONS_TOPIC=/pointpillars/detections
  POINTPILLARS_EXPECTED_FRAME=body
  POINTPILLARS_REQUIRE_RTX2060=true|false
  CLIFF_REQUIRED=false|true
  CLIFF_TOPIC=/terrain/cliff_status
  SAFETY_POLICIES=true|false
  VN_IMU=0|1

Run `bash tools/hybrid.sh setup-gpu` once on the Phantom Canyon NUC before
starting the default GPU profile. Nothing moves until go_hybrid.sh is run.
Existing stop.sh remains authoritative.
EOF
  exit 0
fi
[ "$#" -eq 0 ] || { echo "ERROR: unexpected arguments" >&2; exit 64; }

say() { printf '\n=== %s ===\n' "$1"; }
fail() { echo "ERROR: $*" >&2; exit 1; }

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCALIZATION_WS="${LOCALIZATION_WS:-$HOME/livox_static_localization_ws}"
BASE_START="${BASE_START:-$HOME/start_wheelchair_localization.sh}"
[ -x "$BASE_START" ] || BASE_START="$REPO_ROOT/tools/start_wheelchair_localization.sh"
[ -x "$BASE_START" ] || fail "start_wheelchair_localization.sh not found"
[ -f "$LOCALIZATION_WS/devel/setup.bash" ] || \
  fail "localization workspace is not built: $LOCALIZATION_WS"

START_POINTPILLARS="${START_POINTPILLARS:-true}"
if [ "${REQUIRE_LEARNED+x}" = x ]; then
  REQUIRE_LEARNED="$REQUIRE_LEARNED"
else
  REQUIRE_LEARNED="$START_POINTPILLARS"
fi
POINTPILLARS_REQUIRE_RTX2060="${POINTPILLARS_REQUIRE_RTX2060:-true}"
CLIFF_REQUIRED="${CLIFF_REQUIRED:-false}"
SAFETY_POLICIES="${SAFETY_POLICIES:-true}"
for pair in "START_POINTPILLARS:$START_POINTPILLARS" \
            "REQUIRE_LEARNED:$REQUIRE_LEARNED" \
            "POINTPILLARS_REQUIRE_RTX2060:$POINTPILLARS_REQUIRE_RTX2060" \
            "CLIFF_REQUIRED:$CLIFF_REQUIRED" \
            "SAFETY_POLICIES:$SAFETY_POLICIES"; do
  name="${pair%%:*}"; value="${pair#*:}"
  case "$value" in true|false) ;; *) fail "$name must be true or false" ;; esac
done

POINTPILLARS_ENV="${POINTPILLARS_ENV:-$HOME/.config/unicon/pointpillars.env}"
if [ "$START_POINTPILLARS" = "true" ]; then
  [ -f "$POINTPILLARS_ENV" ] || \
    fail "$POINTPILLARS_ENV missing; run: bash $REPO_ROOT/tools/hybrid.sh setup-gpu"
  set +u
  # shellcheck disable=SC1090
  source "$POINTPILLARS_ENV"
  set -u
fi

MAP="${MAP:-$HOME/wheelchair_localization_maps/merged_0707_0725_v1/merged_0707_0725_0p20m_xyzi.pcd}"
MAP_SHA256="${MAP_SHA256:-ee317581328d3eaeee86ba448b0068c1016ca1452664b6cdaba2d874320d0431}"
ROUTE="${ROUTE:-$HOME/wheelchair_localization_src/routes/20260816_route_v9_clearance_waypoints.json}"
BAND="${BAND:-$HOME/wheelchair_localization_src/routes/20260816_route_v9_clearance_safety_band.json}"
DRIVABLE_MASK="${DRIVABLE_MASK:-$HOME/wheelchair_localization_src/routes/route_2d_map_v9.yaml}"
LATENCY_S="${LATENCY_S:-0.55}"
VN_IMU="${VN_IMU:-0}"
case "$VN_IMU" in 0) BODY_FRAME_PROFILE=builtin ;; 1) BODY_FRAME_PROFILE=vn100 ;;
  *) fail "VN_IMU must be 0 or 1" ;; esac
for asset in "$MAP" "$ROUTE" "$BAND" "$DRIVABLE_MASK"; do
  [ -f "$asset" ] || fail "required asset missing: $asset"
done
actual_map_sha="$(sha256sum "$MAP" | awk '{print $1}')"
[ "$actual_map_sha" = "$MAP_SHA256" ] || fail "runtime map SHA-256 mismatch"

# Base startup predates the RTX/hybrid nodes and cannot clean them. Remove
# orphans before it starts, otherwise an evicted process can keep a GPU context
# or command publisher alive while being invisible to rosnode.
for pattern in '[r]tx_pointpillars_node' '[v]ision_detection_bridge.py' \
               '[h]ybrid_object_fusion.py' '[s]emantic_safety_supervisor.py' \
               '[t]errain_guard.py' '[o]bstacle_clusters_geometric'; do
  pkill -f "$pattern" 2>/dev/null || true
done

say "starting the existing fail-closed field stack (paused)"
PROFILE=dwa SAFETY_POLICIES="$SAFETY_POLICIES" VN_IMU="$VN_IMU" \
  "$BASE_START"

set +u
source /opt/ros/noetic/setup.bash
source "$LOCALIZATION_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
SINGLE_THREAD_ENV="OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1"
LOG="${LOG:-$HOME}"

say "stopping only the nodes replaced by the hybrid profile"
for node in /waypoint_follower /obstacle_clusters /obstacle_clusters_geometric \
            /rtx_pointpillars /vision_detection_bridge \
            /hybrid_object_fusion /semantic_safety_supervisor \
            /terrain_guard /tip_guard; do
  rosnode kill "$node" >/dev/null 2>&1 || true
done
for pattern in '[d]wa_follower.py' '[o]bstacle_clusters.py' \
               '[r]tx_pointpillars_node' '[v]ision_detection_bridge.py' \
               '[h]ybrid_object_fusion.py' '[s]emantic_safety_supervisor.py' \
               '[t]errain_guard.py' '[t]ip_guard.py'; do
  pkill -f "$pattern" 2>/dev/null || true
done
for _ in $(seq 1 20); do
  if ! pgrep -f '[d]wa_follower.py|[o]bstacle_clusters.py|[r]tx_pointpillars_node|[v]ision_detection_bridge.py|[h]ybrid_object_fusion.py|[s]emantic_safety_supervisor.py|[t]errain_guard.py|[t]ip_guard.py' >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done
if pgrep -f '[d]wa_follower.py|[o]bstacle_clusters.py|[r]tx_pointpillars_node|[v]ision_detection_bridge.py|[h]ybrid_object_fusion.py|[s]emantic_safety_supervisor.py|[t]errain_guard.py|[t]ip_guard.py' >/dev/null 2>&1; then
  fail "a replaced motion/perception process survived shutdown"
fi
# safety_gate remains alive and fails closed while /cmd_vel_raw is absent.

say "geometric collision objects (never removed by learned inference)"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization obstacle_clusters.py \
  __name:=obstacle_clusters_geometric \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_band:="$BAND" \
  _map_path:="$MAP" \
  _map_sha256:="$MAP_SHA256" \
  /perception/objects_summary:=/perception/geometric_objects_summary \
  > "$LOG/live_geometric_objects.log" 2>&1 < /dev/null &

LEARNED_VISION_TOPIC="${LEARNED_VISION_TOPIC:-}"
LEARNED_MODEL_ID="${LEARNED_MODEL_ID:-nvidia-pointpillars-kitti-bootstrap}"
if [ "$START_POINTPILLARS" = "true" ]; then
  POINTPILLARS_MODEL="${POINTPILLARS_MODEL:?POINTPILLARS_MODEL missing from $POINTPILLARS_ENV}"
  POINTPILLARS_INPUT_TOPIC="${POINTPILLARS_INPUT_TOPIC:-/cloud_registered_body}"
  POINTPILLARS_DETECTIONS_TOPIC="${POINTPILLARS_DETECTIONS_TOPIC:-/pointpillars/detections}"
  POINTPILLARS_STATUS_TOPIC="${POINTPILLARS_STATUS_TOPIC:-/pointpillars/status}"
  POINTPILLARS_EXPECTED_FRAME="${POINTPILLARS_EXPECTED_FRAME:-body}"
  POINTPILLARS_GPU_DEVICE="${POINTPILLARS_GPU_DEVICE:-0}"
  POINTPILLARS_CONFIG="${POINTPILLARS_CONFIG:-$REPO_ROOT/src/static_livox_localization/config/pointpillars_rtx2060.yaml}"
  [ -s "$POINTPILLARS_MODEL" ] || fail "TensorRT engine missing: $POINTPILLARS_MODEL"
  [ -f "$POINTPILLARS_CONFIG" ] || fail "PointPillars config missing: $POINTPILLARS_CONFIG"
  [ -n "${POINTPILLARS_LIBRARY_PATH:-}" ] || fail "POINTPILLARS_LIBRARY_PATH missing from $POINTPILLARS_ENV"
  export LD_LIBRARY_PATH="$POINTPILLARS_LIBRARY_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

  say "RTX 2060 CUDA/TensorRT PointPillars"
  rosparam load "$POINTPILLARS_CONFIG" /rtx_pointpillars
  rosparam set /rtx_pointpillars/gpu_device "$POINTPILLARS_GPU_DEVICE"
  rosparam set /rtx_pointpillars/model_path "$POINTPILLARS_MODEL"
  rosparam set /rtx_pointpillars/input_topic "$POINTPILLARS_INPUT_TOPIC"
  rosparam set /rtx_pointpillars/detections_topic "$POINTPILLARS_DETECTIONS_TOPIC"
  rosparam set /rtx_pointpillars/status_topic "$POINTPILLARS_STATUS_TOPIC"
  rosparam set /rtx_pointpillars/expected_frame "$POINTPILLARS_EXPECTED_FRAME"
  rosparam set /rtx_pointpillars/require_rtx2060 "$POINTPILLARS_REQUIRE_RTX2060"
  setsid nohup env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
    rosrun static_livox_localization rtx_pointpillars_node \
      __name:=rtx_pointpillars \
      > "$LOG/live_pointpillars_rtx2060.log" 2>&1 < /dev/null &

  POINTPILLARS_ENV="$POINTPILLARS_ENV" \
  REQUIRE_RTX2060="$POINTPILLARS_REQUIRE_RTX2060" \
    "$SCRIPT_DIR/check_rtx2060_pointpillars.sh" 30 || {
      tail -80 "$LOG/live_pointpillars_rtx2060.log" >&2 || true
      fail "RTX PointPillars did not reach live GPU inference"
    }
  timeout 5 rostopic echo -n1 "$POINTPILLARS_DETECTIONS_TOPIC" \
    >/dev/null 2>&1 || fail "$POINTPILLARS_DETECTIONS_TOPIC is silent"
  LEARNED_VISION_TOPIC="$POINTPILLARS_DETECTIONS_TOPIC"
fi

if [ -n "$LEARNED_VISION_TOPIC" ]; then
  say "learned detection bridge ($LEARNED_MODEL_ID)"
  setsid nohup env $SINGLE_THREAD_ENV \
    rosrun static_livox_localization vision_detection_bridge.py \
    __name:=vision_detection_bridge \
    _input_topic:="$LEARNED_VISION_TOPIC" \
    _output_topic:=/perception/learned_objects_summary \
    _output_frame:=lidar \
    _body_frame_profile:="$BODY_FRAME_PROFILE" \
    _model_id:="$LEARNED_MODEL_ID" \
    > "$LOG/live_learned_bridge.log" 2>&1 < /dev/null &
else
  [ "$REQUIRE_LEARNED" = "false" ] || \
    fail "REQUIRE_LEARNED=true needs PointPillars or LEARNED_VISION_TOPIC"
  echo "  learned detector disabled: geometry remains collision authority"
fi

say "hybrid semantic fusion in chair-centre coordinates"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization hybrid_object_fusion.py \
  __name:=hybrid_object_fusion \
  _route:="$ROUTE" \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _require_learned:="$REQUIRE_LEARNED" \
  > "$LOG/live_hybrid_fusion.log" 2>&1 < /dev/null &

for _ in $(seq 1 30); do
  timeout 2 rostopic echo -n1 /perception/objects_summary \
    >/dev/null 2>&1 && break
  sleep 1
done
timeout 3 rostopic echo -n1 /perception/objects_summary \
  >/dev/null 2>&1 || fail "hybrid object summary is silent"

say "DWA produces a proposal, not a motor-authoritative command"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization dwa_follower.py \
  _route:="$ROUTE" \
  _safety_band:="$BAND" \
  _drivable_mask:="$DRIVABLE_MASK" \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_policies:="$SAFETY_POLICIES" \
  _latency_s:="$LATENCY_S" \
  _cmd_topic:=/cmd_vel_planned \
  > "$LOG/live_hybrid_dwa.log" 2>&1 < /dev/null &

say "semantic stop supervisor"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization semantic_safety_supervisor.py \
  __name:=semantic_safety_supervisor \
  > "$LOG/live_semantic_safety.log" 2>&1 < /dev/null &

say "sidewalk/mask terrain guard"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization terrain_guard.py \
  __name:=terrain_guard \
  _route:="$ROUTE" \
  _safety_band:="$BAND" \
  _drivable_mask:="$DRIVABLE_MASK" \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _cliff_required:="$CLIFF_REQUIRED" \
  _cliff_topic:="${CLIFF_TOPIC:-/terrain/cliff_status}" \
  > "$LOG/live_terrain_guard.log" 2>&1 < /dev/null &

say "final-stage relay, now downstream of terrain guard"
setsid nohup rosrun static_livox_localization tip_guard.py \
  __name:=tip_guard \
  /cmd_vel_gated:=/cmd_vel_terrain_safe \
  > "$LOG/live_tipguard.log" 2>&1 < /dev/null &

for topic in /perception/hybrid_status /semantic_safety/status \
             /terrain_guard/status /tip_guard/status; do
  for _ in $(seq 1 20); do
    timeout 2 rostopic echo -n1 "$topic" >/dev/null 2>&1 && break
    sleep 0.5
  done
  timeout 3 rostopic echo -n1 "$topic" >/dev/null 2>&1 || \
    fail "$topic is silent"
done

say "checking the complete chain while it is still paused"
READY=0
for _ in $(seq 1 30); do
  if rosrun static_livox_localization hybrid_preflight.py \
      _require_learned:="$REQUIRE_LEARNED" \
      _require_gpu_detector:="$START_POINTPILLARS" \
      _require_rtx2060:="$POINTPILLARS_REQUIRE_RTX2060" \
      _timeout_s:=3.0; then
    READY=1
    break
  fi
  sleep 1
done
[ "$READY" = "1" ] || fail "hybrid profile never became ready; inspect live_hybrid_*.log"

mkdir -p "$HOME/localization_trials"
RECORD_TOPICS=(
  /perception/geometric_objects_summary
  /perception/learned_objects_summary
  /perception/hybrid_status
  /cmd_vel_planned
  /semantic_safety/status
  /cmd_vel_terrain_safe
  /terrain_guard/status
)
if [ "$START_POINTPILLARS" = "true" ]; then
  RECORD_TOPICS+=(/pointpillars/detections /pointpillars/status)
fi
setsid nohup rosbag record --lz4 \
  -O "$HOME/localization_trials/hybrid_$(date +%Y%m%d_%H%M%S)" \
  "${RECORD_TOPICS[@]}" \
  > "$LOG/live_hybrid_blackbox.log" 2>&1 < /dev/null &

echo ""
echo "=============================================================="
echo " RTX HYBRID AVOIDANCE READY - PAUSED"
echo ""
echo "  geometry : existing MID-360 clusters, never erased by learning"
echo "  GPU      : $([ "$START_POINTPILLARS" = true ] && echo "RTX 2060 PointPillars" || echo disabled)"
echo "  semantics: ${LEARNED_VISION_TOPIC:-geometric-only}"
echo "  planner  : DWA -> /cmd_vel_planned"
echo "  guards   : semantic -> raw gate -> terrain -> tip_guard"
echo "  cliff    : required=$CLIFF_REQUIRED"
echo ""
echo "  GPU status: bash $REPO_ROOT/tools/hybrid.sh gpu-status"
echo "  start     : bash $REPO_ROOT/tools/hybrid.sh go"
echo "  stop      : bash $REPO_ROOT/tools/hybrid.sh stop"
echo "=============================================================="
