#!/usr/bin/env bash
# Bring up the reviewed field stack, then replace only perception/control
# adapters with the hybrid profile. The original startup remains the rollback.
set -eo pipefail

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  cat <<'EOF'
Usage: start_hybrid_avoidance.sh

Starts the existing ROS1 field stack with PROFILE=dwa, then installs:
  all non-ground geometry -> optional learned fusion -> RTX DWA planned command
  -> semantic stop supervisor -> raw safety gate -> terrain guard
  -> tip_guard -> wheelchair base

Environment:
  REQUIRE_GPU=true|false                         # default true on Phantom Canyon
  REQUIRE_LEARNED=false|true
  LEARNED_VISION_TOPIC=/pointpillars/detections # optional vision_msgs input
  LEARNED_MODEL_ID=pointpillars-mid360-v1
  GEOMETRIC_MIN_CELL_POINTS=1                   # raw-gate-aligned thin objects
  GEOMETRIC_MIN_CLUSTER_POINTS=5                # safety gate also requires 5
  GEOMETRIC_MAX_CLUSTERS=80
  CLIFF_REQUIRED=false|true                     # downward sensor contract
  CLIFF_TOPIC=/terrain/cliff_status
  SAFETY_POLICIES=true|false
  VN_IMU=0|1

Run tools/install_nuc_gpu_runtime.sh once if CuPy is not installed.
Nothing moves until go_hybrid.sh is run. Existing stop.sh remains authoritative.
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

REQUIRE_GPU="${REQUIRE_GPU:-true}"
REQUIRE_LEARNED="${REQUIRE_LEARNED:-false}"
CLIFF_REQUIRED="${CLIFF_REQUIRED:-false}"
SAFETY_POLICIES="${SAFETY_POLICIES:-true}"
for pair in "REQUIRE_GPU:$REQUIRE_GPU" \
            "REQUIRE_LEARNED:$REQUIRE_LEARNED" \
            "CLIFF_REQUIRED:$CLIFF_REQUIRED" \
            "SAFETY_POLICIES:$SAFETY_POLICIES"; do
  name="${pair%%:*}"; value="${pair#*:}"
  case "$value" in true|false) ;; *) fail "$name must be true or false" ;; esac
done

GEOMETRIC_MIN_CELL_POINTS="${GEOMETRIC_MIN_CELL_POINTS:-1}"
GEOMETRIC_MIN_CLUSTER_POINTS="${GEOMETRIC_MIN_CLUSTER_POINTS:-5}"
GEOMETRIC_MAX_CLUSTERS="${GEOMETRIC_MAX_CLUSTERS:-80}"
for pair in "GEOMETRIC_MIN_CELL_POINTS:$GEOMETRIC_MIN_CELL_POINTS" \
            "GEOMETRIC_MIN_CLUSTER_POINTS:$GEOMETRIC_MIN_CLUSTER_POINTS" \
            "GEOMETRIC_MAX_CLUSTERS:$GEOMETRIC_MAX_CLUSTERS"; do
  name="${pair%%:*}"; value="${pair#*:}"
  case "$value" in
    ''|*[!0-9]*) fail "$name must be a positive integer" ;;
  esac
  [ "$value" -gt 0 ] || fail "$name must be a positive integer"
done
[ "$GEOMETRIC_MIN_CLUSTER_POINTS" -ge "$GEOMETRIC_MIN_CELL_POINTS" ] || \
  fail "GEOMETRIC_MIN_CLUSTER_POINTS must be >= GEOMETRIC_MIN_CELL_POINTS"

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

say "probing the Phantom Canyon RTX runtime"
if command -v nvidia-smi >/dev/null 2>&1 && python3 - <<'PY'
import cupy as cp
props = cp.cuda.runtime.getDeviceProperties(0)
name = props["name"]
if isinstance(name, bytes):
    name = name.decode("utf-8", "replace")
x = cp.arange(4096, dtype=cp.float32)
value = float((x * 2.0).sum())
expected = 4095.0 * 4096.0
if abs(value - expected) / expected > 2e-6:
    raise SystemExit("CUDA arithmetic mismatch")
cp.cuda.Device().synchronize()
print("  CUDA device: %s" % name)
PY
then
  GPU_READY=true
else
  GPU_READY=false
fi
if [ "$GPU_READY" != "true" ]; then
  if [ "$REQUIRE_GPU" = "true" ]; then
    fail "RTX/CuPy runtime unavailable; run $REPO_ROOT/tools/install_nuc_gpu_runtime.sh"
  fi
  echo "WARNING: GPU unavailable; DWA will use its slower CPU fallback" >&2
fi

# Base startup predates these nodes and cannot clean them. Remove orphaned
# hybrid processes before it starts; otherwise an old supervisor or terrain
# guard may keep publishing under an evicted ROS name and consume whole cores.
pkill -f '[g]pu_dwa_follower.py' 2>/dev/null || true
pkill -f '[h]ybrid_geometric_objects.py' 2>/dev/null || true
pkill -f '[h]ybrid_object_fusion.py' 2>/dev/null || true
pkill -f '[v]ision_detection_bridge.py' 2>/dev/null || true
pkill -f '[l]ocalization_exclusion_boxes.py' 2>/dev/null || true
pkill -f '[s]emantic_safety_supervisor.py' 2>/dev/null || true
pkill -f '[t]errain_guard.py' 2>/dev/null || true

say "starting the existing fail-closed field stack (paused)"
PROFILE=dwa SAFETY_POLICIES="$SAFETY_POLICIES" VN_IMU="$VN_IMU" \
  "$BASE_START"

source /opt/ros/noetic/setup.bash
source "$LOCALIZATION_WS/devel/setup.bash"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
SINGLE_THREAD_ENV="OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1"
LOG="${LOG:-$HOME}"

say "stopping only the nodes replaced by the hybrid profile"
for node in /waypoint_follower /obstacle_clusters /tip_guard \
            /hybrid_geometric_objects /hybrid_object_fusion \
            /vision_detection_bridge /localization_exclusion_boxes \
            /semantic_safety_supervisor /terrain_guard; do
  rosnode kill "$node" >/dev/null 2>&1 || true
done
for pattern in '[g]pu_dwa_follower.py' '[d]wa_follower.py' \
               '[o]bstacle_clusters.py' \
               '[h]ybrid_geometric_objects.py' '[h]ybrid_object_fusion.py' \
               '[v]ision_detection_bridge.py' \
               '[l]ocalization_exclusion_boxes.py' \
               '[s]emantic_safety_supervisor.py' '[t]errain_guard.py' \
               '[t]ip_guard.py'; do
  pkill -f "$pattern" 2>/dev/null || true
done
for _ in $(seq 1 20); do
  if ! pgrep -f '[g]pu_dwa_follower.py|[d]wa_follower.py|[o]bstacle_clusters.py|[h]ybrid_geometric_objects.py|[h]ybrid_object_fusion.py|[v]ision_detection_bridge.py|[l]ocalization_exclusion_boxes.py|[s]emantic_safety_supervisor.py|[t]errain_guard.py|[t]ip_guard.py' >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done
if pgrep -f '[g]pu_dwa_follower.py|[d]wa_follower.py|[o]bstacle_clusters.py|[h]ybrid_geometric_objects.py|[h]ybrid_object_fusion.py|[v]ision_detection_bridge.py|[l]ocalization_exclusion_boxes.py|[s]emantic_safety_supervisor.py|[t]errain_guard.py|[t]ip_guard.py' >/dev/null 2>&1; then
  fail "a replaced hybrid motion/perception node survived shutdown"
fi
# safety_gate remains alive and fails closed while /cmd_vel_raw is absent.

say "all non-ground collision geometry (mapped surfaces retained)"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization hybrid_geometric_objects.py \
  __name:=hybrid_geometric_objects \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_band:="$BAND" \
  _map_path:="$MAP" \
  _map_sha256:="$MAP_SHA256" \
  _min_cell_points:="$GEOMETRIC_MIN_CELL_POINTS" \
  _min_cluster_points:="$GEOMETRIC_MIN_CLUSTER_POINTS" \
  _max_clusters:="$GEOMETRIC_MAX_CLUSTERS" \
  /perception/objects_summary:=/perception/geometric_objects_summary \
  /perception/dynamic_boxes:=/perception/geometric_exclusion_candidates \
  > "$LOG/live_geometric_objects.log" 2>&1 < /dev/null &

LEARNED_VISION_TOPIC="${LEARNED_VISION_TOPIC:-}"
LEARNED_MODEL_ID="${LEARNED_MODEL_ID:-pointpillars-mid360}"
if [ -n "$LEARNED_VISION_TOPIC" ]; then
  say "learned 3D detections ($LEARNED_MODEL_ID)"
  setsid nohup env $SINGLE_THREAD_ENV \
    rosrun static_livox_localization vision_detection_bridge.py \
    _input_topic:="$LEARNED_VISION_TOPIC" \
    _output_topic:=/perception/learned_objects_summary \
    _output_frame:=lidar \
    _body_frame_profile:="$BODY_FRAME_PROFILE" \
    _model_id:="$LEARNED_MODEL_ID" \
    > "$LOG/live_learned_bridge.log" 2>&1 < /dev/null &
else
  [ "$REQUIRE_LEARNED" = "false" ] || \
    fail "REQUIRE_LEARNED=true needs LEARNED_VISION_TOPIC"
  echo "  no learned topic supplied: geometric detection remains authoritative"
fi

say "hybrid semantic fusion in chair-centre coordinates"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization hybrid_object_fusion.py \
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

say "selective localization exclusions"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization localization_exclusion_boxes.py \
  > "$LOG/live_localization_exclusions.log" 2>&1 < /dev/null &

say "RTX-accelerated DWA produces a proposal, not a motor-authoritative command"
setsid nohup env $SINGLE_THREAD_ENV \
  WHEELCHAIR_DWA_GPU=1 WHEELCHAIR_REQUIRE_GPU="$REQUIRE_GPU" \
  rosrun static_livox_localization gpu_dwa_follower.py \
  _route:="$ROUTE" \
  _safety_band:="$BAND" \
  _drivable_mask:="$DRIVABLE_MASK" \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_policies:="$SAFETY_POLICIES" \
  _latency_s:="$LATENCY_S" \
  _prefer_gpu:=true \
  _require_gpu:="$REQUIRE_GPU" \
  _cmd_topic:=/cmd_vel_planned \
  > "$LOG/live_hybrid_dwa.log" 2>&1 < /dev/null &

say "semantic stop supervisor"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization semantic_safety_supervisor.py \
  > "$LOG/live_semantic_safety.log" 2>&1 < /dev/null &

say "sidewalk/mask terrain guard"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization terrain_guard.py \
  _route:="$ROUTE" \
  _safety_band:="$BAND" \
  _drivable_mask:="$DRIVABLE_MASK" \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _cliff_required:="$CLIFF_REQUIRED" \
  _cliff_topic:="${CLIFF_TOPIC:-/terrain/cliff_status}" \
  > "$LOG/live_terrain_guard.log" 2>&1 < /dev/null &

say "final-stage relay, now downstream of terrain guard"
setsid nohup rosrun static_livox_localization tip_guard.py \
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
      _require_gpu:="$REQUIRE_GPU" \
      _require_learned:="$REQUIRE_LEARNED" _timeout_s:=3.0; then
    READY=1
    break
  fi
  sleep 1
done
[ "$READY" = "1" ] || fail "hybrid profile never became ready; inspect live_hybrid_*.log"

# Existing black box already records fused /perception/objects_summary and
# the authoritative raw/gated/final commands. This small companion bag keeps
# the new intermediate evidence without duplicating the point cloud.
mkdir -p "$HOME/localization_trials"
setsid nohup rosbag record --lz4 \
  -O "$HOME/localization_trials/hybrid_$(date +%Y%m%d_%H%M%S)" \
  /perception/geometric_objects_summary \
  /perception/learned_objects_summary \
  /perception/hybrid_status \
  /perception/dynamic_boxes \
  /cmd_vel_planned /semantic_safety/status \
  /cmd_vel_terrain_safe /terrain_guard/status \
  > "$LOG/live_hybrid_blackbox.log" 2>&1 < /dev/null &

echo ""
echo "=============================================================="
echo " HYBRID AVOIDANCE READY - PAUSED"
echo ""
echo "  geometry : all non-ground MID-360 clusters; map subtraction disabled"
echo "  threshold: cell=$GEOMETRIC_MIN_CELL_POINTS cluster=$GEOMETRIC_MIN_CLUSTER_POINTS max=$GEOMETRIC_MAX_CLUSTERS"
echo "  semantics: ${LEARNED_VISION_TOPIC:-geometric-only}"
echo "  planner  : RTX/CuPy DWA -> /cmd_vel_planned (required=$REQUIRE_GPU)"
echo "  guards   : semantic -> raw gate -> terrain -> tip_guard"
echo "  cliff    : required=$CLIFF_REQUIRED"
echo ""
echo "  start:  REQUIRE_GPU=$REQUIRE_GPU bash $REPO_ROOT/tools/hybrid.sh go"
echo "  stop :  bash $REPO_ROOT/tools/hybrid.sh stop"
echo "=============================================================="
