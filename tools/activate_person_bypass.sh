#!/usr/bin/env bash
# Replace the stop-only person policy and fixed-corridor raw gate while the
# hybrid stack is paused. The existing perception, localization, terrain,
# tip/UART, and RTX PointPillars processes remain in place.
set -eo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCALIZATION_WS="${LOCALIZATION_WS:-$HOME/livox_static_localization_ws}"
LOG="${LOG:-$HOME}"
MODE="${1:-activate}"

fail() { echo "ERROR: $*" >&2; exit 1; }
say() { printf '\n=== %s ===\n' "$1"; }

set +u
source /opt/ros/noetic/setup.bash
[ -f "$LOCALIZATION_WS/devel/setup.bash" ] || \
  fail "localization workspace is not built: $LOCALIZATION_WS"
source "$LOCALIZATION_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

check_only() {
  rosrun static_livox_localization person_bypass_preflight.py \
    _timeout_s:="${PERSON_BYPASS_PREFLIGHT_TIMEOUT_S:-5.0}" \
    _maximum_permit_age_s:="${PERSON_BYPASS_PREFLIGHT_MAX_AGE_S:-0.60}"
}

case "$MODE" in
  --check|check)
    check_only
    exit 0
    ;;
  activate|"") ;;
  *) fail "usage: activate_person_bypass.sh [activate|--check]" ;;
esac

for node in /waypoint_follower /semantic_safety_supervisor /safety_gate; do
  rosnode ping -c1 "$node" >/dev/null 2>&1 || \
    fail "$node is not running; start the base hybrid stack first"
done

param() {
  local name="$1"
  rosparam get "$name" 2>/dev/null || fail "missing ROS parameter: $name"
}

ROUTE="$(param /waypoint_follower/route)"
BAND="$(param /waypoint_follower/safety_band)"
DRIVABLE_MASK="$(param /waypoint_follower/drivable_mask)"
BODY_FRAME_PROFILE="$(param /waypoint_follower/body_frame_profile)"
SAFETY_POLICIES="$(param /waypoint_follower/safety_policies)"
LATENCY_S="$(rosparam get /waypoint_follower/latency_s 2>/dev/null || echo 0.55)"
REQUIRE_GPU="${REQUIRE_GPU:-$(rosparam get /waypoint_follower/require_gpu 2>/dev/null || echo true)}"
PREFER_GPU="${PREFER_DWA_GPU:-$(rosparam get /waypoint_follower/prefer_gpu 2>/dev/null || echo true)}"
for value in "$ROUTE" "$BAND" "$DRIVABLE_MASK"; do
  [ -f "$value" ] || fail "runtime asset disappeared: $value"
done
for pair in "SAFETY_POLICIES:$SAFETY_POLICIES" \
            "REQUIRE_GPU:$REQUIRE_GPU" "PREFER_GPU:$PREFER_GPU"; do
  name="${pair%%:*}"; value="${pair#*:}"
  case "$value" in true|false) ;; *) fail "$name must be true or false (got $value)" ;; esac
done

PERSON_BYPASS_CONFIRM_S="${PERSON_BYPASS_CONFIRM_S:-3.0}"
PERSON_BYPASS_MAX_GAP_S="${PERSON_BYPASS_MAX_GAP_S:-0.45}"
PERSON_BYPASS_MAX_JUMP_M="${PERSON_BYPASS_MAX_JUMP_M:-0.35}"
PERSON_BYPASS_PERMIT_LIFETIME_S="${PERSON_BYPASS_PERMIT_LIFETIME_S:-0.45}"
PERSON_BYPASS_MAX_FORWARD_M="${PERSON_BYPASS_MAX_FORWARD_M:-8.0}"
PERSON_BYPASS_MAX_LATERAL_M="${PERSON_BYPASS_MAX_LATERAL_M:-1.0}"
PERSON_BYPASS_LATERAL_HYSTERESIS_M="${PERSON_BYPASS_LATERAL_HYSTERESIS_M:-0.25}"
PERSON_BYPASS_MIN_NEAR_M="${PERSON_BYPASS_MIN_NEAR_M:-0.60}"
PERSON_BYPASS_SPEED_MPS="${PERSON_BYPASS_SPEED_MPS:-0.35}"
PERSON_BYPASS_CLEARANCE_M="${PERSON_BYPASS_CLEARANCE_M:-0.50}"
PERSON_BYPASS_MIN_TURN_RPS="${PERSON_BYPASS_MIN_TURN_RPS:-0.08}"

say "replacing stop-only person policy while the chair remains paused"
for node in /waypoint_follower /semantic_safety_supervisor /safety_gate; do
  rosnode kill "$node" >/dev/null 2>&1 || true
done
for pattern in \
    '[p]erson_bypass_dwa_follower.py' '[g]pu_dwa_follower.py' '[d]wa_follower.py' \
    '[p]erson_bypass_semantic_supervisor.py' '[s]emantic_safety_supervisor.py' \
    '[t]rajectory_safety_gate.py' '[s]afety_gate.py'; do
  pkill -f "$pattern" >/dev/null 2>&1 || true
done
for _ in $(seq 1 20); do
  if ! pgrep -f '[p]erson_bypass_dwa_follower.py|[g]pu_dwa_follower.py|[d]wa_follower.py|[p]erson_bypass_semantic_supervisor.py|[s]emantic_safety_supervisor.py|[t]rajectory_safety_gate.py|[s]afety_gate.py' >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done
if pgrep -f '[p]erson_bypass_dwa_follower.py|[g]pu_dwa_follower.py|[d]wa_follower.py|[p]erson_bypass_semantic_supervisor.py|[s]emantic_safety_supervisor.py|[t]rajectory_safety_gate.py|[s]afety_gate.py' >/dev/null 2>&1; then
  fail "a replaced control process survived shutdown"
fi

SINGLE_THREAD_ENV="OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1"

# Bring the raw gate back first. Until the supervisor and follower follow it,
# input staleness keeps this output at zero.
say "trajectory-aware raw LiDAR safety gate"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization trajectory_safety_gate.py \
  __name:=safety_gate \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_policies:="$SAFETY_POLICIES" \
  _maximum_person_bypass_permit_age_s:="$PERSON_BYPASS_PERMIT_LIFETIME_S" \
  _minimum_person_bypass_turn_rps:="$PERSON_BYPASS_MIN_TURN_RPS" \
  > "$LOG/live_trajectory_safety_gate.log" 2>&1 < /dev/null &

say "RTX DWA with same-track static-person qualification"
setsid nohup env $SINGLE_THREAD_ENV \
  WHEELCHAIR_DWA_GPU="$([ "$PREFER_GPU" = true ] && echo 1 || echo 0)" \
  WHEELCHAIR_REQUIRE_GPU="$([ "$REQUIRE_GPU" = true ] && echo 1 || echo 0)" \
  rosrun static_livox_localization person_bypass_dwa_follower.py \
  __name:=waypoint_follower \
  _route:="$ROUTE" \
  _safety_band:="$BAND" \
  _drivable_mask:="$DRIVABLE_MASK" \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_policies:="$SAFETY_POLICIES" \
  _latency_s:="$LATENCY_S" \
  _prefer_gpu:="$PREFER_GPU" \
  _require_gpu:="$REQUIRE_GPU" \
  _cmd_topic:=/cmd_vel_planned \
  _accepted_cmd_topic:=/cmd_vel \
  _person_bypass_confirmation_s:="$PERSON_BYPASS_CONFIRM_S" \
  _person_bypass_maximum_gap_s:="$PERSON_BYPASS_MAX_GAP_S" \
  _person_bypass_position_jump_m:="$PERSON_BYPASS_MAX_JUMP_M" \
  _person_bypass_permit_lifetime_s:="$PERSON_BYPASS_PERMIT_LIFETIME_S" \
  _person_bypass_maximum_forward_m:="$PERSON_BYPASS_MAX_FORWARD_M" \
  _person_bypass_maximum_lateral_m:="$PERSON_BYPASS_MAX_LATERAL_M" \
  _person_bypass_lateral_hysteresis_m:="$PERSON_BYPASS_LATERAL_HYSTERESIS_M" \
  _person_bypass_minimum_near_m:="$PERSON_BYPASS_MIN_NEAR_M" \
  _person_bypass_speed_mps:="$PERSON_BYPASS_SPEED_MPS" \
  _person_bypass_clearance_m:="$PERSON_BYPASS_CLEARANCE_M" \
  > "$LOG/live_person_bypass_dwa.log" 2>&1 < /dev/null &

say "semantic supervisor with target-only static-person exception"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization person_bypass_semantic_supervisor.py \
  __name:=semantic_safety_supervisor \
  _maximum_person_bypass_permit_age_s:="$PERSON_BYPASS_PERMIT_LIFETIME_S" \
  _person_bypass_maximum_forward_m:="$PERSON_BYPASS_MAX_FORWARD_M" \
  _person_bypass_maximum_lateral_m:="$PERSON_BYPASS_MAX_LATERAL_M" \
  _person_bypass_lateral_hysteresis_m:="$PERSON_BYPASS_LATERAL_HYSTERESIS_M" \
  > "$LOG/live_person_bypass_semantic.log" 2>&1 < /dev/null &

for node in /safety_gate /waypoint_follower /semantic_safety_supervisor; do
  for _ in $(seq 1 30); do
    rosnode ping -c1 "$node" >/dev/null 2>&1 && break
    sleep 0.25
  done
  rosnode ping -c1 "$node" >/dev/null 2>&1 || {
    tail -80 "$LOG/live_trajectory_safety_gate.log" >&2 || true
    tail -80 "$LOG/live_person_bypass_dwa.log" >&2 || true
    tail -80 "$LOG/live_person_bypass_semantic.log" >&2 || true
    fail "$node did not restart"
  }
done

for topic in /person_bypass/permit /semantic_safety/status /safety_gate/status; do
  timeout 5 rostopic echo -n1 "$topic" >/dev/null 2>&1 || \
    fail "$topic is silent"
done
check_only || fail "stationary-person bypass preflight failed"

mkdir -p "$HOME/localization_trials"
setsid nohup rosbag record --lz4 \
  -O "$HOME/localization_trials/person_bypass_$(date +%Y%m%d_%H%M%S)" \
  /person_bypass/permit /waypoint_follower/status \
  /semantic_safety/status /safety_gate/status \
  /cmd_vel_planned /cmd_vel_raw /cmd_vel_gated \
  > "$LOG/live_person_bypass_blackbox.log" 2>&1 < /dev/null &

cat <<EOF

==============================================================
 STATIC-THREAT TRAJECTORY BYPASS READY - PAUSED

  moving/unknown person : WAIT
  one static person     : qualify ${PERSON_BYPASS_CONFIRM_S}s, then RTX DWA
  tracked static object : RTX DWA with raw-trajectory permit
  moving/unknown object : WAIT
  bypass speed          : <= ${PERSON_BYPASS_SPEED_MPS} m/s
  person clearance      : >= ${PERSON_BYPASS_CLEARANCE_M} m
  raw gate              : fixed corridor replaced only by clear curved sweep
  terrain/tip/UART      : unchanged and still downstream

  check : bash $REPO_ROOT/tools/activate_person_bypass.sh --check
  go    : bash $REPO_ROOT/tools/hybrid.sh go
  stop  : bash $REPO_ROOT/tools/hybrid.sh stop
==============================================================
EOF
