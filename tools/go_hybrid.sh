#!/usr/bin/env bash
# Start motion only after both RTX paths and every fail-closed guard prove ready.
set -eo pipefail

source /opt/ros/noetic/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCALIZATION_WS="${LOCALIZATION_WS:-$HOME/livox_static_localization_ws}"
source "$LOCALIZATION_WS/devel/setup.bash"

fail() { echo "REFUSING TO START: $*" >&2; exit 1; }

# Motion authorization must use the same defaults as bring-up.
. "$SCRIPT_DIR/perception_profile.sh"
REQUIRE_GPU="${REQUIRE_GPU:-true}"
POINTPILLARS_REQUIRE_RTX2060="${POINTPILLARS_REQUIRE_RTX2060:-true}"
if [ "${REQUIRE_LEARNED+x}" = x ]; then
  REQUIRE_LEARNED="$REQUIRE_LEARNED"
else
  REQUIRE_LEARNED="$START_POINTPILLARS"
fi
for pair in "START_POINTPILLARS:$START_POINTPILLARS" \
            "REQUIRE_LEARNED:$REQUIRE_LEARNED" \
            "REQUIRE_GPU:$REQUIRE_GPU" \
            "POINTPILLARS_REQUIRE_RTX2060:$POINTPILLARS_REQUIRE_RTX2060"; do
  name="${pair%%:*}"; value="${pair#*:}"
  case "$value" in true|false) ;; *) fail "$name must be true or false" ;; esac
done

for node in /waypoint_follower /hybrid_geometric_objects \
            /hybrid_object_fusion /localization_exclusion_boxes \
            /semantic_safety_supervisor /safety_gate \
            /terrain_guard /tip_guard; do
  rosnode ping -c1 "$node" >/dev/null 2>&1 || fail "$node is not running"
done

if [ "$REQUIRE_GPU" = "true" ]; then
  REQUIRE_RTX2060="$POINTPILLARS_REQUIRE_RTX2060" \
    "$SCRIPT_DIR/check_nuc_gpu_dwa.sh" 5 || \
    fail "RTX/CuPy DWA health check failed"
fi

if [ "$START_POINTPILLARS" = "true" ]; then
  rosnode ping -c1 /rtx_pointpillars >/dev/null 2>&1 || \
    fail "/rtx_pointpillars is not running"
  POINTPILLARS_ENV="${POINTPILLARS_ENV:-$HOME/.config/unicon/pointpillars.env}"
  POINTPILLARS_ENV="$POINTPILLARS_ENV" \
  REQUIRE_RTX2060="$POINTPILLARS_REQUIRE_RTX2060" \
    "$SCRIPT_DIR/check_rtx2060_pointpillars.sh" 5 || \
    fail "RTX 2060 PointPillars health check failed"
fi

rosrun static_livox_localization hybrid_preflight.py \
  _require_learned:="$REQUIRE_LEARNED" \
  _require_gpu_detector:="$START_POINTPILLARS" \
  _require_rtx2060:="$POINTPILLARS_REQUIRE_RTX2060" \
  _require_gpu_dwa:="$REQUIRE_GPU" \
  _timeout_s:=5.0 || fail "hybrid preflight did not pass"

# The ordinary hybrid preflight cannot distinguish the old stop-only person
# supervisor from the stationary-person branch: both use the same node names
# and topics. This contract verifies the permit heartbeat, the target-aware
# semantic implementation, and the curved-trajectory raw gate.
rosrun static_livox_localization person_bypass_preflight.py \
  _timeout_s:=5.0 _maximum_permit_age_s:=0.60 || \
  fail "stationary-person trajectory-bypass preflight did not pass"

# Reuse the original command ordering: every check above occurs before either
# the auto-mode command or the follower start service.
GO="${BASE_GO:-$HOME/go.sh}"
[ -x "$GO" ] || GO="$REPO_ROOT/tools/go.sh"
[ -x "$GO" ] || fail "go.sh not found"
exec "$GO"
