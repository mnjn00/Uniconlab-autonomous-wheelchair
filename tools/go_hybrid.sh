#!/usr/bin/env bash
# Start motion only after the running RTX hybrid graph proves it is ready.
set -euo pipefail

set +u
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCALIZATION_WS="${LOCALIZATION_WS:-$HOME/livox_static_localization_ws}"
set +u
source "$LOCALIZATION_WS/devel/setup.bash"
set -u

fail() { echo "REFUSING TO START: $*" >&2; exit 1; }

START_POINTPILLARS="${START_POINTPILLARS:-true}"
if [ "${REQUIRE_LEARNED+x}" = x ]; then
  REQUIRE_LEARNED="$REQUIRE_LEARNED"
else
  REQUIRE_LEARNED="$START_POINTPILLARS"
fi
REQUIRE_RTX2060="${POINTPILLARS_REQUIRE_RTX2060:-true}"
for pair in "START_POINTPILLARS:$START_POINTPILLARS" \
            "REQUIRE_LEARNED:$REQUIRE_LEARNED" \
            "REQUIRE_RTX2060:$REQUIRE_RTX2060"; do
  name="${pair%%:*}"; value="${pair#*:}"
  case "$value" in true|false) ;; *) fail "$name must be true or false" ;; esac
done

for node in /waypoint_follower /hybrid_object_fusion \
            /semantic_safety_supervisor /terrain_guard /tip_guard; do
  rosnode ping -c1 "$node" >/dev/null 2>&1 || fail "$node is not running"
done
if [ "$START_POINTPILLARS" = "true" ]; then
  rosnode ping -c1 /rtx_pointpillars >/dev/null 2>&1 || \
    fail "/rtx_pointpillars is not running"
  POINTPILLARS_ENV="${POINTPILLARS_ENV:-$HOME/.config/unicon/pointpillars.env}"
  POINTPILLARS_ENV="$POINTPILLARS_ENV" \
  REQUIRE_RTX2060="$REQUIRE_RTX2060" \
    "$SCRIPT_DIR/check_rtx2060_pointpillars.sh" 5 || \
    fail "RTX 2060 PointPillars health check failed"
fi

rosrun static_livox_localization hybrid_preflight.py \
  _require_learned:="$REQUIRE_LEARNED" \
  _require_gpu_detector:="$START_POINTPILLARS" \
  _require_rtx2060:="$REQUIRE_RTX2060" \
  _timeout_s:=5.0 || fail "hybrid preflight did not pass"

# Reuse the original command ordering: every check above occurs before either
# the auto-mode command or the follower start service.
GO="${BASE_GO:-$HOME/go.sh}"
[ -x "$GO" ] || GO="$REPO_ROOT/tools/go.sh"
[ -x "$GO" ] || fail "go.sh not found"
exec "$GO"
