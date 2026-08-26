#!/usr/bin/env bash
# Start motion only after the running hybrid graph proves it is ready.
set -euo pipefail

source /opt/ros/noetic/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCALIZATION_WS="${LOCALIZATION_WS:-$HOME/livox_static_localization_ws}"
source "$LOCALIZATION_WS/devel/setup.bash"

fail() { echo "REFUSING TO START: $*" >&2; exit 1; }

for node in /waypoint_follower /hybrid_object_fusion \
            /semantic_safety_supervisor /terrain_guard /tip_guard; do
  rosnode ping -c1 "$node" >/dev/null 2>&1 || fail "$node is not running"
done

REQUIRE_LEARNED="${REQUIRE_LEARNED:-false}"
case "$REQUIRE_LEARNED" in true|false) ;; *) fail "REQUIRE_LEARNED must be true or false" ;; esac

rosrun static_livox_localization hybrid_preflight.py \
  _require_learned:="$REQUIRE_LEARNED" _timeout_s:=5.0 || \
  fail "hybrid preflight did not pass"

# Reuse the original command ordering: every check above occurs before either
# the auto-mode command or the follower start service.
GO="${BASE_GO:-$HOME/go.sh}"
[ -x "$GO" ] || GO="$REPO_ROOT/tools/go.sh"
[ -x "$GO" ] || fail "go.sh not found"
exec "$GO"
