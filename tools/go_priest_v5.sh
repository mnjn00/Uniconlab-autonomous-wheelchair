#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
"$SCRIPT_DIR/preflight_priest_v5.sh"

echo "auto mode..."
rostopic pub -1 /mode_cmd std_msgs/Int16 65 >/dev/null

echo "starting PRIEST..."
rosservice call /waypoint_follower/start "data: true"

echo "DRIVING WITH PRIEST. Stop with ~/stop.sh or move the joystick."
