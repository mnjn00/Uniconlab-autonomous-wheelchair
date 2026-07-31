#!/usr/bin/env bash
# Start the drive. Refuses rather than starts if anything it depends on is
# missing, because the alternative is a chair that moves while someone is
# still reading the error.
set -eo pipefail

source /opt/ros/noetic/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI=http://127.0.0.1:11311

fail() { echo "REFUSING TO START: $1" >&2; exit 1; }

rosnode ping -c1 /waypoint_follower >/dev/null 2>&1 \
  || fail "the follower is not running - bring the stack up with ~/trial_0727.sh"

# The only guard still watching for people. A silent producer would leave the
# chair driving on an empty object list, which reads exactly like clear road.
timeout 3 rostopic echo -n1 /perception/objects_summary >/dev/null 2>&1 \
  || fail "object tracking is silent (/perception/objects_summary)"

STATE="$(timeout 3 rostopic echo -n1 /fast_lio_icp/localization_diagnostics 2>/dev/null \
  | grep -m1 'message:' | sed 's/.*message: *//' | tr -d '"' || true)"
[ "$STATE" = "TRACKING" ] \
  || fail "localization is '${STATE:-silent}', not TRACKING"

echo "auto mode..."
rostopic pub -1 /mode_cmd std_msgs/Int16 65 >/dev/null

echo "starting..."
rosservice call /waypoint_follower/start "data: true"

echo ""
echo "DRIVING. Stop with ~/stop.sh, or move the joystick."
