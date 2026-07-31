#!/usr/bin/env bash
# Stop the drive. Deliberately does not check anything first - a stop that
# refuses because a precondition failed is not a stop.
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI=http://127.0.0.1:11311

rosservice call /waypoint_follower/start "data: false" 2>/dev/null || true
rostopic pub -1 /mode_cmd std_msgs/Int16 77 >/dev/null 2>&1 || true
echo "STOPPED (follower paused, base out of auto mode)."
