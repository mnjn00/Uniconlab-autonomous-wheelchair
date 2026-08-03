#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI=http://127.0.0.1:11311

fail() { echo "REFUSING PRIEST START: $1" >&2; exit 1; }

for node in /waypoint_follower /safety_gate /tip_guard /obstacle_clusters; do
  rosnode ping -c1 "$node" >/dev/null 2>&1 \
    || fail "$node is not running"
done

for topic in /livox/lidar/header /livox/imu/header /Odometry/header \
             /fast_lio_icp/pose/header /cloud_registered_body/header \
             /perception/objects_summary; do
  timeout 4 rostopic echo -n1 "$topic" >/dev/null 2>&1 \
    || fail "$topic is silent"
done

PLANNER="$(rosparam get /waypoint_follower/planner 2>/dev/null)" \
  || fail "planner identity is missing"
[ "$PLANNER" = "priest" ] || fail "planner is '$PLANNER', not priest"

EXPECTED_ROUTE="$HOME/wheelchair_localization_src/routes/20260803_route_v5_waypoints.json"
EXPECTED_BAND="$HOME/wheelchair_localization_src/routes/20260803_route_v5_safety_band.json"
ROUTE="$(rosparam get /waypoint_follower/route 2>/dev/null)" \
  || fail "global route identity is missing"
BAND="$(rosparam get /waypoint_follower/safety_band 2>/dev/null)" \
  || fail "safety band identity is missing"
[ "$ROUTE" = "$EXPECTED_ROUTE" ] || fail "global route is '$ROUTE'"
[ "$BAND" = "$EXPECTED_BAND" ] || fail "safety band is '$BAND'"

FOLLOWER_POLICIES="$(rosparam get /waypoint_follower/safety_policies 2>/dev/null)" \
  || fail "follower safety policy identity is missing"
GATE_POLICIES="$(rosparam get /safety_gate/safety_policies 2>/dev/null)" \
  || fail "safety-gate policy identity is missing"
[ "$FOLLOWER_POLICIES" = "true" ] \
  || fail "follower safety policies are not enabled"
[ "$GATE_POLICIES" = "true" ] \
  || fail "safety-gate policies are not enabled"

STATE="$(timeout 4 rostopic echo -n1 \
  "/fast_lio_icp/localization_diagnostics/status[0]/message" 2>/dev/null \
  | head -1 | sed 's/.*message: *//' | tr -d '"')"
[ "$STATE" = "TRACKING" ] || fail "localization is '${STATE:-silent}'"
VERIFIED="$(rosparam get /fast_lio_icp/auto_initialization_verified 2>/dev/null)" \
  || fail "auto-initialization receipt is missing"
[ "$VERIFIED" = "true" ] || fail "auto initialization is not verified"

TIP_STATUS="$(timeout 4 rostopic echo -n1 /tip_guard/status 2>/dev/null \
  | sed -n 's/^data: *"\{0,1\}\([^" ]*\).*/\1/p' | head -1)"
[ "$TIP_STATUS" = "OK" ] || fail "tip guard is '${TIP_STATUS:-silent}'"
FOLLOWER_STATUS="$(timeout 4 rostopic echo -n1 /waypoint_follower/status \
  2>/dev/null | sed -n 's/^data: *"\{0,1\}\([^" ]*\).*/\1/p' | head -1)"
[ "$FOLLOWER_STATUS" = "HOLD:PAUSED" ] \
  || fail "follower is '${FOLLOWER_STATUS:-silent}', not PAUSED"

WHEEL="$(timeout 4 rostopic echo -n1 /wheel_status 2>/dev/null \
  | sed -n 's/^data: *\[\([^]]*\)\].*/\1/p' | head -1)"
MODE="$(printf '%s' "$WHEEL" | awk -F, '{gsub(/[[:space:]]/, "", $2); print $2}')"
[ "$MODE" = "77" ] || fail "wheel mode is '${MODE:-silent}', not manual"

COMMAND="$(timeout 4 rostopic echo -n1 /cmd_vel 2>/dev/null)" \
  || fail "/cmd_vel is silent"
LINEAR="$(printf '%s\n' "$COMMAND" | awk '$1 == "x:" {print $2; exit}')"
ANGULAR="$(printf '%s\n' "$COMMAND" | awk '$1 == "z:" {value=$2} END {print value}')"
[ -n "$LINEAR" ] && [ -n "$ANGULAR" ] \
  || fail "final command fields are unreadable"
awk -v linear="$LINEAR" -v angular="$ANGULAR" \
  'BEGIN {exit !((linear + 0.0 == 0.0) && (angular + 0.0 == 0.0))}' \
  || fail "non-zero final command linear=$LINEAR angular=$ANGULAR"

pgrep -af '[r]osbag record.*blackbox_' >/dev/null \
  || fail "black-box recorder is not running"

echo "PREFLIGHT OK: v5 global band, PRIEST local planner, guards ON, PAUSED/manual"
