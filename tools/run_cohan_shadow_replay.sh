#!/usr/bin/env bash
# Isolated rosbag replay for advisory CoHAN/HATEB; never connects to hardware.
set -euo pipefail

REPO="${REPO:-$HOME/wheelchair_localization_src}"
FIELD_WS="${FIELD_WS:-$HOME/livox_static_localization_ws}"
COHAN_WS="${COHAN_WS:-$HOME/.cache/unicon-cohan-shadow}"
BAG="${BAG:-$HOME/localization_trials/blackbox_20260826_220341.bag}"
ROUTE="${ROUTE:-$REPO/routes/20260816_route_v9_clearance_waypoints.json}"
BAND="${BAND:-$REPO/routes/20260816_route_v9_clearance_safety_band.json}"
DRIVABLE_MASK="${DRIVABLE_MASK:-$REPO/routes/route_2d_map_v9.yaml}"
OUT="${OUT:-/tmp/ulw-cohan-shadow}"
RATE="${RATE:-5.0}"
VELOCITY_SINK="/human_aware_shadow/velocity_proposal"
OWNED_PGIDS=()
mkdir -p "$OUT"

finish() {
  local status=$?
  trap - EXIT HUP INT TERM
  for pgid in "${OWNED_PGIDS[@]}"; do
    kill -TERM -- "-$pgid" 2>/dev/null || true
  done
  {
    echo "cleanup_at=$(date -Iseconds)"
    echo "owned_process_groups=${OWNED_PGIDS[*]}"
    echo "cleanup_status=PASS"
  } > "$OUT/cleanup-receipt.txt"
  exit "$status"
}
trap finish EXIT HUP INT TERM

test -f "$BAG"
test -f "$ROUTE"
test -f "$BAND"
test -f "$DRIVABLE_MASK"
test -f "$COHAN_WS/devel/setup.bash"

set +u
source /opt/ros/noetic/setup.bash
source "$FIELD_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${LIVE_ROS_MASTER_URI:-http://127.0.0.1:11311}"
IFS= read -r LIVE_STATUS < <(
  timeout 4 rostopic echo -n1 /waypoint_follower/status/data
)
LIVE_STATUS="${LIVE_STATUS//\"/}"
IFS= read -r LIVE_V < <(timeout 4 rostopic echo -n1 /cmd_vel/linear/x)
IFS= read -r LIVE_W < <(timeout 4 rostopic echo -n1 /cmd_vel/angular/z)
test "$LIVE_STATUS" = "HOLD:PAUSED"
python3 -c \
  "assert abs(float('$LIVE_V')) + abs(float('$LIVE_W')) < 1e-9"

ROS_MASTER_PORT="$(
  python3 -c \
    'import socket; s=socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1]); s.close()'
)"
export ROS_MASTER_URI="http://127.0.0.1:$ROS_MASTER_PORT"
export ROS_IP=127.0.0.1
set +u
source "$COHAN_WS/devel/setup.bash"
set -u

setsid roscore -p "$ROS_MASTER_PORT" > "$OUT/roscore.log" 2>&1 &
OWNED_PGIDS+=("$!")
timeout 20 bash -c \
  'until rostopic list >/dev/null 2>&1; do sleep 0.1; done'
rosparam set /use_sim_time true
python3 "$REPO/tools/check_shadow_ros_graph.py" \
  > "$OUT/ros-graph-baseline.json"

setsid rosrun map_server map_server "$DRIVABLE_MASK" \
  > "$OUT/map-server.log" 2>&1 &
OWNED_PGIDS+=("$!")
setsid roslaunch static_livox_localization cohan_shadow.launch \
  > "$OUT/cohan-shadow.log" 2>&1 &
OWNED_PGIDS+=("$!")
setsid python3 "$REPO/tools/capture_cohan_shadow_replay.py" \
  --output "$OUT/replay-raw.json" \
  --band "$BAND" \
  --drivable-mask "$DRIVABLE_MASK" \
  > "$OUT/capture.log" 2>&1 &
CAPTURE_PID="$!"
OWNED_PGIDS+=("$CAPTURE_PID")

timeout 30 bash -c \
  'until rosnode ping -c1 /capture_cohan_shadow_replay >/dev/null 2>&1; do sleep 0.1; done'
python3 "$REPO/tools/check_shadow_ros_graph.py" \
  > "$OUT/ros-graph-before-replay.json"

setsid rosbag play "$BAG" --clock --rate "$RATE" \
  --topics \
  /perception/objects_summary \
  /fast_lio_icp/pose \
  /fast_lio_icp/localization_diagnostics \
  /Odometry \
  /odom \
  /tf \
  /tf_static \
  > "$OUT/rosbag-play.log" 2>&1 &
PLAYER_PID="$!"
OWNED_PGIDS+=("$PLAYER_PID")

timeout 30 rostopic echo -n1 /fast_lio_icp/pose > /dev/null
read -r GOAL_X GOAL_Y < <(
  python3 -c \
    'import json,sys; w=json.load(open(sys.argv[1]))["waypoints"][-1]; print(w["x"], w["y"])' \
    "$ROUTE"
)
rostopic pub -1 /human_aware_shadow/move_base_simple/goal \
  geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}, orientation: {w: 1.0}}}" \
  > "$OUT/goal-publish.log"

wait "$PLAYER_PID"
rostopic pub -1 /human_aware_shadow/replay_done std_msgs/Empty "{}" \
  > "$OUT/replay-done.log"
wait "$CAPTURE_PID"

python3 "$REPO/tools/validate_cohan_shadow_replay.py" \
  --input "$OUT/replay-raw.json" \
  --output "$OUT/nuc-shadow-replay.json" \
  > "$OUT/validator.log"
python3 "$REPO/tools/check_shadow_ros_graph.py" \
  > "$OUT/ros-graph-final.json"
printf "COHAN_SHADOW_REPLAY_PASS %s\n" "$OUT/nuc-shadow-replay.json"
printf "COMMAND_SINK %s\n" "$VELOCITY_SINK"
