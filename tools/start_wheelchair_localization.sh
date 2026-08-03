#!/usr/bin/env bash
# One-command field startup: driver -> FAST-LIO -> localization(+RViz) -> auto seed.
set -eo pipefail

# Map merged from the 07/07 + 07/25 passes. The 594 MB mergedmap.ply is the
# immutable source; deploy_merged_map.sh verifies it and installs the pinned
# 0.20 m runtime PCD used by both auto-init and ICP.
# A map override is fail-closed: MAP, MAP_SHA256, MAP_ID, and TRAJ must be
# supplied together so every localization and preview node sees one identity.
MAP_OVERRIDE_COUNT=0
[ "${MAP+x}" = "x" ] && MAP_OVERRIDE_COUNT=$((MAP_OVERRIDE_COUNT + 1))
[ "${MAP_SHA256+x}" = "x" ] && MAP_OVERRIDE_COUNT=$((MAP_OVERRIDE_COUNT + 1))
[ "${MAP_ID+x}" = "x" ] && MAP_OVERRIDE_COUNT=$((MAP_OVERRIDE_COUNT + 1))
[ "${TRAJ+x}" = "x" ] && MAP_OVERRIDE_COUNT=$((MAP_OVERRIDE_COUNT + 1))
if [ "$MAP_OVERRIDE_COUNT" -ne 0 ] && [ "$MAP_OVERRIDE_COUNT" -ne 4 ]; then
  echo "ERROR: override MAP, MAP_SHA256, MAP_ID, and TRAJ together" >&2
  exit 64
fi
MAP="${MAP:-$HOME/wheelchair_localization_maps/merged_0707_0725_v1/merged_0707_0725_0p20m_xyzi.pcd}"
MAP_SHA256="${MAP_SHA256:-ee317581328d3eaeee86ba448b0068c1016ca1452664b6cdaba2d874320d0431}"
MAP_ID="${MAP_ID:-merged_0707_0725_v1}"
TRAJ="${TRAJ:-$HOME/wheelchair_localization_maps/merged_0707_0725_v1/traj_lidar.txt}"
ROUTE="${ROUTE:-$HOME/wheelchair_localization_src/routes/20260803_route_v5_waypoints.json}"
BAND="${BAND:-$HOME/wheelchair_localization_src/routes/20260803_route_v5_safety_band.json}"
RVIZ="${RVIZ:-true}"
# SAFETY_POLICIES=false drives with every discretionary guard switched off,
# leaving the joystick override as the failsafe. It exists to measure one
# thing per run without another guard ending the measurement first. Only the
# two literals are accepted: a typo here must not silently be truthy, and a
# rosparam bool will not take "0" or "no" anyway.
SAFETY_POLICIES="${SAFETY_POLICIES:-true}"
if [ "$SAFETY_POLICIES" != "true" ] && [ "$SAFETY_POLICIES" != "false" ]; then
  echo "ERROR: SAFETY_POLICIES must be true or false, got '$SAFETY_POLICIES'" >&2
  exit 65
fi
# PLANNER=priest swaps the route follower for the PRIEST corridor planner
# (field-trial opt-in; same topics, service and status contract). The
# field-validated route follower stays the default, and the PRIEST node
# refuses SAFETY_POLICIES=false outright - an unvalidated planner with its
# guards off is not a diagnostic configuration.
PLANNER="${PLANNER:-route}"
if [ "$PLANNER" != "route" ] && [ "$PLANNER" != "priest" ]; then
  echo "ERROR: PLANNER must be route or priest, got '$PLANNER'" >&2
  exit 66
fi
LOG=$HOME

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export DISPLAY="${DISPLAY:-:0}"

echo "[0/5] display"
XAUTHORITY="$HOME/.Xauthority" xrandr --output HDMI-1 --mode 1920x1080 2>/dev/null || true

# VNC is opt-in, password-file authenticated, and loopback-only by default.
# Create the credential once with: x11vnc -storepasswd ~/.vnc/passwd
# Connect through a tunnel: ssh -L 5900:127.0.0.1:5900 mprp3@<nuc>
VNC="${VNC:-0}"
VNC_AUTH="${VNC_AUTH:-$HOME/.vnc/passwd}"
if [ "$VNC" = "1" ]; then
  if [ ! -f "$VNC_AUTH" ]; then
    echo "ERROR: VNC requested but auth file is missing: $VNC_AUTH" >&2
    exit 7
  fi
  VNC_BIND="-localhost"
  VNC_WHERE="127.0.0.1 only"
  if [ "${VNC_ALLOW_REMOTE:-0}" = "1" ]; then
    VNC_BIND=""
    VNC_WHERE="all interfaces"
    echo "WARNING: VNC is exposed to the network" >&2
  fi
  if ! pgrep -x x11vnc >/dev/null; then
    # VNC_BIND is intentionally either one trusted option or empty.
    # shellcheck disable=SC2086
    setsid nohup x11vnc -display :0 -auth guess -rfbauth "$VNC_AUTH" \
      $VNC_BIND -forever -shared -repeat -wait 15 -defer 15 \
      -o "$HOME/x11vnc.log" -bg >/dev/null 2>&1 < /dev/null || true
  fi
  echo "  vnc on port 5900 ($VNC_WHERE)"
else
  echo "  vnc disabled (set VNC=1 after creating $VNC_AUTH)"
fi

echo "[1/5] cleaning old processes"
for pattern in '[r]oslaunch' '[r]osbag record' '[f]astlio_mapping' '[a]uto_initial_pose' '[s]afety_gate' '[t]ip_guard' '[w]aypoint_follower'; do
  pkill -f "$pattern" 2>/dev/null || true
done
sleep 2
if ! pgrep -f '[r]osmaster' >/dev/null; then
  setsid nohup roscore > "$LOG/live_roscore.log" 2>&1 < /dev/null &
  sleep 4
fi
rosparam set /use_sim_time false

echo "[2/5] livox driver"
source "$HOME/ws_livox/devel/setup.bash"
setsid nohup roslaunch livox_ros_driver2 msg_MID360.launch \
  > "$LOG/live_livox.log" 2>&1 < /dev/null &
for i in $(seq 1 30); do
  timeout 3 rostopic echo -n1 /livox/lidar/header >/dev/null 2>&1 && break
  sleep 2
done
if ! timeout 3 rostopic echo -n1 /livox/lidar/header >/dev/null 2>&1; then
  echo "ERROR: /livox/lidar not publishing (lidar power/cable?)"; exit 2
fi
echo "  lidar OK"

# The 0727 route was recorded with the MID-360's built-in IMU. Keep that
# sensor/profile as the default; VN_IMU=1 remains an explicit override.
VN_IMU="${VN_IMU:-0}"
if [ "$VN_IMU" = "1" ]; then
  echo "[2b/5] VectorNav VN-100"
  # A SIGTERM'd vnpub leaves the sensor streaming binary at 921600; the
  # next driver start parses that backlog as register replies and
  # segfaults. Silence async output before opening it.
  VN_RESET=""
  for candidate in "$(dirname "$0")/vn_reset.py" "$HOME/vn_reset.py"; do
    [ -f "$candidate" ] && { VN_RESET="$candidate"; break; }
  done
  if [ -n "$VN_RESET" ]; then
    python3 "$VN_RESET" 2>&1 | sed 's/^/  vn_reset: /' || \
      echo "  vn_reset failed - continuing, driver may still negotiate"
  else
    echo "  WARNING: vn_reset.py not found; the driver may segfault on a"
    echo "           stale stream. Expected next to this script or in \$HOME."
  fi
  source "$HOME/catkin_ws/devel/setup.bash"
  setsid nohup roslaunch base_model vectornav.launch \
    > "$LOG/live_vectornav.log" 2>&1 < /dev/null &
  for i in $(seq 1 20); do
    timeout 3 rostopic echo -n1 /vectornav/IMU/header >/dev/null 2>&1 && break
    sleep 1
  done
  if ! timeout 3 rostopic echo -n1 /vectornav/IMU/header >/dev/null 2>&1; then
    echo "ERROR: /vectornav/IMU silent (check /dev/vn cable)"; exit 6
  fi
  echo "  VN-100 OK"
  FASTLIO_LAUNCH="mapping_mid360_vn100.launch"
  BODY_FRAME_PROFILE="vn100"
else
  echo "  VN_IMU=0 - using the lidar's built-in IMU"
  for i in $(seq 1 20); do
    timeout 3 rostopic echo -n1 /livox/imu/header >/dev/null 2>&1 && break
    sleep 1
  done
  if ! timeout 3 rostopic echo -n1 /livox/imu/header >/dev/null 2>&1; then
    echo "ERROR: /livox/imu not publishing; FAST-LIO was not started"
    exit 8
  fi
  echo "  Livox IMU OK"
  FASTLIO_LAUNCH="mapping_mid360.launch"
  BODY_FRAME_PROFILE="builtin"
fi

echo "[3/5] FAST-LIO (keep the wheelchair STILL for a few seconds)"
source "$HOME/fast_lio_ws/devel/setup.bash"

start_fastlio() {
  # Kill the roslaunch wrapper as well as the node: left running it keeps the
  # laserMapping name registered, and the replacement would evict the old node
  # through a name conflict instead of starting cleanly.
  pkill -f '[r]oslaunch fast_lio' 2>/dev/null || true
  pkill -f '[f]astlio_mapping' 2>/dev/null || true
  for _ in $(seq 1 10); do
    pgrep -f '[f]astlio_mapping' >/dev/null 2>&1 || break
    sleep 1
  done
  sleep 2
  setsid nohup roslaunch fast_lio "$FASTLIO_LAUNCH" rviz:=false \
    > "$LOG/live_fastlio.log" 2>&1 < /dev/null &
  for _ in $(seq 1 20); do
    timeout 3 rostopic echo -n1 /Odometry/header >/dev/null 2>&1 && return 0
    sleep 2
  done
  return 1
}

# FAST-LIO fixes gravity and IMU bias from its first seconds on the assumption
# the chair is stationary. Wheeling it into position and starting immediately -
# or a rider still settling - tilts that estimate, and a tilted gravity vector
# integrates into velocity until the odometry runs away in its own frame. No
# initial pose can correct it, because the seed only sets map-to-odom and the
# odom underneath is what is wrong. Parked, the fault is measurable in seconds,
# so it is measured here rather than left to the operator to notice mid-drive.
FASTLIO_HEALTH_RETRIES="${FASTLIO_HEALTH_RETRIES:-1}"
FASTLIO_HEALTH_S="${FASTLIO_HEALTH_S:-8.0}"
attempt=0
while : ; do
  if ! start_fastlio; then
    echo "ERROR: /Odometry not publishing"; exit 3
  fi
  echo "  odometry OK - checking init health (do not move the chair)"
  source "$HOME/livox_static_localization_ws/devel/setup.bash"
  if rosrun static_livox_localization fastlio_init_health.py \
      _duration_s:="$FASTLIO_HEALTH_S" 2>&1 | sed 's/^/  health: /'; then
    echo "  FAST-LIO init OK"
    break
  fi
  attempt=$((attempt + 1))
  if [ "$attempt" -gt "$FASTLIO_HEALTH_RETRIES" ]; then
    echo "ERROR: FAST-LIO keeps initializing badly after $attempt attempts."
    echo "  Its odometry drifts while parked, so localization cannot hold."
    echo "  Keep the chair and rider completely still, then rerun."
    exit 10
  fi
  echo "  restarting FAST-LIO (attempt $((attempt + 1)))"
done
source "$HOME/fast_lio_ws/devel/setup.bash"

echo "[4/5] localization + rviz + auto init"
source "$HOME/livox_static_localization_ws/devel/setup.bash"
rosparam set /fast_lio_icp/auto_initialization_verified false
rosparam set /fast_lio_icp/auto_initialization_stable false
rosparam set /fast_lio_icp/auto_initialization_source none
setsid nohup roslaunch static_livox_localization moving_localization.launch \
  rviz:="$RVIZ" auto_init:=true auto_init_global_only:=true \
  map_path:="$MAP" map_sha256:="$MAP_SHA256" map_id:="$MAP_ID" \
  auto_init_map:="$MAP" auto_init_traj:="$TRAJ" \
  auto_init_route:="$ROUTE" \
  auto_init_body_frame_profile:="$BODY_FRAME_PROFILE" \
  auto_init_min_refined_score:="${MIN_REFINED_SCORE:-0.80}" \
  > "$LOG/live_localization.log" 2>&1 < /dev/null &

echo "[5/7] waiting for TRACKING (auto seed + consensus)"
LOCALIZED=0
AUTO_INIT_SEEN=0
AUTO_INIT_TIMEOUT_S="${AUTO_INIT_TIMEOUT_S:-180}"
case "$AUTO_INIT_TIMEOUT_S" in
  *[!0-9]*|"") echo "ERROR: AUTO_INIT_TIMEOUT_S must be an integer"; exit 9 ;;
esac
AUTO_INIT_DEADLINE=$(( $(date +%s) + AUTO_INIT_TIMEOUT_S ))
while [ "$(date +%s)" -lt "$AUTO_INIT_DEADLINE" ]; do
  STATE=$(
    timeout 3 rostopic echo -n1 \
      /fast_lio_icp/localization_diagnostics/status[0]/message \
      2>/dev/null | head -1 || true
  )
  AUTO_INITIALIZATION_VERIFIED=$(
    rosparam get /fast_lio_icp/auto_initialization_verified 2>/dev/null ||
      echo false
  )
  AUTO_INITIALIZATION_STABLE=$(
    rosparam get /fast_lio_icp/auto_initialization_stable 2>/dev/null ||
      echo false
  )
  AUTO_INITIALIZATION_SOURCE=$(
    rosparam get /fast_lio_icp/auto_initialization_source 2>/dev/null ||
      echo none
  )
  echo "  state: $STATE"
  if echo "$STATE" | grep -q TRACKING &&
     [ "$AUTO_INITIALIZATION_VERIFIED" = "true" ] &&
     [ "$AUTO_INITIALIZATION_STABLE" = "true" ] &&
     [ "$AUTO_INITIALIZATION_SOURCE" = "global_search" ]; then
    LOCALIZED=1
    break
  fi
  if rosnode ping -c1 /auto_initial_pose >/dev/null 2>&1; then
    AUTO_INIT_SEEN=1
  elif [ "$AUTO_INIT_SEEN" = "1" ]; then
    echo "  auto initializer exited without TRACKING"
    break
  fi
  sleep 2
done
if [ "$LOCALIZED" != "1" ]; then
  echo "WARNING: global no-prior localization did not become stable."
  echo "Inspect $LOG/live_localization.log; motion remains disabled."
  exit 4
fi
echo "LOCALIZED"

echo "[6/7] wheel base + safety gate + follower (paused)"
source "$HOME/catkin_ws/devel/setup.bash"
setsid nohup roslaunch base_model wheel.launch \
  > "$LOG/live_base.log" 2>&1 < /dev/null &
for i in $(seq 1 15); do
  timeout 3 rostopic echo -n1 /wheel_status >/dev/null 2>&1 && break
  sleep 2
done
if ! timeout 3 rostopic echo -n1 /wheel_status >/dev/null 2>&1; then
  echo "ERROR: wheel base not responding (/wheel_status silent)"; exit 5
fi
source "$HOME/livox_static_localization_ws/devel/setup.bash"
setsid nohup rosrun static_livox_localization safety_gate.py \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_policies:="$SAFETY_POLICIES" \
  > "$LOG/live_gate.log" 2>&1 < /dev/null &
setsid nohup rosrun static_livox_localization tip_guard.py \
  > "$LOG/live_tipguard.log" 2>&1 < /dev/null &
# The follower steers around what this node reports as parked and waits for
# what it reports as moving, so it starts BEFORE the follower and the follower
# refuses to drive without it (HOLD:CLUSTERS_STALE). A missing producer must
# not read as an empty road.
setsid nohup rosrun static_livox_localization obstacle_clusters.py \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_band:="$BAND" \
  > "$LOG/live_clusters.log" 2>&1 < /dev/null &
for i in $(seq 1 15); do
  timeout 2 rostopic echo -n1 /perception/objects_summary >/dev/null 2>&1 && break
  sleep 1
done
if ! timeout 3 rostopic echo -n1 /perception/objects_summary >/dev/null 2>&1; then
  echo "ERROR: object clustering silent (/perception/objects_summary)" >&2
  echo "       the follower will hold on CLUSTERS_STALE; see $LOG/live_clusters.log" >&2
  exit 6
fi
echo "  object tracking up - watch /perception/objects_summary"
for i in $(seq 1 10); do
  timeout 2 rostopic echo -n1 /tip_guard/status >/dev/null 2>&1 && break
  sleep 1
done
echo "  final-stage relay up - watch /tip_guard/status"
FOLLOWER_SCRIPT="waypoint_follower.py"
if [ "$PLANNER" = "priest" ]; then
  FOLLOWER_SCRIPT="priest_follower.py"
  echo "  PLANNER=priest - PRIEST corridor planner (opt-in, guards always on)"
fi
setsid nohup rosrun static_livox_localization "$FOLLOWER_SCRIPT" \
  _route:="$ROUTE" _safety_band:="$BAND" \
  _planner:="$PLANNER" \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_policies:="$SAFETY_POLICIES" \
  > "$LOG/live_follower.log" 2>&1 < /dev/null &

echo "[7/7] black-box recorder"
mkdir -p "$HOME/localization_trials"
setsid nohup rosbag record --lz4 \
  -O "$HOME/localization_trials/blackbox_$(date +%Y%m%d_%H%M%S)" \
  /fast_lio_icp/pose /fast_lio_icp/localization_diagnostics /vectornav/IMU \
  /cmd_vel_raw /cmd_vel_gated /cmd_vel /wheel_cmd /wheel_status /mode_cmd \
  /waypoint_follower/status /tip_guard/status /Odometry /livox/imu \
  /perception/objects_summary \
  > "$LOG/live_blackbox.log" 2>&1 < /dev/null &

echo ""
if [ "$SAFETY_POLICIES" = "false" ]; then
  echo "*********************************************************************"
  echo "SAFETY POLICIES ARE OFF. No band containment, no raw-scan obstacle"
  echo "stop, no localization hold, no geofence."
  echo "STILL ACTIVE: tracked-cluster avoidance - the chair steers around"
  echo "what has been seen standing still and waits for what is moving."
  echo "The joystick is the failsafe. Keep a hand on it for the whole run."
  echo "Suppressed guards are published as WOULD_HOLD: on"
  echo "/waypoint_follower/status and recorded in the black box."
  echo "*********************************************************************"
  echo ""
fi
echo "READY. To drive the route:"
echo "  1) rostopic pub -1 /mode_cmd std_msgs/Int16 65     # auto mode"
echo "  2) rosservice call /waypoint_follower/start \"data: true\""
echo "Pause:  rosservice call /waypoint_follower/start \"data: false\""
echo "E-stop: joystick to manual mode (or: rostopic pub -1 /mode_cmd std_msgs/Int16 77)"
exit 0
