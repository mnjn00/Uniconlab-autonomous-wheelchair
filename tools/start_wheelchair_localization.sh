#!/usr/bin/env bash
# One-command field startup: driver -> FAST-LIO -> localization(+RViz) -> auto seed.
set -eo pipefail

# Map merged from three passes (07/07 + 07/25), pose-graph optimised with
# 1460 submaps over 27519 frames. Deploy with tools/deploy_merged_map.sh,
# which converts the .ply GLIM emits into the .pcd the localizer reads.
# The previous single-pass map is still selectable:
#   MAP=$HOME/wheelchair_localization_maps/livox_raw_20260707/livox_raw_20260707_0p20m_xyzi.pcd \
#   TRAJ=$HOME/wheelchair_localization_maps/livox_raw_20260707/traj_lidar.txt ./start_wheelchair_localization.sh
MAP="${MAP:-$HOME/wheelchair_localization_maps/merged_0707_0725_v1/merged_0707_0725.pcd}"
TRAJ="${TRAJ:-$HOME/wheelchair_localization_maps/merged_0707_0725_v1/traj_lidar.txt}"
RVIZ="${RVIZ:-true}"
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

# The VN-100 is the inertial half of the localization solution and must
# be publishing BEFORE FAST-LIO, which blocks waiting for IMU messages to
# initialise. VN_IMU=0 reverts to the lidar's built-in IMU: it selects the
# previously validated mapping_mid360.launch and the matching body-frame
# profile, so the swap can be backed out on its own.
VN_IMU="${VN_IMU:-1}"
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
  echo "  VN_IMU=0 - falling back to the lidar's built-in IMU"
  FASTLIO_LAUNCH="mapping_mid360.launch"
  BODY_FRAME_PROFILE="builtin"
fi

echo "[3/5] FAST-LIO (keep the wheelchair STILL for a few seconds)"
source "$HOME/fast_lio_ws/devel/setup.bash"
setsid nohup roslaunch fast_lio "$FASTLIO_LAUNCH" rviz:=false \
  > "$LOG/live_fastlio.log" 2>&1 < /dev/null &
for i in $(seq 1 20); do
  timeout 3 rostopic echo -n1 /Odometry/header >/dev/null 2>&1 && break
  sleep 2
done
if ! timeout 3 rostopic echo -n1 /Odometry/header >/dev/null 2>&1; then
  echo "ERROR: /Odometry not publishing"; exit 3
fi
echo "  odometry OK"

echo "[4/5] localization + rviz + auto init"
source "$HOME/livox_static_localization_ws/devel/setup.bash"
setsid nohup roslaunch static_livox_localization moving_localization.launch \
  rviz:="$RVIZ" auto_init:=true auto_init_map:="$MAP" auto_init_traj:="$TRAJ" \
  > "$LOG/live_localization.log" 2>&1 < /dev/null &

echo "[5/7] waiting for TRACKING (auto seed + consensus)"
LOCALIZED=0
for i in $(seq 1 45); do
  STATE=$(timeout 3 rostopic echo -n1 /fast_lio_icp/localization_diagnostics/status[0]/message 2>/dev/null | head -1)
  echo "  state: $STATE"
  echo "$STATE" | grep -q TRACKING && { LOCALIZED=1; break; }
  sleep 2
done
if [ "$LOCALIZED" != "1" ]; then
  echo "WARNING: not TRACKING yet. Seed manually in RViz, then re-run or continue by hand."
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
  > "$LOG/live_gate.log" 2>&1 < /dev/null &
setsid nohup rosrun static_livox_localization tip_guard.py \
  > "$LOG/live_tipguard.log" 2>&1 < /dev/null &
for i in $(seq 1 10); do
  timeout 2 rostopic echo -n1 /tip_guard/status >/dev/null 2>&1 && break
  sleep 1
done
echo "  final-stage relay up - watch /tip_guard/status"
ROUTE="${ROUTE:-$HOME/wheelchair_localization_src/routes/20260727_new_route_waypoints.json}"
BAND="${BAND:-$HOME/wheelchair_localization_src/routes/20260727_new_route_safety_band.json}"
setsid nohup rosrun static_livox_localization waypoint_follower.py \
  _route:="$ROUTE" _safety_band:="$BAND" \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  > "$LOG/live_follower.log" 2>&1 < /dev/null &

echo "[7/7] black-box recorder"
mkdir -p "$HOME/localization_trials"
setsid nohup rosbag record --lz4 \
  -O "$HOME/localization_trials/blackbox_$(date +%Y%m%d_%H%M%S)" \
  /fast_lio_icp/pose /fast_lio_icp/localization_diagnostics /vectornav/IMU \
  /cmd_vel_raw /cmd_vel_gated /cmd_vel /wheel_cmd /wheel_status /mode_cmd \
  /waypoint_follower/status /tip_guard/status /Odometry /livox/imu \
  > "$LOG/live_blackbox.log" 2>&1 < /dev/null &

echo ""
echo "READY. To drive the route:"
echo "  1) rostopic pub -1 /mode_cmd std_msgs/Int16 65     # auto mode"
echo "  2) rosservice call /waypoint_follower/start \"data: true\""
echo "Pause:  rosservice call /waypoint_follower/start \"data: false\""
echo "E-stop: joystick to manual mode (or: rostopic pub -1 /mode_cmd std_msgs/Int16 77)"
exit 0
