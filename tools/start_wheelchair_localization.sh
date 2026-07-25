#!/usr/bin/env bash
# One-command field startup: driver -> FAST-LIO -> localization(+RViz) -> auto seed.
set -eo pipefail

MAP="${MAP:-$HOME/wheelchair_localization_maps/livox_raw_20260707/livox_raw_20260707_0p20m_xyzi.pcd}"
TRAJ="${TRAJ:-$HOME/wheelchair_localization_maps/livox_raw_20260707/traj_lidar.txt}"
RVIZ="${RVIZ:-true}"
LOG=$HOME

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export DISPLAY="${DISPLAY:-:0}"

echo "[0/5] display"
XAUTHORITY="$HOME/.Xauthority" xrandr --output HDMI-1 --mode 1920x1080 2>/dev/null || true

# VNC is OFF by default and bound to loopback when enabled.
#
# This used to start x11vnc unconditionally with `-passwd 0000` on the
# command line, no -localhost, and then print the reachable NUC IP. The
# password is in this repository's public history, so it is not a secret and
# cannot be made one by editing it here - anyone on the network could take
# the desktop session, and from that session drive the chair via rosservice.
# Treat 0000 as burned: generate a new password into the -rfbauth file below.
#
#   VNC=1                       enable, listening on 127.0.0.1 only
#   VNC_ALLOW_REMOTE=1          also bind the network interface (avoid)
#   VNC_AUTH=<path>             password file, default ~/.vnc/passwd
#
# Create the password file once, readable only by the operator account:
#   x11vnc -storepasswd ~/.vnc/passwd
# Then reach the desktop over an SSH tunnel rather than exposing the port:
#   ssh -L 5900:127.0.0.1:5900 mprp3@<nuc>
VNC="${VNC:-0}"
VNC_AUTH="${VNC_AUTH:-$HOME/.vnc/passwd}"
if [ "$VNC" = "1" ]; then
  if [ ! -f "$VNC_AUTH" ]; then
    echo "  VNC requested but $VNC_AUTH is missing."
    echo "  Create it with: x11vnc -storepasswd $VNC_AUTH"
    echo "  (refusing to start an unauthenticated or hardcoded-password VNC)"
    exit 7
  fi
  if [ "$(stat -c %a "$VNC_AUTH" 2>/dev/null)" != "600" ]; then
    echo "  WARNING: $VNC_AUTH is not mode 600 - tighten it (chmod 600)"
  fi
  VNC_BIND="-localhost"
  VNC_WHERE="127.0.0.1 only (tunnel with: ssh -L 5900:127.0.0.1:5900 ...)"
  if [ "$VNC_ALLOW_REMOTE" = "1" ]; then
    VNC_BIND=""
    VNC_WHERE="ALL INTERFACES - exposed to the network"
    echo "  WARNING: VNC_ALLOW_REMOTE=1, the desktop is reachable from the network"
  fi
  if ! pgrep -x x11vnc >/dev/null; then
    # shellcheck disable=SC2086
    setsid nohup x11vnc -display :0 -auth guess -rfbauth "$VNC_AUTH" \
      $VNC_BIND -forever -shared -repeat -wait 15 -defer 15 \
      -o "$HOME/x11vnc.log" -bg >/dev/null 2>&1 < /dev/null || true
  fi
  echo "  vnc on 5900, $VNC_WHERE"
else
  echo "  vnc disabled (VNC=1 to enable; see the notes in this script)"
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

# VN-100: the inertial source for both FAST-LIO and tip_guard. It must be
# up BEFORE FAST-LIO, which waits on IMU messages to initialise. To fall
# back to the lidar's built-in IMU, set VN_IMU=0 and the FAST-LIO launch
# below reverts to the previously validated mapping_mid360.launch.
VN_IMU="${VN_IMU:-1}"
if [ "$VN_IMU" = "1" ]; then
  echo "[2b/5] VectorNav VN-100"
  # A SIGTERM'd vnpub leaves the sensor streaming binary at 921600; the next
  # driver start parses that backlog as register replies and segfaults
  # (observed: exit code -11). Silence async output before opening it.
  # Resolve next to this script first so a repo checkout works, then fall
  # back to the deployed layout where only the script itself is copied to
  # $HOME. Hardcoding $HOME meant the repo copy could never run it.
  VN_RESET=""
  for candidate in "$(dirname "$0")/vn_reset.py" "$HOME/vn_reset.py"; do
    [ -f "$candidate" ] && { VN_RESET="$candidate"; break; }
  done
  if [ -n "$VN_RESET" ]; then
    python3 "$VN_RESET" 2>&1 | sed 's/^/  vn_reset: /' || \
      echo "  vn_reset failed - continuing, driver may still negotiate"
  else
    echo "  vn_reset.py not found - the driver may segfault on a stale stream"
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
else
  echo "  VN_IMU=0 - falling back to the lidar's built-in IMU"
  FASTLIO_LAUNCH="mapping_mid360.launch"
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
  > "$LOG/live_gate.log" 2>&1 < /dev/null &
if [ "$VN_IMU" = "1" ]; then
  IMU_TOPIC="${IMU_TOPIC:-/vectornav/IMU}"
else
  IMU_TOPIC="${IMU_TOPIC:-/livox/imu}"
fi
setsid nohup rosrun static_livox_localization tip_guard.py \
  _imu_topic:="$IMU_TOPIC" \
  > "$LOG/live_tipguard.log" 2>&1 < /dev/null &
for i in $(seq 1 10); do
  timeout 2 rostopic echo -n1 /tip_guard/status >/dev/null 2>&1 && break
  sleep 1
done
echo "  tip_guard armed - watch /tip_guard/status; if it stays"
echo "  CONFIG_UNVERIFIED for more than ~30s of driving, the IMU axis"
echo "  needs checking (see IMU_TOPIC / rosparam ~gyro_pitch_axis/sign)"
ROUTE="${ROUTE:-$HOME/wheelchair_localization_src/routes/aejimun_to_gongsen_waypoints.json}"
BAND="${BAND:-$HOME/wheelchair_localization_src/routes/aejimun_to_gongsen_safety_band.json}"
setsid nohup rosrun static_livox_localization waypoint_follower.py \
  _route:="$ROUTE" _safety_band:="$BAND" \
  > "$LOG/live_follower.log" 2>&1 < /dev/null &

echo "[7/7] black-box recorder"
mkdir -p "$HOME/localization_trials"
setsid nohup rosbag record --lz4 \
  -O "$HOME/localization_trials/blackbox_$(date +%Y%m%d_%H%M%S)" \
  /fast_lio_icp/pose /fast_lio_icp/localization_diagnostics \
  /cmd_vel_raw /cmd_vel_gated /cmd_vel /wheel_cmd /wheel_status /mode_cmd \
  /waypoint_follower/status /tip_guard/status /Odometry \
  /livox/imu /vectornav/IMU \
  > "$LOG/live_blackbox.log" 2>&1 < /dev/null &

echo ""
echo "READY. To drive the route:"
echo "  1) rostopic pub -1 /mode_cmd std_msgs/Int16 65     # auto mode"
echo "  2) rosservice call /waypoint_follower/start \"data: true\""
echo "Pause:  rosservice call /waypoint_follower/start \"data: false\""
echo "E-stop: joystick to manual mode (or: rostopic pub -1 /mode_cmd std_msgs/Int16 77)"
exit 0
