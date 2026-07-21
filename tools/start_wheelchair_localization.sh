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

echo "[0/5] display + vnc"
XAUTHORITY="$HOME/.Xauthority" xrandr --output HDMI-1 --mode 1920x1080 2>/dev/null || true
if ! pgrep -x x11vnc >/dev/null; then
  setsid nohup x11vnc -display :0 -auth guess -passwd 0000 -forever -shared \
    -repeat -wait 15 -defer 15 -o "$HOME/x11vnc.log" -bg >/dev/null 2>&1 < /dev/null || true
fi
echo "  vnc on port 5900 (pw 0000), NUC IP: $(hostname -I | tr ' ' '\n' | grep -v '^192\.168\.1\.' | head -1)"

echo "[1/5] cleaning old processes"
for pattern in '[r]oslaunch' '[r]osbag record' '[f]astlio_mapping' '[a]uto_initial_pose'; do
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

echo "[3/5] FAST-LIO (keep the wheelchair STILL for a few seconds)"
source "$HOME/fast_lio_ws/devel/setup.bash"
setsid nohup roslaunch fast_lio mapping_mid360.launch rviz:=false \
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

echo "[5/5] waiting for TRACKING (auto seed + consensus)"
for i in $(seq 1 45); do
  STATE=$(timeout 3 rostopic echo -n1 /fast_lio_icp/localization_diagnostics/status[0]/message 2>/dev/null | head -1)
  echo "  state: $STATE"
  echo "$STATE" | grep -q TRACKING && { echo "LOCALIZED - ready to drive"; exit 0; }
  sleep 2
done
echo "WARNING: not TRACKING yet. Check RViz overlay; you can still seed manually (2D Pose Estimate)."
exit 4
