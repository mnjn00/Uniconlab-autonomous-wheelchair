#!/usr/bin/env bash
# One-command field startup: driver -> FAST-LIO -> localization(+RViz) -> auto seed.
set -eo pipefail

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  cat <<'EOF'
Usage: start_wheelchair_localization.sh

Environment: SHADOW_QA=1 starts sensors, perception, and localization only.
PROFILE=pursuit|mpc|dwa selects the controller for a normal field startup.
No command-line arguments other than --help are accepted.
EOF
  exit 0
fi
[ "$#" -eq 0 ] || {
  echo "ERROR: unexpected arguments; use --help" >&2
  exit 64
}

# Keep the field workspace as the default, while allowing a reviewed branch to
# be built and replayed in isolation before it replaces the live package.
LOCALIZATION_WS="${LOCALIZATION_WS:-$HOME/livox_static_localization_ws}"
MIN_REFINED_SCORE="${MIN_REFINED_SCORE:-0.78}"
[ -f "$LOCALIZATION_WS/devel/setup.bash" ] || {
  echo "ERROR: localization workspace is not built: $LOCALIZATION_WS" >&2
  exit 66
}

# CUDA component wheels live below the user's site-packages rather than in a
# system linker directory.  Discover them instead of pinning a Python patch
# version or a CUDA component list; children inherit the result.  With
# AUTO_INIT_REQUIRE_GPU=true, auto_initial_pose still fails closed if these
# libraries or the driver cannot execute a real kernel.
CUDA_PY_ROOT="$HOME/.local/lib/python3.8/site-packages/nvidia"
if [ -d "$CUDA_PY_ROOT" ]; then
  CUDA_PY_LIBS=$(find "$CUDA_PY_ROOT" -mindepth 2 -maxdepth 2 \
    -type d -name lib | sort | paste -sd: -)
  if [ -n "$CUDA_PY_LIBS" ]; then
    export LD_LIBRARY_PATH="$CUDA_PY_LIBS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
fi

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
TRAJ_SHA256="${TRAJ_SHA256:-4a5972e176ff9aa036f538ca67e20c87f1d5a469865cb8d6b8079f7023dccbbe}"
ACTUAL_TRAJ_SHA256="$(sha256sum "$TRAJ" | awk '{print $1}')"
if [ "$ACTUAL_TRAJ_SHA256" != "$TRAJ_SHA256" ]; then
  echo "ERROR: trajectory SHA-256 mismatch" >&2
  exit 2
fi
ROUTE="${ROUTE:-$HOME/wheelchair_localization_src/routes/20260814_route_algorithm_waypoints.json}"
AUTO_INIT_ROUTE="${AUTO_INIT_ROUTE:-$ROUTE}"
# Global search only, by default. The known-start shortcut hands the route's
# first waypoint straight to the localizer and reaches TRACKING in about 16 s,
# where the global search regularly spends its whole 180 s budget and gives up
# into MANUAL_ALIGN. It stays opt-in because it is only correct when the chair
# really is parked at that waypoint -- and either way the seed is a hypothesis,
# since ICP consensus still has to accept it and rejects a wrong prior.
#   AUTO_INIT_GLOBAL_ONLY=false ./start_wheelchair_localization.sh
AUTO_INIT_GLOBAL_ONLY="${AUTO_INIT_GLOBAL_ONLY:-true}"
case "$AUTO_INIT_GLOBAL_ONLY" in
  true|false) ;;
  *) echo "ERROR: AUTO_INIT_GLOBAL_ONLY must be true or false, got '$AUTO_INIT_GLOBAL_ONLY'" >&2
     exit 67 ;;
esac
BAND="${BAND:-$HOME/wheelchair_localization_src/routes/20260814_route_algorithm_safety_band.json}"
DRIVABLE_MASK="${DRIVABLE_MASK:-$HOME/wheelchair_localization_src/routes/route_2d_map_algorithm.yaml}"
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
# PROFILE picks the control law. Both run behind the same guards, publish the
# same status topic and answer the same start service; they differ only in
# how a pose becomes a Twist.
#
# pursuit  the validated one. Two complete autonomous runs of the 0727 route
#          on 2026-07-31. This is the default and should stay the default.
# dwa      Trajectory rollout with the band as a hard reject. Simulated,
#          never driven. This is the one to reach for when the run is about
#          getting past something, not about following the line.
# mpc      Completes the route in simulation, at a jitter measured to be
#          harsher than the chair's own, without leaving the band. Has never
#          driven the chair. Those are different things: it is here to be
#          measured on the NUC, watched, and promoted only on evidence from
#          the ground rather than from a plant model.
#
# Same two-literal rule as above, for the same reason: a typo must not pick a
# control law. Unlike SAFETY_POLICIES the unset default is the SAFE one, so
# an operator who has never heard of this variable gets the validated law.
# Which registration the localizer runs. Lived only on the NUC's deploy
# branch until now, which meant the machine and the repository disagreed
# about what was deployed - the divergence is the hazard, not the value.
# fast_vgicp_cuda is what the NUC has been running since 2026-08-06.
REGISTRATION_BACKEND="${REGISTRATION_BACKEND:-fast_vgicp_cuda}"
case "$REGISTRATION_BACKEND" in
  pcl_gicp|fast_vgicp_cuda) ;;
  *) echo "ERROR: REGISTRATION_BACKEND must be pcl_gicp or fast_vgicp_cuda, got '$REGISTRATION_BACKEND'" >&2
     exit 65 ;;
esac
# Corrections are suppressed until the chair has moved 0.10 m or turned 2
# deg, so a parked chair never runs registration - which also means the
# backend cannot be measured until the chair is already driving.
# STATIONARY_CORRECTION=on drops both thresholds to zero so a bench run can
# read fitness, inlier ratio and elapsed time standing still.
#
# Off by default, and it should stay off for a drive. Those thresholds are
# not an optimisation: parked at the goal after the 2026-07-31 runs the fix
# degraded to inlier 0.124-0.262 and crossed its gate four times with the
# chair motionless. Correcting continuously against that is how a good fix
# is talked out of itself.
STATIONARY_CORRECTION="${STATIONARY_CORRECTION:-off}"
case "$STATIONARY_CORRECTION" in
  on)  MIN_CORRECTION_TRANSLATION_M=0.0;  MIN_CORRECTION_YAW_DEG=0.0 ;;
  off) MIN_CORRECTION_TRANSLATION_M=0.10; MIN_CORRECTION_YAW_DEG=2.0 ;;
  *)   echo "ERROR: STATIONARY_CORRECTION must be on or off, got '$STATIONARY_CORRECTION'" >&2
       exit 65 ;;
esac

PROFILE="${PROFILE:-pursuit}"
SHADOW_QA="${SHADOW_QA:-0}"
case "$SHADOW_QA" in
  0|1) ;;
  *) echo "ERROR: SHADOW_QA must be 0 or 1" >&2; exit 64 ;;
esac
case "$PROFILE" in
  pursuit) FOLLOWER_NODE=waypoint_follower.py ;;
  mpc)     FOLLOWER_NODE=mpc_follower.py ;;
  # dwa     rolls candidate velocities out and rejects the ones that leave
  #         the safety band. The only profile that avoids an obstacle by
  #         choosing a velocity the chair can hold, rather than by pushing
  #         the pursuit target 0.6 m sideways - which from a standstill is a
  #         34 degree demand and put the chair at a wall three times on
  #         2026-08-04. Simulated, never driven.
  dwa)     FOLLOWER_NODE=dwa_follower.py ;;
  *) echo "ERROR: PROFILE must be pursuit, mpc or dwa, got '$PROFILE'" >&2
     exit 65 ;;
esac
# Actuation delay, in seconds, for the MPC profile to plan from where the
# chair WILL be rather than where it was. Zero until measured on this NUC -
# a guessed lead biases every command on the route in one direction, which
# is worse than no compensation at all. The runbook carries the procedure.
# Rejected rather than rounded if it is not a plain non-negative number: a
# typo here becomes a steering phase shift nobody typed.
LATENCY_S="${LATENCY_S:-0}"
case "$LATENCY_S" in
  *[!0-9.]*|*.*.*|'') LATENCY_BAD=1 ;;   # stray characters, or two dots
  *[0-9]*)            LATENCY_BAD=  ;;   # ...and it has to contain a digit
  *)                  LATENCY_BAD=1 ;;   # a lone "." reaches here
esac
if [ -n "$LATENCY_BAD" ]; then
  echo "ERROR: LATENCY_S must be a non-negative number, got '$LATENCY_S'" >&2
  exit 65
fi
# OpenBLAS sizes its thread pool to the core count and then SPIN-WAITS
# those threads between calls. Every numpy operation in the control-loop
# nodes is tiny - a 25-step horizon, a few hundred band stations - so the
# pool never does useful parallel work and the spinning is pure burn.
# Measured on this NUC on 2026-08-06 with the MPC profile armed and IDLE:
#
#   mpc_follower.py    28 threads   266 % CPU
#   safety_gate.py     17 threads   267 %
#   obstacle_clusters  17 threads   126 %
#   load average 16.77 on 8 threads
#
# ...which is why the control loop ran at a median 5.2 Hz against a nominal
# 10 during the 2026-08-05 drive. At these matrix sizes one thread is also
# FASTER than eight; the threading overhead dominates the arithmetic.
#
# Deliberately NOT applied to auto_initial_pose: its coarse search is the
# one place here that genuinely parallelises, over 16k hypotheses, and it
# runs once at startup rather than every cycle.
SINGLE_THREAD_ENV="OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
NUMEXPR_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1"

LOG=$HOME

source /opt/ros/noetic/setup.bash
if [ "$SHADOW_QA" = "1" ]; then
  : "${ROS_MASTER_URI:?shadow QA requires its isolated ROS master}"
  DETACH=""
else
  export ROS_MASTER_URI=http://127.0.0.1:11311
  DETACH="setsid nohup"
fi
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
# Every node this script starts has to appear here. A node left out does not
# fail loudly - it keeps running, keeps its ROS name, and keeps publishing,
# and the next run comes up on top of it. On 2026-08-06 an mpc_follower and
# an obstacle_clusters from a run 14 minutes earlier were still alive at
# 447% and 177% CPU while a new bringup sat in WAITING_INITIALIZATION:
# system CPU 68.1%, idle 3.7%. Killing those two orphans took idle to 85.3%.
# The stale follower also still held /cmd_vel_raw as its publisher, which is
# the part that matters most. The single-thread limits above bound what one
# node costs; nothing but this list bounds how many of them there are.
# test_every_detached_node_is_also_cleaned_up keeps it honest - and did:
# dwa_follower was added by the DWA profile in the same window this sweep was
# written in, on a branch that did not have it, so the two merged clean and
# the derived list caught what neither side could see alone.
if [ "$SHADOW_QA" != "1" ]; then
for pattern in '[r]oslaunch' '[r]osbag record' '[f]astlio_mapping' '[a]uto_initial_pose' '[s]afety_gate' '[t]ip_guard' '[w]aypoint_follower' '[m]pc_follower' '[d]wa_follower' '[o]bstacle_clusters' '[r]oute_identity_publisher'; do
  pkill -f "$pattern" 2>/dev/null || true
done
sleep 2
fi
if ! pgrep -f '[r]osmaster' >/dev/null; then
  $DETACH roscore > "$LOG/live_roscore.log" 2>&1 < /dev/null &
  sleep 4
fi
rosparam set /use_sim_time false

echo "[2/5] livox driver"
source "$HOME/ws_livox/devel/setup.bash"
$DETACH roslaunch livox_ros_driver2 msg_MID360.launch \
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
    $DETACH roslaunch base_model vectornav.launch \
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
  if [ "$SHADOW_QA" != "1" ]; then
    pkill -f '[r]oslaunch fast_lio' 2>/dev/null || true
    pkill -f '[f]astlio_mapping' 2>/dev/null || true
    for _ in $(seq 1 10); do
      pgrep -f '[f]astlio_mapping' >/dev/null 2>&1 || break
      sleep 1
    done
    sleep 2
  fi
  $DETACH roslaunch fast_lio "$FASTLIO_LAUNCH" rviz:=false \
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
  source "$LOCALIZATION_WS/devel/setup.bash"
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
source "$LOCALIZATION_WS/devel/setup.bash"
rosparam set /fast_lio_icp/auto_initialization_verified false
rosparam set /fast_lio_icp/auto_initialization_stable false
rosparam set /fast_lio_icp/auto_initialization_source none
$DETACH roslaunch static_livox_localization moving_localization.launch \
  rviz:="$RVIZ" auto_init:=true auto_init_global_only:="$AUTO_INIT_GLOBAL_ONLY" \
  map_path:="$MAP" map_sha256:="$MAP_SHA256" map_id:="$MAP_ID" \
  auto_init_map:="$MAP" auto_init_traj:="$TRAJ" \
  auto_init_route:="$AUTO_INIT_ROUTE" \
  auto_init_body_frame_profile:="$BODY_FRAME_PROFILE" \
  auto_init_min_refined_score:="${MIN_REFINED_SCORE:-0.78}" \
  auto_init_require_gpu:="${AUTO_INIT_REQUIRE_GPU:-true}" \
  auto_init_gpu_lateral_radius_m:="${AUTO_INIT_GPU_LATERAL_RADIUS_M:-10.0}" \
  auto_init_gpu_lateral_step_m:="${AUTO_INIT_GPU_LATERAL_STEP_M:-1.0}" \
  registration_backend:="$REGISTRATION_BACKEND" \
  min_tracking_correction_translation_m:="$MIN_CORRECTION_TRANSLATION_M" \
  min_tracking_correction_yaw_deg:="$MIN_CORRECTION_YAW_DEG" \
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

start_object_tracking() {
  source "$LOCALIZATION_WS/devel/setup.bash"
  $DETACH env $SINGLE_THREAD_ENV \
    rosrun static_livox_localization obstacle_clusters.py \
    _body_frame_profile:="$BODY_FRAME_PROFILE" \
    _safety_band:="$BAND" \
    _map_path:="$MAP" \
    _map_sha256:="$MAP_SHA256" \
    _shadow_qa:="$SHADOW_QA" \
    > "$LOG/live_clusters.log" 2>&1 < /dev/null &
  for i in $(seq 1 15); do
    timeout 2 rostopic echo -n1 /perception/objects_summary \
      >/dev/null 2>&1 && break
    sleep 1
  done
  if ! timeout 3 rostopic echo -n1 /perception/objects_summary \
      >/dev/null 2>&1; then
    echo "ERROR: object clustering silent (/perception/objects_summary)" >&2
    echo "       see $LOG/live_clusters.log" >&2
    return 1
  fi
  echo "  object tracking up - watch /perception/objects_summary"
}

if [ "$SHADOW_QA" = "1" ]; then
  start_object_tracking
  if pgrep -af 'wheel_cmd|waypoint_follower|dwa_follower|mpc_follower|safety_gate' \
      >/dev/null; then
    echo "ERROR: motion process present during SHADOW_QA" >&2
    exit 20
  fi
  echo "SHADOW_QA_READY: sensor + perception + localization only"
  exit 0
fi

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
source "$LOCALIZATION_WS/devel/setup.bash"
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization safety_gate.py \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_policies:="$SAFETY_POLICIES" \
  > "$LOG/live_gate.log" 2>&1 < /dev/null &
setsid nohup rosrun static_livox_localization tip_guard.py \
  > "$LOG/live_tipguard.log" 2>&1 < /dev/null &
# The follower steers around what this node reports as parked and waits for
# what it reports as moving, so it starts BEFORE the follower and the follower
# refuses to drive without it (HOLD:CLUSTERS_STALE). A missing producer must
# not read as an empty road.
start_object_tracking
for i in $(seq 1 10); do
  timeout 2 rostopic echo -n1 /tip_guard/status >/dev/null 2>&1 && break
  sleep 1
done
echo "  final-stage relay up - watch /tip_guard/status"
if [ "$PROFILE" != "pursuit" ]; then
  echo "  PROFILE=$PROFILE - simulation-only control law, never driven on the chair"
  echo "  watch /waypoint_follower/status; see docs/runbooks/mpc-profile-ko.md"
fi
setsid nohup env $SINGLE_THREAD_ENV \
  rosrun static_livox_localization "$FOLLOWER_NODE" \
  _route:="$ROUTE" _safety_band:="$BAND" \
  _drivable_mask:="$DRIVABLE_MASK" \
  _body_frame_profile:="$BODY_FRAME_PROFILE" \
  _safety_policies:="$SAFETY_POLICIES" \
  _latency_s:="$LATENCY_S" \
  > "$LOG/live_follower.log" 2>&1 < /dev/null &

echo "[7/7] black-box recorder"
mkdir -p "$HOME/localization_trials"
setsid nohup rosrun static_livox_localization route_identity_publisher.py \
  _route:="$ROUTE" _safety_band:="$BAND" \
  _drivable_mask:="$DRIVABLE_MASK" \
  > "$LOG/live_route_identity.log" 2>&1 < /dev/null &
setsid nohup rosbag record --lz4 \
  -O "$HOME/localization_trials/blackbox_$(date +%Y%m%d_%H%M%S)" \
  /fast_lio_icp/pose /fast_lio_icp/localization_diagnostics /vectornav/IMU \
  /cmd_vel_raw /cmd_vel_gated /cmd_vel /wheel_cmd /wheel_status /mode_cmd \
  /waypoint_follower/status /tip_guard/status /Odometry /livox/imu \
  /perception/objects_summary /perception/dynamic_boxes /perception/objects \
  /waypoint_follower/route_identity \
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
