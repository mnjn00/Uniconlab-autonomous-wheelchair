#!/usr/bin/env bash
# Start the drive, but only if the MPC control law is the one that will do
# the driving.
#
# go.sh is control-law agnostic on purpose: it checks the things that must
# hold whichever law is running. That leaves one thing unchecked, and it is
# the thing an MPC run exists to observe - MPC is asked for by an
# environment variable at bringup, hours earlier, and every downstream
# signal looks identical either way. Same node name, same topics, same
# service, and a status line that reads HOLD:PAUSED for both right up until
# the chair moves. An operator who forgot PROFILE=mpc finds out by watching
# a pursuit run and recording it as an MPC measurement.
#
# The check reads the identity the follower published about itself. That
# distinction is the whole point: ~/preflight_priest_v5.sh compared a shell
# variable it had exported one line earlier against itself, and went on
# reporting "priest" long after 81fed5d reverted the PRIEST planner and the
# bringup stopped forwarding it.
#
# Nothing safety-bearing is duplicated here. Once the law is confirmed this
# hands straight to go.sh, which owns the follower/perception/localization
# checks and the start itself.
set -eo pipefail

source /opt/ros/noetic/setup.bash >/dev/null 2>&1
export ROS_MASTER_URI=http://127.0.0.1:11311

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"

fail() { echo "REFUSING TO START: $1" >&2; exit 1; }

LAW="$(rosparam get /waypoint_follower/control_law 2>/dev/null)" || fail \
  "the follower is not publishing a control law - it is either not running,
   or it predates ~control_law and cannot be identified. Bring the stack up
   with: PROFILE=mpc ~/start_wheelchair_localization.sh"

[ "$LAW" = "mpc" ] || fail \
  "the running control law is '$LAW', not mpc. Nothing here can switch it -
   the profile is chosen when the follower starts. Stop the stack and bring
   it up with: PROFILE=mpc ~/start_wheelchair_localization.sh"

echo "control law: mpc (confirmed by the running follower)"

# Unmeasured on this NUC, so it is reported rather than enforced: the
# runbook's promotion gate is solve-time p99 <= 25 ms with FAST-LIO running,
# and a run that never sees REUSED. Watch /waypoint_follower/status.
LATENCY="$(rosparam get /waypoint_follower/latency_s 2>/dev/null || echo "?")"
echo "latency compensation: ${LATENCY} s"
if [ "$LATENCY" = "0.0" ] || [ "$LATENCY" = "0" ]; then
  echo "  (unmeasured - see docs/runbooks/mpc-profile-ko.md section 5)"
fi

echo ""
echo "MPC has never driven this chair. Watch for steering hunting and for"
echo "REUSED on /waypoint_follower/status; abort criteria are in section 4"
echo "of docs/runbooks/mpc-profile-ko.md. Keep a hand on the joystick."
echo ""

exec "$SCRIPT_DIR/go.sh"
