#!/usr/bin/env bash
# Bring the chair up for the 0727 localization trial and stop, armed but not
# driving.
#
# The question this run answers is narrow: does localization stay attached
# over the whole 383 m of the 0727 line? So the discretionary guards are off
# - a band refusal, a geofence trip or a raw-scan obstacle stop would all end
# the measurement, and from outside a stationary chair they look identical to
# a lost fix. The suppressed guards are still evaluated and published as
# WOULD_HOLD: on /waypoint_follower/status, which is where the thresholds get
# calibrated afterwards.
#
# What stays on is what watches for people: the cluster tracker. Objects it
# has seen standing still get driven around from 5 m out; anything moving, or
# not yet watched long enough to judge, is waited for. And the joystick, which
# is the failsafe - moving it drops the base out of auto mode and the follower
# holds on MANUAL_MODE within a control cycle.
#
# Nothing drives until go.sh.
set -eo pipefail

cd "$HOME"
export SAFETY_POLICIES=false
export ROUTE="$HOME/wheelchair_localization_src/routes/20260727_chair_centred_waypoints.json"
export BAND="$HOME/wheelchair_localization_src/routes/20260727_chair_centred_safety_band.json"

echo "=============================================================="
echo " 0727 LOCALIZATION TRIAL"
echo "   route      the 0727 drive itself, 1446 points, 0.2 m apart"
echo "   guards     discretionary ones OFF - measuring localization"
echo "   watching   cluster tracker (go round parked, wait for moving)"
echo "   failsafe   the joystick"
echo "=============================================================="
echo ""

./start_wheelchair_localization.sh

echo ""
echo "=============================================================="
echo " READY - not driving."
echo ""
echo "   start:  ~/go.sh"
echo "   stop:   ~/stop.sh   (or just move the joystick)"
echo ""
echo " Watch:  rostopic echo /waypoint_follower/status"
echo "         rostopic echo /fast_lio_icp/localization_diagnostics"
echo "=============================================================="
