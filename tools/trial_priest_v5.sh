#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
export SAFETY_POLICIES=true
export PLANNER=priest
export VN_IMU=0
export ROUTE="$HOME/wheelchair_localization_src/routes/20260803_route_v5_waypoints.json"
export BAND="$HOME/wheelchair_localization_src/routes/20260803_route_v5_safety_band.json"

"$SCRIPT_DIR/start_wheelchair_localization.sh"
"$SCRIPT_DIR/stop.sh"
"$SCRIPT_DIR/preflight_priest_v5.sh"

echo "READY AND PAUSED. Start only with ~/go_priest_v5.sh"
