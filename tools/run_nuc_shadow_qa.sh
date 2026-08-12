#!/usr/bin/env bash
# Sensor/perception/localization-only QA. Never starts wheel or follower nodes.
set -euo pipefail

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  cat <<'EOF'
Usage: run_nuc_shadow_qa.sh

Builds and runs sensor/perception/localization-only QA. It refuses any
connected ROS motion-command surface and never launches a wheel or follower.
Configure REPO, WS, MAP, MAP_SHA256, BAND, OUT, and TIMEOUT_S by environment.
EOF
  exit 0
fi
[ "$#" -eq 0 ] || {
  echo "ERROR: unexpected arguments; use --help" >&2
  exit 64
}

REPO="${REPO:-$HOME/wheelchair_localization_src}"
WS="${WS:-$HOME/livox_static_localization_ws}"
MAP="${MAP:-$HOME/wheelchair_localization_maps/merged_0707_0725_v1/merged_0707_0725_0p20m_xyzi.pcd}"
MAP_SHA256="${MAP_SHA256:-ee317581328d3eaeee86ba448b0068c1016ca1452664b6cdaba2d874320d0431}"
BAND="${BAND:-$REPO/routes/20260812_route_v6_v8_safety_band.json}"
OUT="${OUT:-/tmp/ulw-evidence}"
TIMEOUT_S="${TIMEOUT_S:-45}"
mkdir -p "$OUT"

AUTONOMOUS_RE='wheel_cmd|waypoint_follower|dwa_follower|mpc_follower|safety_gate'
LAUNCHED_PIDS=()
STARTED_STACK=0

cleanup() {
  local pid
  for pid in "${LAUNCHED_PIDS[@]}"; do
    kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
  done
  pkill -f 'obstacle_clusters.py.*_shadow_qa:=true' 2>/dev/null || true
  if [ "$STARTED_STACK" = "1" ]; then
    for pattern in \
        '[r]oslaunch livox_ros_driver2' '[f]astlio_mapping' \
        '[m]oving_icp_localizer' '[m]ap_preview_publisher' \
        '[o]bstacle_clusters' '[a]uto_initial_pose'; do
      pkill -f "$pattern" 2>/dev/null || true
    done
  fi
  cleanup_failed=0
  for _ in $(seq 1 30); do
    if ! pgrep -af 'obstacle_clusters.py.*_shadow_qa:=true' >/dev/null &&
       ! pgrep -af "$AUTONOMOUS_RE" >/dev/null; then
      break
    fi
    sleep 0.2
  done
  if pgrep -af 'obstacle_clusters.py.*_shadow_qa:=true' >/dev/null ||
      pgrep -af "$AUTONOMOUS_RE" >/dev/null; then
    cleanup_failed=1
  fi
  {
    echo "cleanup_at=$(date -Iseconds)"
    echo "cleanup_status=$([ "$cleanup_failed" = "0" ] && echo PASS || echo FAIL)"
    echo "remaining_autonomous_processes:"
    pgrep -af "$AUTONOMOUS_RE" || true
    echo "remaining_shadow_processes:"
    pgrep -af 'obstacle_clusters.py.*_shadow_qa:=true' || true
  } > "$OUT/cleanup-receipt.txt"
  return "$cleanup_failed"
}
finish() {
  local status=$?
  trap - EXIT INT TERM
  cleanup || status=90
  exit "$status"
}
trap finish EXIT INT TERM

if pgrep -af "$AUTONOMOUS_RE" > "$OUT/autonomous-before.txt"; then
  echo "ERROR: autonomous process present; refusing shadow QA" >&2
  cat "$OUT/autonomous-before.txt" >&2
  exit 20
fi
cd "$WS"
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u

catkin_make > "$OUT/catkin-build.txt" 2>&1
catkin_make run_tests_static_livox_localization \
  > "$OUT/catkin-tests.txt" 2>&1
catkin_test_results build/static_livox_localization \
  > "$OUT/catkin-test-results.txt" 2>&1

if ! rostopic list >/dev/null 2>&1; then
  setsid roscore > "$OUT/roscore.log" 2>&1 &
  LAUNCHED_PIDS+=("$!")
  for _ in $(seq 1 30); do
    rostopic list >/dev/null 2>&1 && break
    sleep 0.2
  done
fi
python3 "$REPO/tools/check_shadow_ros_graph.py" \
  > "$OUT/ros-graph-before.json" || {
  echo "ERROR: ROS motion surface present; refusing shadow QA" >&2
  cat "$OUT/ros-graph-before.json" >&2
  exit 25
}

if ! rostopic list | grep -qx '/cloud_registered_body' ||
    ! rostopic list | grep -qx '/fast_lio_icp/pose'; then
  STARTED_STACK=1
  SHADOW_QA=1 "$REPO/tools/start_wheelchair_localization.sh" \
    > "$OUT/shadow-stack.log" 2>&1
fi

if ! rostopic list | grep -qx '/cloud_registered_body'; then
  echo "ERROR: /cloud_registered_body is absent after shadow startup" >&2
  exit 21
fi
if ! rostopic list | grep -qx '/fast_lio_icp/pose'; then
  echo "ERROR: /fast_lio_icp/pose is absent after shadow startup" >&2
  exit 22
fi

if ! rostopic list | grep -qx '/perception/objects_summary'; then
  setsid rosrun static_livox_localization obstacle_clusters.py \
    _body_frame_profile:=vn100 \
    _safety_band:="$BAND" \
    _map_path:="$MAP" \
    _map_sha256:="$MAP_SHA256" \
    _shadow_qa:=true \
    > "$OUT/shadow-clusters.log" 2>&1 &
  LAUNCHED_PIDS+=("$!")
fi

pgrep -af "$AUTONOMOUS_RE" > "$OUT/autonomous-after.txt" && {
  echo "ERROR: autonomous process appeared during shadow QA" >&2
  exit 23
}
python3 "$REPO/tools/check_shadow_ros_graph.py" \
  > "$OUT/ros-graph-after.json" || {
  echo "ERROR: ROS motion surface appeared during shadow QA" >&2
  exit 26
}

for attempt in $(seq 1 10); do
  timeout "$TIMEOUT_S" rostopic echo -n 1 /perception/objects_summary \
    > "$OUT/objects-summary.yaml"
  timeout "$TIMEOUT_S" rostopic echo -n 1 /perception/dynamic_boxes \
    > "$OUT/dynamic-boxes.yaml"
  timeout "$TIMEOUT_S" rostopic echo -n 1 \
    /fast_lio_icp/localization_diagnostics \
    > "$OUT/localization-diagnostics.yaml"
  if python3 "$REPO/tools/validate_nuc_shadow_snapshot.py" \
      --summary "$OUT/objects-summary.yaml" \
      --boxes "$OUT/dynamic-boxes.yaml" \
      --diagnostics "$OUT/localization-diagnostics.yaml" \
      > "$OUT/nuc-shadow-qa.txt" 2> "$OUT/validator-last-error.txt"; then
    break
  fi
  if [ "$attempt" -eq 10 ]; then
    cat "$OUT/validator-last-error.txt" >&2
    exit 24
  fi
done
cp "$OUT/nuc-shadow-qa.txt" "$OUT/integration-green.txt"
