#!/usr/bin/env bash
# Sensor/perception/localization-only QA. Never starts wheel or follower nodes.
set -euo pipefail

REPO="${REPO:-$HOME/unicon-wheelchair}"
WS="${WS:-$HOME/livox_static_localization_ws}"
MAP="${MAP:-$HOME/maps/merged_0707_0725_v1/field_localization_map.pcd}"
MAP_SHA256="${MAP_SHA256:-$HOME/maps/merged_0707_0725_v1/field_localization_map.sha256}"
BAND="${BAND:-$REPO/routes/20260812_route_v6_v8_safety_band.json}"
OUT="${OUT:-/tmp/ulw-evidence}"
TIMEOUT_S="${TIMEOUT_S:-45}"
mkdir -p "$OUT"

AUTONOMOUS_RE='wheel_cmd|waypoint_follower|dwa_follower|mpc_follower|safety_gate'
LAUNCHED_PIDS=()

cleanup() {
  local pid
  for pid in "${LAUNCHED_PIDS[@]}"; do
    kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
  done
  pkill -f 'obstacle_clusters.py.*_shadow_qa:=true' 2>/dev/null || true
  {
    echo "cleanup_at=$(date -Iseconds)"
    echo "remaining_autonomous_processes:"
    pgrep -af "$AUTONOMOUS_RE" || true
    echo "remaining_shadow_processes:"
    pgrep -af 'obstacle_clusters.py.*_shadow_qa:=true' || true
  } > "$OUT/cleanup-receipt.txt"
}
trap cleanup EXIT INT TERM

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

if ! rostopic list | grep -qx '/cloud_registered_body'; then
  echo "ERROR: /cloud_registered_body is absent; start sensor/localizer only" >&2
  exit 21
fi
if ! rostopic list | grep -qx '/fast_lio_icp/pose'; then
  echo "ERROR: /fast_lio_icp/pose is absent; localization is not ready" >&2
  exit 22
fi

EXPECTED_MAP_SHA="$(awk '{print $1}' "$MAP_SHA256")"
setsid rosrun static_livox_localization obstacle_clusters.py \
  _body_frame_profile:=vn100 \
  _safety_band:="$BAND" \
  _map_path:="$MAP" \
  _map_sha256:="$EXPECTED_MAP_SHA" \
  _shadow_qa:=true \
  > "$OUT/shadow-clusters.log" 2>&1 &
LAUNCHED_PIDS+=("$!")

timeout "$TIMEOUT_S" rostopic echo -n 1 /fast_lio_icp/localization_diagnostics \
  > "$OUT/localization-diagnostics.yaml"

pgrep -af "$AUTONOMOUS_RE" > "$OUT/autonomous-after.txt" && {
  echo "ERROR: autonomous process appeared during shadow QA" >&2
  exit 23
}

for attempt in $(seq 1 10); do
  timeout "$TIMEOUT_S" rostopic echo -n 1 /perception/objects_summary \
    > "$OUT/objects-summary.yaml"
  timeout "$TIMEOUT_S" rostopic echo -n 1 /perception/dynamic_boxes \
    > "$OUT/dynamic-boxes.yaml"
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
