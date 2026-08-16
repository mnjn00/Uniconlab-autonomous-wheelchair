#!/usr/bin/env bash
# Find duplicate and orphaned nodes on the NUC.
#
# The failure this looks for: a node from an earlier bringup outlives the
# cleanup sweep, the replacement registers the same name, and the master
# evicts the old one. Eviction unregisters the loser but does not kill it, so
# the process keeps running - still burning CPU, still holding its
# publishers. On 2026-08-06 an mpc_follower and an obstacle_clusters from a
# run 14 minutes earlier were still alive at 447% and 177%.
#
# Nothing reports this on its own. rosnode list shows one of each, because
# the orphan is exactly the one the master forgot. The two things that do
# show it are the process table and the publisher count on a topic.
#
# Read-only: this reports, it does not kill anything.
set -u

source /opt/ros/noetic/setup.bash 2>/dev/null || true

NODES='moving_icp_localizer safety_gate obstacle_clusters waypoint_follower
       dwa_follower mpc_follower tip_guard fastlio_mapping auto_initial_pose
       route_identity_publisher bounded_cloud_preview_node
       livox_ros_driver2 wheel_cmd'

# The topics where a second publisher actually changes what the chair does.
TOPICS='/fast_lio_icp/pose /cmd_vel /cmd_vel_raw /cmd_vel_gated
        /waypoint_follower/status /mode_cmd'

status=0

echo "=== 1. 노드별 프로세스 수 (2 이상이면 중복) ==="
for node in $NODES; do
  # Match the executable, not the roslaunch wrapper or the log path: a
  # wrapper's command line contains the node name too, and counting those
  # reports duplicates that are not there.
  pids="$(pgrep -f "[${node:0:1}]${node:1}" 2>/dev/null || true)"
  real=""
  for pid in $pids; do
    args="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
    case "$args" in
      *roslaunch*|*rosmaster*|*" -f "*) continue ;;
    esac
    case "$args" in
      *"$node"*) real="$real $pid" ;;
    esac
  done
  count="$(echo $real | wc -w)"
  [ "$count" -eq 0 ] && continue
  if [ "$count" -gt 1 ]; then
    printf '  !! %-26s %s개  pid:%s\n' "$node" "$count" "$real"
    status=1
    for pid in $real; do
      printf '       pid %-7s cpu %-7s 시작 %s\n' "$pid" \
        "$(ps -o pcpu= -p "$pid" 2>/dev/null | tr -d ' ')" \
        "$(ps -o lstart= -p "$pid" 2>/dev/null)"
    done
  else
    printf '     %-26s 1개  cpu %s\n' "$node" \
      "$(ps -o pcpu= -p $real 2>/dev/null | tr -d ' ')"
  fi
done

echo
echo "=== 2. 토픽별 퍼블리셔 수 (2 이상이면 유령이 살아있다) ==="
if ! timeout 5 rosnode list >/dev/null 2>&1; then
  echo "  roscore 없음 - 건너뜀"
else
  for topic in $TOPICS; do
    info="$(timeout 8 rostopic info "$topic" 2>/dev/null)"
    [ -z "$info" ] && continue
    n="$(echo "$info" | sed -n '/^Publishers:/,/^Subscribers:/p' \
         | grep -c '^ \* ')"
    if [ "$n" -gt 1 ]; then
      printf '  !! %-32s 퍼블리셔 %s\n' "$topic" "$n"
      echo "$info" | sed -n '/^Publishers:/,/^Subscribers:/p' \
        | grep '^ \* ' | sed 's/^/       /'
      status=1
    else
      printf '     %-32s 퍼블리셔 %s\n' "$topic" "$n"
    fi
  done
fi

echo
echo "=== 3. 등록되지 않았는데 살아있는 프로세스 (축출된 유령) ==="
if timeout 5 rosnode list >/dev/null 2>&1; then
  registered="$(timeout 8 rosnode list 2>/dev/null)"
  for node in $NODES; do
    running="$(pgrep -fc "[${node:0:1}]${node:1}" 2>/dev/null || echo 0)"
    [ "$running" -eq 0 ] && continue
    if ! echo "$registered" | grep -q "$node"; then
      printf '  !! %-26s 프로세스는 있는데 rosnode list 에 없음\n' "$node"
      status=1
    fi
  done
  [ "$status" -eq 0 ] && echo "     없음"
else
  echo "  roscore 없음 - 건너뜀"
fi

echo
echo "=== 4. 전체 부하 ==="
nproc | sed 's/^/  코어 /'
uptime | sed 's/.*load/  load/'
ps -eo pcpu,comm --sort=-pcpu | head -6 | sed 's/^/  /'

echo
if [ "$status" -ne 0 ]; then
  echo "판정: 중복/유령 있음. start_wheelchair_localization.sh 를 다시 돌리면"
  echo "      1단계 청소가 걷어냅니다 (그래도 남으면 SIGKILL 합니다)."
else
  echo "판정: 중복 없음."
fi
exit "$status"
