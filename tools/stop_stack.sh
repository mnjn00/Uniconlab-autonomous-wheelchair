#!/usr/bin/env bash
# 스택 내리기 -- take the whole running stack down. The counterpart of
# start_wheelchair_localization.sh, and the [스택 내리기] button's script.
#
# Everything the bring-up started goes: the drive, the followers and guards,
# the wheel base nodes, the lidar, FAST-LIO, the localizer, rviz and the black
# box. roscore is the single deliberate exception, for the reason below.
#
# The node list is NOT duplicated here. start_wheelchair_localization.sh already
# carries a sweep that the field corrected several times -- it has to reach what
# roslaunch starts and not only what the script detaches itself, because a node
# that outlives the wrapper keeps its ROS name, gets evicted by a name conflict
# on the next bring-up, and survives as an unregistered orphan still holding its
# publishers. Two moving_icp_localizers means /fast_lio_icp/pose has two, and the
# follower believes whichever arrives last. A second copy of that list here would
# drift from the one a test pins, and a teardown that misses a node is precisely
# the failure it was written against. So read it from there, and refuse to run if
# it cannot be found rather than tearing down a guess.
#
# roscore is deliberately left up. The Bluetooth bridge holds a rospy node
# against it; dropping the master would take the operator's link down with the
# stack, and [로컬 켜기] cannot be pressed from a dashboard that just went dark.
# start_wheelchair_localization.sh starts roscore only if none is running, so
# leaving it costs nothing on the way back up.

set -u

BRINGUP="${BRINGUP:-$HOME/start_wheelchair_localization.sh}"

if [ ! -f "$BRINGUP" ]; then
    echo "REFUSE: $BRINGUP 이 없어 종료 대상 목록을 읽을 수 없습니다." >&2
    exit 2
fi

# The one line that owns the list: `for pattern in '[r]oslaunch' ... ; do`
PATTERN_LINE="$(sed -n "s/^for pattern in \(.*\); do[[:space:]]*$/\1/p" "$BRINGUP" | head -1)"
if [ -z "$PATTERN_LINE" ]; then
    echo "REFUSE: $BRINGUP 에서 노드 목록(for pattern in ...)을 찾지 못했습니다." >&2
    echo "        스크립트가 바뀌었다면 tools/stop_stack.sh 의 파싱을 맞춰주세요." >&2
    exit 2
fi
eval "set -- $PATTERN_LINE"

# The bring-up sweep is authoritative but not complete: it launches
# `rosrun static_livox_localization stop_watchdog.py` and never sweeps it, so
# every bring-up so far has run on top of the previous run's stop_watchdog.
# Verified 2026-08-23 -- it was the one process left alive by the first real
# teardown. Rather than pin a rival list here, derive what the script detaches
# and add anything its own patterns do not already cover. A gap in their list
# then shows up as an extra pattern instead of as an orphan.
EXTRA=""
for node in $(grep -oE "rosrun [a-z_]+ [A-Za-z_]+\.py" "$BRINGUP"               | awk "{print \$3}" | sed "s/\.py$//" | sort -u); do
    covered=0
    for pattern in "$@"; do
        case "$node" in *"$(printf %s "$pattern" | tr -d "[]")"*) covered=1 ;; esac
    done
    [ "$covered" -eq 0 ] && EXTRA="$EXTRA $node"
done
if [ -n "$EXTRA" ]; then
    echo "기동 스크립트 스윕이 놓친 노드:$EXTRA"
    set -- "$@" $EXTRA
fi

echo "[스택 내리기] 대상 $# 종류 (start_wheelchair_localization.sh 에서 읽음)"

# 1. Fail-safe first. Whatever happens below, the base is already out of auto
#    and the follower is paused, so the chair is on the joystick before a single
#    node goes away. stop.sh checks nothing on purpose.
if [ -x "$HOME/stop.sh" ]; then
    echo "[1/4] 주행 정지 (팔로워 정지 + mode_cmd 77)"
    "$HOME/stop.sh" || true
else
    echo "[1/4] stop.sh 없음 -- mode_cmd 77 직접 발행" >&2
    source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true
    rostopic pub -1 /mode_cmd std_msgs/Int16 77 >/dev/null 2>&1 || true
fi
sleep 1

# 2. The black box needs SIGINT, not SIGTERM: rosbag record writes its index on
#    a clean interrupt and leaves a .active file on anything harsher. The
#    bring-up sweep does not care -- it is about to start a new bag -- but a
#    teardown that corrupts the recording of the drive that just happened does.
echo "[2/4] 블랙박스 rosbag 정리 (SIGINT)"
pkill -INT -f '[r]osbag record' 2>/dev/null || true
for _ in $(seq 1 10); do
    pgrep -f '[r]osbag record' >/dev/null 2>&1 || break
    sleep 1
done

# 3. The sweep itself, in the bring-up script's own words.
echo "[3/4] 노드 종료"
for pattern in "$@"; do
    pkill -f "$pattern" 2>/dev/null || true
done

# 4. Confirm rather than assume a fixed sleep did it -- same reasoning the
#    bring-up script applies to its own sweep.
WATCH='[m]oving_icp_localizer|[s]afety_gate|[o]bstacle_clusters|[w]aypoint_follower|[d]wa_follower|[m]pc_follower|[l]ivox_ros_driver2|[f]astlio_mapping'
for _ in $(seq 1 10); do
    pgrep -f "$WATCH" >/dev/null 2>&1 || break
    sleep 1
done
survivors="$(pgrep -af "$WATCH" 2>/dev/null || true)"
if [ -n "$survivors" ]; then
    echo "[4/4] 남은 프로세스 강제 종료:" >&2
    echo "$survivors" >&2
    pkill -9 -f "$WATCH" 2>/dev/null || true
    sleep 1
else
    echo "[4/4] 남은 프로세스 없음"
fi

source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true

# A killed node stays registered with the master until something unregisters
# it, so rosnode list keeps naming processes that are gone -- the first
# teardown left seven of them, every one answering "connection refused" to a
# ping. Left behind they make a torn-down stack look half up, and the next
# bring-up's duplicate-node check has to reason about ghosts.
echo "[5/5] 죽은 노드 등록 정리 (rosnode cleanup)"
yes | timeout 30 rosnode cleanup >/dev/null 2>&1 || true

echo "=== 남아 있는 ROS 노드 ==="
rosnode list 2>/dev/null || echo "(roscore 응답 없음)"
echo
echo "스택 내리기 완료. roscore 와 블루투스 브릿지는 그대로 둡니다."
echo "다시 올리려면 앱에서 [로컬 켜기]."
