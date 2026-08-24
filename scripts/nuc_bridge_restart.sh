#!/usr/bin/env bash
# 브릿지를 재시작한다.
#
# 주의: pkill -f ros1_bluetooth_bridge.py 를 SSH 한 줄 명령으로 보내면 패턴이
# 자기 명령줄과도 매칭되어 셸 자신을 죽인다(exit 127 처럼 보임). 대괄호 트릭으로
# 자기 자신을 제외한다.
#
#   ./scripts/nuc_bridge_restart.sh                 # 관찰 전용
#   ./scripts/nuc_bridge_restart.sh --allow-commands
#   ./scripts/nuc_bridge_restart.sh --stop
#
# 앱의 [로컬 켜기]는 start_wheelchair_localization.sh 를 그대로 실행하는데, 그
# 스크립트는 컨트롤러를 $PROFILE 로 고르고 기본값이 pursuit 이다. 현장 주행은
# PROFILE=dwa 로 띄우므로, 아무것도 넘기지 않으면 같은 버튼이 마지막에 주행한
# 것과 다른 컨트롤러를 기동한다. 그래서 여기서 현장 기본값을 박아둔다. 다른
# 프로파일이 필요하면 환경변수로 덮어쓴다:
#
#   PROFILE=pursuit ./scripts/nuc_bridge_restart.sh --allow-commands --allow-scripts

cd "$HOME/wheelchair_localization_src" || exit 1

pkill -f '[r]os1_bluetooth_bridge' 2>/dev/null
sleep 2

if [ "${1:-}" = "--stop" ]; then
    echo "브릿지를 중지했습니다."
    exit 0
fi

source /opt/ros/noetic/setup.bash
[ -f devel/setup.bash ] && source devel/setup.bash

JOB_PROFILE="${PROFILE:-dwa}"
JOB_SAFETY="${SAFETY_POLICIES:-true}"

setsid nohup python3 scripts/ros1_bluetooth_bridge.py --job-env "PROFILE=$JOB_PROFILE" --job-env "SAFETY_POLICIES=$JOB_SAFETY" "$@" \
    </dev/null >/tmp/btbridge.log 2>&1 &

sleep 5
echo "=== 브릿지 로그 ==="
cat /tmp/btbridge.log
echo "=== 프로세스 ==="
pgrep -af '[r]os1_bluetooth_bridge' | head -2
echo "=== SDP(Serial Port) 등록 개수 ==="
bluetoothctl show | grep -c "Serial Port"
echo "=== [로컬 켜기]가 기동할 프로파일 ==="
echo "PROFILE=$JOB_PROFILE SAFETY_POLICIES=$JOB_SAFETY"
