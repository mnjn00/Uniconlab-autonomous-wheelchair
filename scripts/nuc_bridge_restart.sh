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

cd "$HOME/wheelchair_localization_src" || exit 1

pkill -f '[r]os1_bluetooth_bridge' 2>/dev/null
sleep 2

if [ "${1:-}" = "--stop" ]; then
    echo "브릿지를 중지했습니다."
    exit 0
fi

source /opt/ros/noetic/setup.bash
[ -f devel/setup.bash ] && source devel/setup.bash

setsid nohup python3 scripts/ros1_bluetooth_bridge.py "$@" \
    </dev/null >/tmp/btbridge.log 2>&1 &

sleep 5
echo "=== 브릿지 로그 ==="
cat /tmp/btbridge.log
echo "=== 프로세스 ==="
pgrep -af '[r]os1_bluetooth_bridge' | head -2
echo "=== SDP(Serial Port) 등록 개수 ==="
bluetoothctl show | grep -c "Serial Port"
