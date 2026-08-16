#!/usr/bin/env bash
# 폰과 NUC를 페어링할 수 있는 상태로 만든다.
#
# 왜 필요한가:
#   1) BlueZ 의 discoverable 은 DiscoverableTimeout(기본 180초) 뒤에 저절로 꺼진다.
#      꺼지면 hci0 에서 ISCAN 플래그가 사라지고, 폰의 "사용 가능한 기기" 목록에
#      NUC 가 아예 나타나지 않는다.
#   2) bluetoothctl 을 `bluetoothctl -- discoverable on` 처럼 한 번만 실행하면
#      명령 직후 프로세스가 끝나면서 페어링 에이전트가 사라진다. 에이전트가 없으면
#      폰이 NUC 를 찾더라도 페어링 요청이 거절된다.
#
# 그래서 이 스크립트는 stdin 을 열어둔 채 bluetoothctl 을 상주시켜서 에이전트를
# 유지하고, DiscoverableTimeout 을 0(무제한)으로 바꾼다. sudo 가 필요 없다.
#
#   ./scripts/nuc_bluetooth_pair.sh          # 페어링 모드 켜기
#   ./scripts/nuc_bluetooth_pair.sh --off    # 페어링 끝난 뒤 되돌리기
#
# 페어링이 끝나면 --off 로 되돌릴 것. 상시 discoverable 은 아무나 페어링을 시도할
# 수 있다는 뜻이고, 이미 본딩된 폰은 discoverable 이 꺼져 있어도 잘 붙는다.

set -o pipefail

AGENT_LOG=/tmp/bt_pair_agent.log

if [ "${1:-}" = "--off" ]; then
    pkill -f "bluetoothctl" 2>/dev/null
    bluetoothctl -- discoverable off >/dev/null 2>&1
    bluetoothctl -- discoverable-timeout 180 >/dev/null 2>&1
    echo "페어링 모드를 껐습니다 (discoverable off, timeout 180s 복구)."
    bluetoothctl show | grep -E "Discoverable:|Pairable:"
    exit 0
fi

pkill -f "bluetoothctl" 2>/dev/null
sleep 1

# stdin 을 sleep 으로 붙잡아 bluetoothctl 을 살려둔다 -> 에이전트가 유지된다.
setsid nohup bash -c '{
    printf "power on\n"
    printf "agent NoInputNoOutput\n"
    printf "default-agent\n"
    printf "discoverable-timeout 0\n"
    printf "pairable on\n"
    printf "discoverable on\n"
    sleep 100000
} | bluetoothctl' >"$AGENT_LOG" 2>&1 &

sleep 5

echo "=== 어댑터 상태 ==="
bluetoothctl show | grep -E "Name:|Alias:|Powered:|Discoverable:|DiscoverableTimeout:|Pairable:"

echo
echo "=== hci0 스캔 플래그 (ISCAN 이 있어야 폰에 보입니다) ==="
flags="$(hciconfig hci0 | sed -n '3p')"
echo "   $flags"
case "$flags" in
    *ISCAN*) echo "   OK: 검색 가능 상태입니다." ;;
    *)       echo "   경고: ISCAN 이 없습니다. 폰에서 안 보입니다." ;;
esac

echo
echo "=== 에이전트 ==="
if grep -qi "Agent registered" "$AGENT_LOG" 2>/dev/null; then
    echo "   OK: 페어링 에이전트 등록됨 (NoInputNoOutput / Just Works)"
else
    echo "   경고: 에이전트 등록 로그가 없습니다 -> cat $AGENT_LOG"
fi

echo
echo "이제 폰에서: 설정 > 블루투스 > '$(bluetoothctl show | sed -n 's/.*Alias: //p')' 선택"
echo "페어링이 끝나면  ./scripts/nuc_bluetooth_pair.sh --off  로 되돌리세요."
