#!/usr/bin/env bash
# Preflight for the Bluetooth SPP bridge on the Intel NUC (Ubuntu 20.04 / Noetic).
#
# Run this BEFORE blaming the phone.  Every check prints what to do when it fails.
# Read-only by default; pass --fix to let it unblock rfkill, start bluetoothd and
# make the adapter discoverable/pairable.
#
#   ./scripts/nuc_bluetooth_check.sh
#   ./scripts/nuc_bluetooth_check.sh --fix
#
# See docs/handoff_bluetooth_ui.md.

set -uo pipefail

CHANNEL="${CHANNEL:-1}"
FIX=0
[ "${1:-}" = "--fix" ] && FIX=1

pass=0; fail=0; warn=0
ok()   { printf '  \033[32mOK  \033[0m %s\n' "$1"; pass=$((pass+1)); }
bad()  { printf '  \033[31mFAIL\033[0m %s\n     -> %s\n' "$1" "$2"; fail=$((fail+1)); }
note() { printf '  \033[33mWARN\033[0m %s\n     -> %s\n' "$1" "$2"; warn=$((warn+1)); }
hdr()  { printf '\n\033[1m%s\033[0m\n' "$1"; }

hdr "1. Adapter present"
if command -v hciconfig >/dev/null 2>&1 && hciconfig 2>/dev/null | grep -q '^hci'; then
    ok "$(hciconfig | head -1 | cut -d: -f1) present"
elif command -v bluetoothctl >/dev/null 2>&1 && bluetoothctl list 2>/dev/null | grep -q Controller; then
    ok "$(bluetoothctl list | head -1)"
else
    bad "no Bluetooth controller found" \
        "check 'lsusb | grep -i blue' and 'dmesg | grep -i bluetooth'; the NUC's radio may be disabled in BIOS"
fi

hdr "2. Not blocked by rfkill"
if command -v rfkill >/dev/null 2>&1; then
    if rfkill list bluetooth 2>/dev/null | grep -q 'yes'; then
        if [ "$FIX" = 1 ]; then
            sudo rfkill unblock bluetooth && ok "was blocked, unblocked"
        else
            bad "rfkill has Bluetooth blocked" "sudo rfkill unblock bluetooth   (or re-run with --fix)"
        fi
    else
        ok "not soft/hard blocked"
    fi
else
    note "rfkill not installed" "sudo apt install rfkill"
fi

hdr "3. bluetoothd running"
if systemctl is-active --quiet bluetooth; then
    ok "bluetooth.service active"
else
    if [ "$FIX" = 1 ]; then
        sudo systemctl start bluetooth && ok "started bluetooth.service"
    else
        bad "bluetooth.service not active" "sudo systemctl start bluetooth   (or re-run with --fix)"
    fi
fi

hdr "4. Adapter powered, discoverable, pairable"
if command -v bluetoothctl >/dev/null 2>&1; then
    info="$(bluetoothctl show 2>/dev/null)"
    for prop in Powered Discoverable Pairable; do
        if printf '%s' "$info" | grep -q "$prop: yes"; then
            ok "$prop: yes"
        elif [ "$FIX" = 1 ]; then
            bluetoothctl -- "$(printf '%s' "$prop" | tr '[:upper:]' '[:lower:]')" on >/dev/null 2>&1 \
                && ok "$prop turned on"
        else
            bad "$prop: no" "bluetoothctl -- $(printf '%s' "$prop" | tr '[:upper:]' '[:lower:]') on   (or --fix)"
        fi
    done
    printf '%s' "$info" | grep -E 'Name:|Alias:|^Controller' | sed 's/^/       /'
else
    bad "bluetoothctl missing" "sudo apt install bluez"
fi

hdr "5. Phone is already paired (the app only lists BONDED devices)"
paired="$(bluetoothctl paired-devices 2>/dev/null || bluetoothctl devices Paired 2>/dev/null)"
if [ -n "$paired" ]; then
    ok "paired devices:"; printf '%s\n' "$paired" | sed 's/^/       /'
else
    bad "no paired devices" \
        "pair from the NUC: bluetoothctl -> power on / agent on / default-agent / discoverable on, then accept on the phone"
fi

hdr "6. RFCOMM channel $CHANNEL is free"
if command -v ss >/dev/null 2>&1 && ss -l --bluetooth 2>/dev/null | grep -q ":$CHANNEL"; then
    bad "something is already listening on RFCOMM channel $CHANNEL" \
        "find it with 'sudo ss -lp --bluetooth' and stop it, or run the bridge with --channel 2"
else
    ok "nothing else bound to channel $CHANNEL (as far as ss can tell)"
fi

hdr "7. SDP: can Android's UUID connect path find us?"
if python3 -c "import bluetooth" 2>/dev/null; then
    ok "python3-bluez present -- the bridge registers its own SDP record"
elif command -v sdptool >/dev/null 2>&1 && sdptool browse local 2>/dev/null | grep -qi "serial port"; then
    ok "an SPP record is already published (sdptool browse local)"
else
    note "no SDP record and no PyBluez" \
        "install 'sudo apt install python3-bluez', or run bluetoothd with --compat and 'sudo sdptool add --channel=$CHANNEL SP'. Without this only the app's legacy reflection path works, which newer Android blocks."
fi

hdr "8. ROS side"
if [ -z "${ROS_DISTRO:-}" ]; then
    note "ROS environment not sourced" "source /opt/ros/noetic/setup.bash && source devel/setup.bash"
else
    ok "ROS_DISTRO=$ROS_DISTRO"
    if python3 -c "from wheelchair_interfaces.msg import SafetyState" 2>/dev/null; then
        ok "wheelchair_interfaces importable -> /safety/state can be read"
    else
        bad "wheelchair_interfaces not importable" \
            "source the workspace devel/setup.bash; without it armed/estop/speed all stay null"
    fi
    if rostopic list >/dev/null 2>&1; then
        ok "roscore reachable"
        for t in /safety/state /runtime/mode; do
            rostopic list 2>/dev/null | grep -qx "$t" && ok "$t present" \
                || note "$t absent" "the dashboard field it feeds will read 'unavailable'"
        done
    else
        note "no roscore" "the bridge still serves, but every ROS-sourced field is null"
    fi
fi

hdr "9. Bridge protocol self-test (no radio, no ROS needed)"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if python3 "$here/ros1_bluetooth_bridge.py" --self-test >/tmp/bt_selftest.log 2>&1; then
    ok "protocol self-test passed"
else
    bad "protocol self-test failed" "cat /tmp/bt_selftest.log"
fi

printf '\n\033[1m%d passed, %d failed, %d warnings\033[0m\n' "$pass" "$fail" "$warn"
[ "$fail" -eq 0 ] || printf 'Fix the FAIL lines before starting the bridge.\n'
exit $(( fail > 0 ? 1 : 0 ))
