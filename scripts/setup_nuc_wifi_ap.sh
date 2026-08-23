#!/usr/bin/env bash
# ==============================================================================
# Setup Intel NUC Wi-Fi Access Point (Hotspot) for Autonomous Wheelchair
# Target OS: Ubuntu 20.04 LTS (NetworkManager / nmcli)
# ==============================================================================

set -e

WIFI_INTERFACE="${WIFI_INTERFACE:-wlan0}"
AP_SSID="${AP_SSID:-Wheelchair_NUC_AP}"
AP_PASSWORD="${AP_PASSWORD:-wheelchair2026!}"
GATEWAY_IP="192.168.12.1/24"

echo "=== Setting up Intel NUC Wi-Fi Hotspot ==="
echo "Interface : ${WIFI_INTERFACE}"
echo "SSID      : ${AP_SSID}"
echo "Gateway IP: ${GATEWAY_IP}"

if ! command -v nmcli &> /dev/null; then
    echo "[ERROR] NetworkManager (nmcli) is not installed. Run: sudo apt install network-manager"
    exit 1
fi

# Delete existing hotspot profile if present
nmcli connection delete id "${AP_SSID}" 2>/dev/null || true

echo "Creating new Wi-Fi Hotspot profile..."
nmcli connection add type wifi ifname "${WIFI_INTERFACE}" con-name "${AP_SSID}" autoconnect yes ssid "${AP_SSID}"
nmcli connection modify "${AP_SSID}" 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared ipv4.addresses "${GATEWAY_IP}"
nmcli connection modify "${AP_SSID}" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "${AP_PASSWORD}"

echo "Activating Hotspot connection..."
nmcli connection up "${AP_SSID}"

echo "=== Wi-Fi Hotspot setup complete! ==="
echo "Connect your Android device to SSID: ${AP_SSID}"
echo "Android App target IP: 192.168.12.1 Port: 8081"
