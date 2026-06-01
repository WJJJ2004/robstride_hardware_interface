#!/usr/bin/env bash
set -euo pipefail

BITRATE="${1:-1000000}"
TXQLEN="${2:-4096}"

INTERFACES=("can_right_leg" "can_left_leg")

for ifname in "${INTERFACES[@]}"; do
    echo "[CAN_SETUP] configuring ${ifname}"

    sudo ip link set "${ifname}" down 2>/dev/null || true
    sudo ip link set "${ifname}" type can bitrate "${BITRATE}" loopback off
    sudo ip link set "${ifname}" txqueuelen "${TXQLEN}"
    sudo ip link set "${ifname}" up

    ip -details link show "${ifname}"

    if ! ip -details link show "${ifname}" | grep -q "state UP"; then
        echo "[CAN_SETUP][ERROR] ${ifname} is not UP"
        exit 1
    fi

    if ! ip -details link show "${ifname}" | grep -q "bitrate ${BITRATE}"; then
        echo "[CAN_SETUP][ERROR] ${ifname} bitrate mismatch"
        exit 1
    fi
done

echo "[CAN_SETUP] done"