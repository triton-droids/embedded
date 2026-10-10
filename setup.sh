#!/usr/bin/env bash
set -euo pipefail

# Find exactly one connected /dev/ttyACM[0-5] device.
acm_devices=()
for i in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
    dev="/dev/ttyACM${i}"
    if [[ -c "$dev" ]]; then
        acm_devices+=("$dev")
    fi
done

if (( ${#acm_devices[@]} == 0 )); then
    echo "Error: no /dev/ttyACM[0-5] device found." >&2
    exit 1
fi

if (( ${#acm_devices[@]} > 1 )); then
    echo "Error: multiple ttyACM devices found: ${acm_devices[*]}" >&2
    echo "Expected exactly one connected device." >&2
    exit 1
fi

ACM_DEV="${acm_devices[0]}"
echo "Using serial device: ${ACM_DEV}"

# Load slcan kernel module
sudo modprobe slcan
# Attach the CANable as can0 at 1 Mbit/s. With slcan the bitrate comes from
# -s8 (S8 = 1 Mbit/s); `ip link ... type can bitrate` does not apply.
sudo slcand -o -c -s8 "${ACM_DEV}" can0
# Room for a full control cycle of frames (the default queue of 10 drops frames
# when 11 are sent per cycle), then bring the interface up.
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip -d link show can0 | grep -oE "state [A-Z-]+|ERROR-[A-Z]+|qlen [0-9]+" | tr '\n' ' '; echo
