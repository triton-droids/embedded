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
# Attach detected serial device as can0 interface
sudo slcand -o -c -s6 "${ACM_DEV}" can0
# Bring down can0 interface for configuration
sudo ip link set can0 down
# Set CAN bitrate to 1 Mbps
sudo ip link set can0 type can bitrate 1000000
# Bring up can0 interface
sudo ip link set can0 up
