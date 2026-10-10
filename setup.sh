#!/usr/bin/env bash
# Bring up can0 on the CANable2 adapter at 1 Mbit/s.
#   ./setup.sh                 find the CANable by its /dev/serial/by-id name
#   ./setup.sh /dev/ttyACM2    or name the device yourself
# Run again after every boot or replug. Never needs the IMU unplugged: the IMU
# board is also a /dev/ttyACM*, so the adapter is picked by name, not number.
set -euo pipefail

if [[ $# -ge 1 ]]; then
    DEV="$1"
else
    shopt -s nullglob
    matches=(/dev/serial/by-id/*CANable*)
    shopt -u nullglob
    if (( ${#matches[@]} == 0 )); then
        echo "Error: no CANable found in /dev/serial/by-id/. Is it plugged in?" >&2
        ls -l /dev/serial/by-id/ 2>/dev/null >&2 || true
        exit 1
    fi
    if (( ${#matches[@]} > 1 )); then
        echo "Error: more than one CANable found; pass the device to use:" >&2
        printf '  %s\n' "${matches[@]}" >&2
        exit 1
    fi
    DEV="${matches[0]}"
fi
echo "CANable: ${DEV} -> $(readlink -f "${DEV}")"

if ip link show can0 >/dev/null 2>&1; then
    echo "can0 already exists; restarting slcand"
    sudo ip link set can0 down || true
    sudo pkill slcand || true
    sleep 0.5
fi

sudo modprobe slcan
# With slcan the bitrate comes from -s8 (S8 = 1 Mbit/s; -s6 would be 500 kbit/s
# and the motors would never answer). `ip link ... type can bitrate` does not apply.
sudo slcand -o -c -s8 "${DEV}" can0
sleep 0.3
# Room for a full control cycle of frames (the default queue of 10 drops frames
# when 11 are sent per cycle), then bring the interface up.
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip -d link show can0 | grep -oE "state [A-Z-]+|ERROR-[A-Z]+|qlen [0-9]+" | tr '\n' ' '; echo
