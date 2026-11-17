#!/usr/bin/env bash
# Jetson (JetPack 6) side of the direct wired link to the WSL simulator host.
# Same NetworkManager profile as the JetPack 7 Jetson (wsl-direct on enP8p1s0),
# with address 192.168.50.3 so both Jetsons can share the 192.168.50.0/24 link.
#   sudo bash jetson_link_setup.sh            # create/update and bring up
#   sudo bash jetson_link_setup.sh remove     # delete the profile again
set -euo pipefail
ifname="${IFNAME:-enP8p1s0}" addr="${ADDR:-192.168.50.3/24}" peer="${PEER:-192.168.50.1}" con=wsl-direct
[[ $EUID -eq 0 ]] || { echo "Run with sudo." >&2; exit 1; }
if [[ "${1:-}" == remove ]]; then
  nmcli con delete "$con"
  exit 0
fi
if nmcli -t -f NAME con show | grep -qx "$con"; then
  nmcli con mod "$con" connection.interface-name "$ifname" ipv4.method manual ipv4.addresses "$addr"
else
  nmcli con add type ethernet ifname "$ifname" con-name "$con" ipv4.method manual ipv4.addresses "$addr"
fi
# No gateway/DNS: Wi-Fi and Tailscale keep the default route.
nmcli con mod "$con" ipv4.never-default yes ipv4.gateway '' ipv4.dns '' ipv6.method disabled \
  connection.autoconnect yes connection.autoconnect-priority 10
# DDS receive buffer (cyclonedds_split.xml asks for 16 MB).
if (( $(cat /proc/sys/net/core/rmem_max) < 33554432 )); then
  echo 'net.core.rmem_max=33554432' > /etc/sysctl.d/60-cyclonedds.conf
  sysctl -q -p /etc/sysctl.d/60-cyclonedds.conf
fi
if [[ "$(cat "/sys/class/net/$ifname/carrier" 2>/dev/null)" != 1 ]]; then
  echo "Profile $con saved; $ifname has no carrier (cable not plugged in?). It comes up on link."
  exit 0
fi
nmcli con up "$con" >/dev/null
sleep 2
ip -4 -br addr show "$ifname"
echo "speed: $(cat "/sys/class/net/$ifname/speed") Mb/s  rmem_max: $(cat /proc/sys/net/core/rmem_max)"
ping -c 3 -W 2 "$peer" | tail -2
