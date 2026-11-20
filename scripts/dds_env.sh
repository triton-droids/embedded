#!/usr/bin/env bash
# Select CycloneDDS for this shell; sourced by scripts/activate_rosenv.sh.
#
#   DDS_PROFILE=local  (default) loopback only: the robot's own graph, robust to
#                      Wi-Fi / VPN changes. Use for control and bench work.
#                      Also points Fast DDS processes (micro-ROS Agent) at the
#                      matching loopback profile, config/dds/fastdds_localhost.xml.
#   DDS_PROFILE=lan    loopback + the LAN interface (multicast discovery), for an
#                      operator PC running RViz or ros2 CLI against the robot.
#                      Optional: DDS_INTERFACE=<iface> (default: default-route
#                      interface), DDS_PEER_IP=<other host> for unicast peers.
#   DDS_PROFILE=none   leave RMW_IMPLEMENTATION / CYCLONEDDS_URI untouched.
#
# An explicitly exported RMW_IMPLEMENTATION other than CycloneDDS is respected
# with a warning. Every process of one ROS graph must use the same profile.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Run: source scripts/dds_env.sh (or scripts/activate_rosenv.sh)" >&2
  exit 2
fi

_humanoid_dds_env() {
  local repo profile rmem
  repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  profile="${DDS_PROFILE:-local}"
  [[ "$profile" == none ]] && return 0
  if [[ "$profile" != local && "$profile" != lan ]]; then
    echo "DDS_PROFILE must be local, lan or none (got '$profile')" >&2
    return 1
  fi
  if [[ -n "${RMW_IMPLEMENTATION:-}" && "$RMW_IMPLEMENTATION" != rmw_cyclonedds_cpp ]]; then
    echo "Warning: RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION is set; this workspace expects" \
         "rmw_cyclonedds_cpp. Unset it or use DDS_PROFILE=none to silence this." >&2
    return 0
  fi
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  if [[ "$profile" == lan ]]; then
    if [[ -z "${DDS_INTERFACE:-}" ]]; then
      DDS_INTERFACE="$(ip -o route get 1.1.1.1 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p')"
    fi
    if [[ -z "$DDS_INTERFACE" ]] || ! ip link show "$DDS_INTERFACE" >/dev/null 2>&1; then
      echo "DDS_PROFILE=lan: no usable interface; set DDS_INTERFACE=<iface>" >&2
      return 1
    fi
    export DDS_INTERFACE
  fi
  export CYCLONEDDS_URI="file://$repo/config/dds/cyclonedds_$profile.xml"
  # Fast DDS processes (the micro-ROS Agent embeds Fast DDS) must be on the
  # same loopback-only footing as the local CycloneDDS profile, or the two never
  # discover each other. On the LAN both sides use multicast discovery.
  if [[ "$profile" == local ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$repo/config/dds/fastdds_localhost.xml"
  elif [[ "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" == "$repo/config/dds/fastdds_localhost.xml" ]]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE
  fi
  # The profiles pin their interfaces; ROS_LOCALHOST_ONLY would add a second
  # loopback selection (fatal on Humble) and is deprecated on Jazzy.
  unset ROS_LOCALHOST_ONLY
  rmem="$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)"
  if (( rmem < 16777216 )); then
    echo "Note: net.core.rmem_max=$rmem caps the 16 MB DDS receive buffer; raise it with" \
         "sudo sysctl -w net.core.rmem_max=33554432 (persist in /etc/sysctl.d)." >&2
  fi
}
_humanoid_dds_env
_humanoid_dds_env_result=$?
unset -f _humanoid_dds_env
return "$_humanoid_dds_env_result"
