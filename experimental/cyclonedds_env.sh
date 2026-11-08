#!/usr/bin/env bash
# Experimental, repo-local CycloneDDS (rmw_cyclonedds_cpp) for ROS 2 Humble.
# Nothing is installed system-wide: the Humble .debs are extracted into the
# gitignored local_test/cyclonedds/ on first use. Default stays FastRTPS.
#
#   source scripts/activate_rosenv.sh
#   source experimental/cyclonedds_env.sh
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Run: source experimental/cyclonedds_env.sh" >&2
  exit 2
fi

_experimental_cyclonedds_env() {
  local repo dir prefix pkgs f
  repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  dir="$repo/local_test/cyclonedds"
  prefix="$dir/root/opt/ros/humble"
  pkgs=(ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds
        ros-humble-iceoryx-binding-c ros-humble-iceoryx-posh ros-humble-iceoryx-hoofs)
  if [[ "${ROS_DISTRO:-}" != humble ]]; then
    echo "experimental CycloneDDS is packaged for Humble only (ROS_DISTRO=${ROS_DISTRO:-unset})" >&2
    return 1
  fi
  if [[ ! -f "$prefix/lib/librmw_cyclonedds_cpp.so" ]]; then
    mkdir -p "$dir/debs" || return 1
    (cd "$dir/debs" && apt-get download "${pkgs[@]}") || return 1
    for f in "$dir"/debs/*.deb; do dpkg -x "$f" "$dir/root" || return 1; done
  fi
  case ":${AMENT_PREFIX_PATH:-}:" in *":$prefix:"*) ;;
    *) export AMENT_PREFIX_PATH="$prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}" ;; esac
  case ":${LD_LIBRARY_PATH:-}:" in *":$prefix/lib:"*) ;;
    *) export LD_LIBRARY_PATH="$prefix/lib:$prefix/lib/$(uname -m)-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;; esac
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (experimental, repo-local)"
}
_experimental_cyclonedds_env
_cyclonedds_env_result=$?
unset -f _experimental_cyclonedds_env
return "$_cyclonedds_env_result"
