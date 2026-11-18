#!/usr/bin/env bash
# Source this in each terminal; installation is handled by setup_rosenv.sh.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Run: source scripts/activate_rosenv.sh" >&2
  exit 2
fi

_activate_workspace_rosenv() {
  local repo distro ros_setup
  repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  distro="${ROS_DISTRO:-jazzy}"
  ros_setup="/opt/ros/$distro/setup.bash"
  if [[ ! -f "$ros_setup" || ! -x "$repo/rosenv/bin/python" ]]; then
    echo "ROS or rosenv is missing. Run: source scripts/setup_rosenv.sh" >&2
    return 1
  fi
  source "$ros_setup" || return 1
  source "$repo/rosenv/bin/activate" || return 1
  if [[ -f "$repo/install/setup.bash" ]]; then
    source "$repo/install/setup.bash" || return 1
  fi
  source "$repo/scripts/rosenv_paths.sh"
}
_activate_workspace_rosenv
_activate_rosenv_result=$?
unset -f _activate_workspace_rosenv
return "$_activate_rosenv_result"
