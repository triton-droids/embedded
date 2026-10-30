#!/usr/bin/env bash
# Build the MuJoCo simulation and its shared control/policy dependencies.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-jazzy}"
cd "$repo_dir"
ROS_DISTRO="$ros_distro" source scripts/setup_rosenv.sh
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install --executor sequential \
  --packages-up-to humanoid_sim \
  --cmake-args "-DPython3_EXECUTABLE=$repo_dir/rosenv/bin/python"
printf '\nReady. In each ROS terminal run:\nROS_DISTRO=%s source %s/scripts/activate_rosenv.sh\n' \
  "$ros_distro" "$repo_dir"
