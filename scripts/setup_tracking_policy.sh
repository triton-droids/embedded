#!/usr/bin/env bash
# Build only the packages needed for the ONNX/ROS bench, without SDK dependencies.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-jazzy}"
cd "$repo_dir"
ROS_DISTRO="$ros_distro" source scripts/setup_rosenv.sh
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install --executor sequential \
  --base-paths humanoid_control "imu_process/Attitude_Sensing(Depreciated)/src" \
  --packages-select motor_control_interfaces humanoid_safety motor_control_hybrid attitude_sensing_pkg \
  --cmake-args "-DPython3_EXECUTABLE=$repo_dir/rosenv/bin/python"
printf '\nReady. In each ROS terminal run:\nROS_DISTRO=%s source %s/scripts/activate_rosenv.sh\n' \
  "$ros_distro" "$repo_dir"
