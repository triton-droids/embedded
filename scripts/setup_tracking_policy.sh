#!/usr/bin/env bash
# Build only the packages needed for the ONNX/ROS bench, without SDK dependencies.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
source "/opt/ros/${ros_distro}/setup.bash"
cd "$repo_dir"
if [[ ! -x rosenv/bin/python ]]; then
  /usr/bin/python3 -m venv rosenv
fi
touch rosenv/COLCON_IGNORE
source rosenv/bin/activate
python -m pip install -r requirements-policy-onnx.txt
colcon build --symlink-install \
  --base-paths humanoid_control Attitude_Sensing/src \
  --packages-select motor_control_interfaces humanoid_safety motor_control_hybrid attitude_sensing_pkg \
  --cmake-args "-DPython3_EXECUTABLE=$repo_dir/rosenv/bin/python"
printf '\nReady. In each ROS terminal run:\nsource /opt/ros/%s/setup.bash\nsource %s/rosenv/bin/activate\nsource %s/install/setup.bash\n' \
  "$ros_distro" "$repo_dir" "$repo_dir"
