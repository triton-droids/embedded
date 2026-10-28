#!/usr/bin/env bash
# One-time dev container setup: rosdep, the rosenv venv, and shell sourcing.
# Set INSTALL_TORCH=1 to also install CPU-only torch for the legacy policy_bridge_node.
set -eo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
source /opt/ros/humble/setup.bash

# The image drops apt lists; rosdep's apt installs need them.
sudo apt-get update
rosdep update
rosdep install -y -r --ignore-src --rosdistro humble \
  --from-paths humanoid_control Attitude_Sensing/src humanoid_vision \
  humanoid_arm_description humanoid_leg_description \
  humanoid_arm_moveit_config humanoid_leg_moveit_config \
  || echo "Warning: some rosdep keys failed to install; see above." >&2

# A venv made on another OS/Python cannot be reused (see README).
if [[ -x rosenv/bin/python ]] && ! rosenv/bin/python -c 'import sys; assert sys.version_info[:2] == (3, 10)' 2>/dev/null; then
  echo "Error: rosenv/ was not created for this container's Python 3.10; delete it and rebuild." >&2
  exit 1
fi
if [[ ! -x rosenv/bin/python ]]; then
  /usr/bin/python3 -m venv rosenv
fi
touch rosenv/COLCON_IGNORE

# Torch is excluded by default: the default wheel is several GB and only the
# legacy policy bridge uses it. One resolve keeps Humble's numpy<2 pin.
reqs="$(mktemp)"
grep -v '^torch' humanoid_control/motor_control_hybrid/requirements.txt > "$reqs"
# Without PYTHONPATH, pip only checks the venv, not ROS's /opt/ros packages.
env -u PYTHONPATH rosenv/bin/python -m pip install -r "$reqs" -r requirements-policy-onnx.txt \
  -r simulation/humanoid_sim/requirements.txt 'pytest>=7,<9'
rm -f "$reqs"
if [[ "${INSTALL_TORCH:-0}" == "1" ]]; then
  env -u PYTHONPATH rosenv/bin/python -m pip install torch 'numpy<2' --index-url https://download.pytorch.org/whl/cpu \
    --extra-index-url https://pypi.org/simple
fi
# Stops scripts/setup_rosenv.sh from reinstalling the full requirements (CUDA torch).
touch rosenv/.requirements.stamp

line="source $repo/.devcontainer/ros_env.sh"
grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc

cat <<'EOF'

Dev container ready. In a new terminal:
  colcon build --symlink-install
  source install/setup.bash
EOF
