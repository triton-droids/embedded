#!/usr/bin/env bash
# One-time dev container setup: rosdep, the rosenv venv, and shell sourcing.
set -eo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
source /opt/ros/humble/setup.bash

# The image drops apt lists; rosdep's apt installs need them.
sudo apt-get update
rosdep update
rosdep install -y -r --ignore-src --rosdistro humble \
  --from-paths humanoid_control humanoid_vision \
  simulation/humanoid_sim \
  humanoid_leg_description \
  || echo "Warning: some rosdep keys failed to install; see above." >&2

# Share dependency resolution, Python ABI checks and activation with native hosts.
ROS_DISTRO=humble source scripts/setup_rosenv.sh

line="source $repo/.devcontainer/ros_env.sh"
grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc

cat <<'EOF'

Dev container ready. In a new terminal:
  CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install --executor sequential
  source install/setup.bash
EOF
