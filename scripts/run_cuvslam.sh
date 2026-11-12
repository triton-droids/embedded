#!/usr/bin/env bash
# cuVSLAM stereo visual odometry for the simulation, in the Isaac ROS 3.2 container (Jetson, JetPack 6).
#   scripts/run_cuvslam.sh build    # once: derived image with cuVSLAM + CycloneDDS
#   scripts/run_cuvslam.sh          # run in the foreground (Ctrl+C stops the container)
# Needs the base image isaac_ros_dev-aarch64 (isaac_ros_common release-3.2:
#   scripts/build_image_layers.sh --image_key aarch64.ros2_humble --image_name isaac_ros_dev-aarch64)
# and a simulation started with enable_stereo:=true on the same ROS_DOMAIN_ID.
# DDS matches scripts/ball_chase_walk.sh: CycloneDDS, localhost only.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image=humanoid_cuvslam:3.2
if [[ "${1:-}" == build ]]; then
  exec docker build -t "$image" -f "$repo_dir/humanoid_control/humanoid_localization/docker/Dockerfile.cuvslam" \
    "$repo_dir/humanoid_control/humanoid_localization/docker"
fi
docker image inspect "$image" >/dev/null 2>&1 || { echo "Run '$0 build' first" >&2; exit 1; }
config=cyclonedds_sim_humble.xml  # The container is Humble.
exec docker run --rm --name humanoid_cuvslam --runtime nvidia --privileged \
  --network host --ipc host --pid host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
  -v /usr/share/vpi3:/usr/share/vpi3 \
  -v "$repo_dir":/workspaces/humanoid:ro \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e ROS_LOCALHOST_ONLY=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI="file:///workspaces/humanoid/simulation/humanoid_sim/config/$config" \
  --entrypoint /bin/bash "$image" -c \
  'source /opt/ros/humble/setup.bash && exec ros2 launch /workspaces/humanoid/humanoid_control/humanoid_localization/launch/cuvslam.launch.py'
