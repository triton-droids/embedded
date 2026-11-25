#!/usr/bin/env bash
# JetPack 6 baseline: cuVSLAM in the existing Isaac ROS 3.2 / Humble image
# (humanoid_cuvslam:3.2), for comparison with the JetPack 7.2 deployment
# (scripts/run_cuvslam.sh, which forwards here on L4T R36).
#   scripts/run_cuvslam_jp6.sh check
#   scripts/run_cuvslam_jp6.sh [ROS launch arguments...]   # e.g. imu_fusion:=false
# The image is not rebuilt here: Dockerfile.cuvslam now targets Isaac ROS 4.6.
# DDS as scripts/run_cuvslam.sh: localhost only, or the two-host unicast config
# when DDS_PEER_IP is set (scripts/ball_chase_split.sh).
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image=humanoid_cuvslam:3.2
mode=run
case "${1:-}" in
  check|run) mode="$1"; shift ;;
  -h|--help) echo "Usage: $0 [check|run] [ROS launch arguments...]"; exit 0 ;;
  *:=*|'') ;;
  *) echo "Usage: $0 [check|run] [ROS launch arguments...]" >&2; exit 2 ;;
esac
if [[ "$(uname -m)" != aarch64 ]] ||
   ! grep -Eq '^# R36 \(release\)' /etc/nv_tegra_release 2>/dev/null; then
  echo "Requires Jetson with JetPack 6 (L4T R36)." >&2
  exit 1
fi
command -v docker >/dev/null || { echo "Docker is required." >&2; exit 1; }
docker image inspect "$image" >/dev/null 2>&1 || { echo "Image $image not found." >&2; exit 1; }
if [[ "$mode" == check ]]; then
  [[ $# -eq 0 ]] || { echo "Launch arguments are only accepted in run mode." >&2; exit 2; }
  echo "JetPack 6 host with $image."
  exit 0
fi
config=cyclonedds_sim_humble.xml localhost_only=1 split_env=()
if [[ -n "${DDS_PEER_IP:-}" ]]; then
  : "${DDS_LOCAL_IP:?Set DDS_LOCAL_IP with DDS_PEER_IP}"
  config=cyclonedds_split.xml localhost_only=0
  split_env=(-e DDS_LOCAL_IP="$DDS_LOCAL_IP" -e DDS_PEER_IP="$DDS_PEER_IP")
fi
name="${ISAAC_CONTAINER_NAME:-humanoid_cuvslam}"
launch="${ISAAC_LAUNCH:-/workspaces/humanoid/perception/humanoid_localization/launch/cuvslam.launch.py}"
cpuset=()
[[ -n "${ISAAC_CPUSET:-}" ]] && cpuset=(--cpuset-cpus "$ISAAC_CPUSET")
# Isaac ROS 3.2 needs the host's tegra and VPI libraries mounted.
exec docker run --rm --name "$name" --runtime nvidia --privileged "${cpuset[@]}" \
  --network host --ipc host --pid host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
  -v /usr/share/vpi3:/usr/share/vpi3 \
  -v "$repo_dir":/workspaces/humanoid:ro \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e ROS_LOCALHOST_ONLY="$localhost_only" "${split_env[@]}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI="file:///workspaces/humanoid/simulation/humanoid_sim/config/$config" \
  --entrypoint /bin/bash "$image" -c \
  'source /opt/ros/humble/setup.bash && exec ros2 launch "$@"' bash "$launch" "$@"
