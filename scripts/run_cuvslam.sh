#!/usr/bin/env bash
# cuVSLAM (Isaac ROS 4.6, JetPack 7.2.x) on the D435 infrared pair, in a container.
# See humanoid_control/humanoid_localization/README.md. The host must already run
# the camera and vio_localizer (ros2 launch humanoid_localization vio_d435.launch.py).
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image=humanoid_cuvslam:4.6-jp7
mode=run
case "${1:-}" in
  check|build|run) mode="$1"; shift ;;
  -h|--help) echo "Usage: $0 [check|build|run] [ROS launch arguments...]"; exit 0 ;;
  *:=*|'') ;;
  *) echo "Usage: $0 [check|build|run] [ROS launch arguments...]" >&2; exit 2 ;;
esac
if [[ "$mode" != run && $# -gt 0 ]]; then
  echo "Launch arguments are only accepted in run mode." >&2
  exit 2
fi
# A container cannot replace the host BSP, driver or firmware.
if [[ "$(uname -m)" != aarch64 ]] ||
   ! grep -Eq '^# R39 \(release\), REVISION: 2\.' /etc/nv_tegra_release 2>/dev/null; then
  echo "Requires a Jetson with JetPack 7.2.x (L4T R39.2.x)." >&2
  exit 1
fi
if ! grep -qx 'VERSION_ID="24.04"' /etc/os-release; then
  echo "Requires the JetPack 7 Ubuntu 24.04 userspace." >&2
  exit 1
fi
if [[ "$mode" == check ]]; then
  echo "Host BSP and Ubuntu match the JetPack 7.2.x deployment target."
  exit 0
fi
command -v docker >/dev/null || { echo "Docker is required." >&2; exit 1; }
if [[ "$mode" == build ]]; then
  : "${ISAAC_ROS_BASE_IMAGE:?Set ISAAC_ROS_BASE_IMAGE to an Isaac ROS 4.6 JetPack 7.2 Jazzy development image; see README.md}"
  exec docker build --build-arg "BASE_IMAGE=$ISAAC_ROS_BASE_IMAGE" \
    -t "$image" -f "$repo_dir/humanoid_control/humanoid_localization/docker/Dockerfile.cuvslam" \
    "$repo_dir/humanoid_control/humanoid_localization/docker"
fi
docker image inspect "$image" >/dev/null 2>&1 || { echo "Run '$0 build' first" >&2; exit 1; }
# Same DDS profile as the host graph (scripts/dds_env.sh, default DDS_PROFILE=local).
profile="${DDS_PROFILE:-local}"
if [[ "$profile" != local && "$profile" != lan ]]; then
  echo "DDS_PROFILE must be local or lan for the container (got '$profile')" >&2
  exit 1
fi
name="${ISAAC_CONTAINER_NAME:-humanoid_cuvslam}"
launch=/workspaces/humanoid/humanoid_control/humanoid_localization/launch/cuvslam.launch.py
cpuset=()
[[ -n "${ISAAC_CPUSET:-}" ]] && cpuset=(--cpuset-cpus "$ISAAC_CPUSET")
exec docker run --rm --name "$name" --runtime nvidia "${cpuset[@]}" \
  --network host --ipc host --pid host \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v "$repo_dir":/workspaces/humanoid:ro \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI="file:///workspaces/humanoid/config/dds/cyclonedds_$profile.xml" \
  -e DDS_INTERFACE="${DDS_INTERFACE:-}" -e DDS_PEER_IP="${DDS_PEER_IP:-}" \
  --entrypoint /bin/bash "$image" -c \
  'source /opt/ros/jazzy/setup.bash && exec ros2 launch "$@"' bash "$launch" "$@"
