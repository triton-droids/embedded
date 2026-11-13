#!/usr/bin/env bash
# All Isaac ROS perception in one container process: cuVSLAM (stereo VO for
# humanoid_localization) and the YOLO pipeline (/isaac_yolo/detections), instead of
# scripts/run_cuvslam.sh + scripts/run_isaac_yolo.sh as two containers.
# Launch arguments pass through to isaac_yolo.launch.py (e.g. num_classes:=6 ...).
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_CONTAINER_NAME=humanoid_isaac_perception \
ISAAC_LAUNCH=/workspaces/humanoid/simulation/humanoid_ball_chase/launch/isaac_yolo.launch.py \
  exec "$repo_dir/scripts/run_cuvslam.sh" with_cuvslam:=true "$@"
