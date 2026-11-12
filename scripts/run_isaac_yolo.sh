#!/usr/bin/env bash
# GPU YOLO detection with Isaac ROS (NITROS: encoder -> TensorRT -> YOLOv8 decoder) in the
# cuVSLAM image (scripts/run_cuvslam.sh build). Launch arguments pass through, e.g.
#   scripts/run_isaac_yolo.sh num_classes:=6 engine_file_path:=/workspaces/humanoid/.../nubot-yolo-fp16.engine
# Output: /isaac_yolo/detections (vision_msgs/Detection2DArray, 640x640 network pixels).
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_CONTAINER_NAME=humanoid_isaac_yolo \
ISAAC_LAUNCH=/workspaces/humanoid/simulation/humanoid_ball_chase/launch/isaac_yolo.launch.py \
  exec "$repo_dir/scripts/run_cuvslam.sh" "$@"
