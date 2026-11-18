#!/usr/bin/env bash
# Build a device-specific FP16 engine, then install the CUDA vision package.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
trtexec_bin="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
engine_dir="$repo_root/.deps/tensorrt"
mkdir -p "$engine_dir"
"$trtexec_bin" --onnx=humanoid_vision/models/yolov8n-face-lindevs.onnx \
  --saveEngine="$engine_dir/yolov8n-face-fp16.engine.tmp" \
  --fp16 --skipInference --memPoolSize=workspace:512 \
  --builderOptimizationLevel="${TRT_OPTIMIZATION_LEVEL:-0}" \
  --timingCacheFile="$engine_dir/timing.cache"
mv "$engine_dir/yolov8n-face-fp16.engine.tmp" "$engine_dir/yolov8n-face-fp16.engine"
# ROS setup files may reference unset variables.
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
colcon build --packages-select humanoid_vision --symlink-install \
  --cmake-args -DENABLE_TENSORRT=ON -DBUILD_TESTING=ON
