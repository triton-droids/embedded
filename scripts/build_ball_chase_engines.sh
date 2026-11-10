#!/usr/bin/env bash
# Build TensorRT FP16 engines for the ball-chase YOLO models on this Jetson.
#   scripts/build_ball_chase_engines.sh                  # every assets/*.onnx
#   scripts/build_ball_chase_engines.sh nubot-yolo       # one model
# Engines are device/TensorRT-version specific and are not tracked by Git. Each
# <model>-fp16.engine is written next to its ONNX with <model>-fp16.engine.json
# recording the ONNX SHA-256 and TensorRT version; the detector refuses an
# engine whose record does not match its ONNX.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
assets="$repo_dir/simulation/humanoid_ball_chase/assets"
trtexec_bin="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
[[ -x "$trtexec_bin" ]] || { echo "trtexec not found ($trtexec_bin); set TRTEXEC" >&2; exit 1; }
trt_version="$(/usr/bin/python3 -c 'import tensorrt; print(tensorrt.__version__)')"
models=("$@")
if [[ ${#models[@]} -eq 0 ]]; then
  for f in "$assets"/*.onnx; do models+=("$(basename "$f" .onnx)"); done
fi
for m in "${models[@]}"; do
  onnx="$assets/$m.onnx"; engine="$assets/$m-fp16.engine"
  [[ -f "$onnx" ]] || { echo "Missing $onnx" >&2; exit 1; }
  echo "Building $engine (TensorRT $trt_version) ..."
  "$trtexec_bin" --onnx="$onnx" --saveEngine="$engine.tmp" --fp16 --skipInference \
    --memPoolSize=workspace:512 --timingCacheFile="$assets/.trt-timing.cache" > "$engine.log" 2>&1 \
    || { tail -20 "$engine.log" >&2; exit 1; }
  mv "$engine.tmp" "$engine"
  printf '{"onnx": "%s", "onnx_sha256": "%s", "tensorrt": "%s", "precision": "fp16"}\n' \
    "$m.onnx" "$(sha256sum "$onnx" | cut -d' ' -f1)" "$trt_version" > "$engine.json"
  echo "  ok: $(du -h "$engine" | cut -f1)"
done
