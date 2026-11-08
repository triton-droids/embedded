#!/usr/bin/env bash
# Shadow-mode fall predictor test on the simulation (read-only predictor).
#   [DDS=cyclonedds|fastrtps] [COLLECT_ARGS=--wbc] evaluation/fall_predictor/run_shadow_test.sh OUT_DIR EPISODES SEED [DOMAIN]
# Starts shadow_node.py on the collector's domain, runs collect_fall_data.py
# (recovery profile; default local CycloneDDS via scripts/run_cyclonedds_test.sh,
# DDS=fastrtps: ROS default Fast DDS, which the WBC path needs on this Jetson), then
# stops the shadow node and writes OUT_DIR/analysis/summary.json.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
out="$(realpath -m "$1")"; episodes="$2"; seed="$3"; domain="${4:-90}"
train="${TRAIN_DIR:-$repo/data/fall_training/dual_v2_20261107/training}"
best="$train/best.json"
model="$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['model_path'])" "$best")"
py="${SHADOW_PYTHON:-/tmp/safety-nano-venv/bin/python}"
dds="${DDS:-cyclonedds}"
[[ $dds == cyclonedds || $dds == fastrtps ]] || { echo "DDS must be cyclonedds or fastrtps" >&2; exit 2; }
mkdir -p "$out"

# Same middleware environment as run_cyclonedds_test.sh, without rosenv's site-packages.
cyclone="$repo/.deps/cyclonedds/root/opt/ros/humble"
(
  export ROS_DISTRO=humble
  set +u; source /opt/ros/humble/setup.bash; set -u
  if [[ $dds == cyclonedds ]]; then
    export AMENT_PREFIX_PATH="$cyclone:$AMENT_PREFIX_PATH"
    export LD_LIBRARY_PATH="$cyclone/lib:$cyclone/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://$repo/simulation/humanoid_sim/config/cyclonedds_local.xml"
  else
    unset RMW_IMPLEMENTATION CYCLONEDDS_URI
  fi
  export ROS_DOMAIN_ID="$domain" ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=1
  exec "$py" -u "$repo/evaluation/fall_predictor/shadow_node.py" \
    --model "$model" --best "$best" --out "$out/shadow.jsonl"
) > "$out/shadow_node.log" 2>&1 &
shadow=$!
trap 'kill -INT $shadow 2>/dev/null || true' EXIT
sleep 3
kill -0 $shadow || { echo "shadow node failed; see $out/shadow_node.log" >&2; exit 1; }

collect=(--profile recovery --hours 3 --episode-seconds 20 --max-episodes "$episodes"
         --seed "$seed" --domain "$domain" --output "$out/collection" ${COLLECT_ARGS:-})
if [[ $dds == cyclonedds ]]; then
  "$repo/scripts/run_cyclonedds_test.sh" "${collect[@]}" > "$out/collection.log" 2>&1 || echo "collector exited $?"
else
  (cd "$repo"; export ROS_DISTRO=humble; unset RMW_IMPLEMENTATION CYCLONEDDS_URI
   set +u; source scripts/activate_rosenv.sh; set -u
   exec python -u scripts/collect_fall_data.py "${collect[@]}") > "$out/collection.log" 2>&1 || echo "collector exited $?"
fi

kill -INT $shadow; wait $shadow || true; trap - EXIT
"$py" "$repo/evaluation/fall_predictor/shadow_analyze.py" --collection "$out/collection" \
  --shadow "$out/shadow.jsonl" --best "$best" --model "$model" --out "$out/analysis"
