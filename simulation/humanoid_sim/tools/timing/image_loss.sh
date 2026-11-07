#!/usr/bin/env bash
# Measure camera frame loss between the simulator and a detector-like subscriber.
#
#   simulation/humanoid_sim/tools/timing/image_loss.sh [RUNS]          # experimental launch
#   LAUNCH=sim simulation/humanoid_sim/tools/timing/image_loss.sh 3     # original, Fast DDS
#
# Camera + depth on, robot held; after a 12 s warm-up image_probe.py listens for
# 20 s (PROBE_S). Results: $OUT_ROOT/image_<launch>_<host>_<time>/summary.md
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
timing_setup || exit 1
RUNS="${1:-3}"
DOMAIN="${DOMAIN:-84}"
PROBE_S="${PROBE_S:-20}"
out="$(new_out_dir image)"
write_env "$out/environment.txt"
stop_domain "$DOMAIN"

for i in $(seq "$RUNS"); do
  start_launch "$DOMAIN" "$out/run_$i.log" use_viewer:=false enable_vision:=true \
    start_detector:=false enable_depth:=true
  sleep 12
  { echo "### run $i"; echo
    ROS_DOMAIN_ID="$DOMAIN" python3 "$TOOLS_DIR/image_probe.py" "$PROBE_S"
    grep -hE 'SAFETY ESTOP LATCHED|IMU timeout' "$out/run_$i.log" | head -1; echo
  } | tee -a "$out/summary.md"
  stop_domain "$DOMAIN"
  sleep 2
done
echo "Results: $out"
