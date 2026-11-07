#!/usr/bin/env bash
# Start the full simulation N times and count startup estops and IMU stalls.
#
#   simulation/humanoid_sim/tools/timing/startup_trials.sh [N]          # experimental launch
#   LAUNCH=sim simulation/humanoid_sim/tools/timing/startup_trials.sh 10 # original, Fast DDS
#
# Each trial: robot held on the gantry, camera + depth + policy (override with
# ARGS="..."), 14 s run, an independent recorder logging every IMU stamp.
# Results: $OUT_ROOT/startup_<launch>_<host>_<time>/summary.md
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
timing_setup || exit 1
N="${1:-10}"
DOMAIN="${DOMAIN:-82}"
ARGS="${ARGS:-enable_vision:=true start_detector:=false enable_depth:=true}"
RUN_S="${RUN_S:-14}"
out="$(new_out_dir startup)"
write_env "$out/environment.txt"
echo "launch args: use_viewer:=false auto_start:=false $ARGS" >> "$out/environment.txt"
stop_domain "$DOMAIN"

for i in $(seq "$N"); do
  ROS_DOMAIN_ID="$DOMAIN" python3 "$TOOLS_DIR/imu_recorder.py" "$out/trial_$i.imu" $((RUN_S + 2)) &
  rec=$!
  sleep 1
  # shellcheck disable=SC2086
  start_launch "$DOMAIN" "$out/trial_$i.log" use_viewer:=false auto_start:=false $ARGS
  sleep "$RUN_S"
  wait "$rec"
  stop_domain "$DOMAIN"
  sleep 2
  if grep -qE 'SAFETY ESTOP LATCHED|IMU timeout' "$out/trial_$i.log"; then
    echo "trial $i/$N: ESTOP $(grep -oE '(imu_freshness|fall_detection|IMU timeout)[^]]{0,40}' "$out/trial_$i.log" | head -1)"
  else
    echo "trial $i/$N: ok"
  fi
done
python3 "$TOOLS_DIR/startup_report.py" "$out" | tee "$out/summary.md"
echo; echo "Results: $out"
