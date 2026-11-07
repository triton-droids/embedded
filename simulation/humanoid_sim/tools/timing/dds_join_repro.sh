#!/usr/bin/env bash
# Minimal reproduction of the Fast DDS publish stall.
#
#   LAUNCH=sim simulation/humanoid_sim/tools/timing/dds_join_repro.sh [N]  # Fast DDS: stalls
#   simulation/humanoid_sim/tools/timing/dds_join_repro.sh [N]             # CycloneDDS: none
#
# Only the simulator (+ safety supervisor) runs: minimal:=true, no camera, no
# policy. 2.5 s after start a node joins that only creates the two simulator
# service clients cpp_control_node uses (service_clients_node.py). The IMU
# recorder shows whether the simulator then stops publishing for ~330 ms,
# reported relative to the join time (E). ARGS="..." adds launch arguments.
# On an i7-1165G7 with Fast DDS 2.14 this happened in 12 of 17 trials, about
# 2 s after the join (E+1.9 to E+2.3 s); the rate varies, so run 6 or more.
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
timing_setup || exit 1
N="${1:-6}"
DOMAIN="${DOMAIN:-83}"
out="$(new_out_dir join)"
write_env "$out/environment.txt"
stop_domain "$DOMAIN"
hits=0

for i in $(seq "$N"); do
  ROS_DOMAIN_ID="$DOMAIN" python3 "$TOOLS_DIR/imu_recorder.py" "$out/trial_$i.imu" 16 &
  rec=$!
  sleep 1
  # shellcheck disable=SC2086
  start_launch "$DOMAIN" "$out/trial_$i.log" use_viewer:=false minimal:=true \
    enable_vision:=false enable_policy:=false ${ARGS:-}
  sleep 2.5
  join="$(date +%s.%N)"
  ROS_DOMAIN_ID="$DOMAIN" python3 "$TOOLS_DIR/service_clients_node.py" > "$out/trial_$i.join.log" 2>&1 &
  wait "$rec"
  stop_domain "$DOMAIN"
  sleep 2
  line="$(python3 - "$out/trial_$i.imu" "$join" <<'PY'
import sys
stamps = sorted(float(l.split()[2]) for l in open(sys.argv[1]))
join = float(sys.argv[2])
gaps = [f'E{a - join:+.2f}s {1e3 * (b - a):.0f} ms' for a, b in zip(stamps, stamps[1:])
        if b - a > 0.1 and a > join]
print(', '.join(gaps) if gaps else '-')
PY
)"
  [[ "$line" != - ]] && hits=$((hits + 1))
  echo "| $i | $line |" | tee -a "$out/rows.md"
done
{ cat "$out/environment.txt" | sed 's/^/    /'; echo
  echo "| trial | simulator IMU stalls > 100 ms after the clients joined (E) |"; echo "|---|---|"
  cat "$out/rows.md"; echo; echo "**$hits/$N trials stalled.**"
} > "$out/summary.md"
echo "$hits/$N trials stalled. Results: $out"
