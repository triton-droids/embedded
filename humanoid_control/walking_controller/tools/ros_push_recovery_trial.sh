#!/usr/bin/env bash
# ROS fall-recovery trial: full_body_arms + model_walk + walk_fall_recovery on simulated time. Waits until the
# robot has taken 4 steps, pushes it over with /sim/push, records a bag and logs /walk/status every ~10 s.
#
#   tools/ros_push_recovery_trial.sh OUT_DIR [RTF=0.03] [FX=150] [FY=0] [PUSH_S=0.3] [SIM_LIMIT_S=60]
#
# OUT_DIR/progress.log (state, recovery mode, steps), events.log (mode changes), launch.log, bag/.
# The get-up MPC plans ~0.5-0.8 s of CPU per 0.04 s: keep the real-time factor near 0.03.
OUT=${1:?usage: ros_push_recovery_trial.sh OUT_DIR [RTF] [FX] [FY] [PUSH_S] [SIM_LIMIT_S]}
RTF=${2:-0.03}; FX=${3:-150}; FY=${4:-0}; PS=${5:-0.3}; LIMIT=${6:-60}
OUT=$(realpath -m "$OUT"); mkdir -p "$OUT"
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$repo" || exit 1
# shellcheck source=/dev/null
source scripts/activate_rosenv.sh >/dev/null 2>&1; source install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-57} ROS_LOCALHOST_ONLY=1
ros2 launch humanoid_sim sim.launch.py robot:=full_body_arms controller:=model_walk auto_start:=true \
  use_viewer:=false sim_time:=true real_time_factor:="$RTF" walk_fall_recovery:=true \
  fall_detection_enabled:=false walk_max_tilt_rad:=0.6 step_length_m:=0.04 > "$OUT/launch.log" 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT "$bag_pid" 2>/dev/null; sleep 3
  kill -INT "$launch_pid" 2>/dev/null; sleep 8; kill -9 "$launch_pid" 2>/dev/null
  grep "Fall recovery\|Push\|fault" "$OUT/launch.log" > "$OUT/events.log"
}
trap cleanup EXIT
sleep 25
ros2 bag record -o "$OUT/bag" /sim/ground_truth/odom /joint_states /walk/status /sim/status > "$OUT/bag.log" 2>&1 &
bag_pid=$!
field() { timeout 90 ros2 topic echo --once --field data "$1" std_msgs/msg/String 2>/dev/null | head -1; }
pushed=0
while true; do
  read -r st rec steps nrec <<< "$(field /walk/status | python3 -c '
import sys, json
try: s = json.loads(sys.stdin.read())
except Exception: print("? ? ? ?"); sys.exit()
print(s.get("state"), s.get("recovery"), s.get("steps"), s.get("recoveries"))')"
  t=$(field /sim/status | python3 -c 'import sys, json
try: print(round(json.loads(sys.stdin.read()).get("sim_time_s", 0), 2))
except Exception: print(0)')
  echo "$(date +%T) sim $t s | state $st | recovery $rec | steps $steps | recoveries $nrec" >> "$OUT/progress.log"
  if [[ $pushed == 0 && "$st" == active && "$steps" =~ ^[0-9]+$ && $steps -ge 4 ]]; then
    ros2 topic pub --once /sim/push std_msgs/msg/Float64MultiArray "{data: [$FX, $FY, 0.0, $PS]}" >/dev/null 2>&1
    echo "$(date +%T) PUSH [$FX $FY] N for $PS s at sim $t s" >> "$OUT/progress.log"; pushed=1
  fi
  if grep -q "Recovery planner failed\|Traceback\|Stale inputs\|estop" "$OUT/launch.log"; then
    echo "ERROR in launch.log, stopping" >> "$OUT/progress.log"; break
  fi
  if (( $(echo "$t > $LIMIT" | bc) )); then echo "sim limit reached" >> "$OUT/progress.log"; break; fi
  sleep 10
done
