#!/usr/bin/env bash
# Run the mode-manager scenario on the full-body robot and render the video.
#   control/humanoid_mode_manager/tools/run_scenario.sh OUT_DIR [RTF] [RECOVERY_THREADS] [POLICY]
# From the repository root with rosenv built. The get-up MPC is not real time: RTF 0.03 with
# 8 threads (~20-30 min wall); a 24-thread host managed 0.08 with 16 threads. POLICY: a get-up student
# policy npz (walking_controller tools/getup_bc.py) instead of the MPC; that runs at RTF 1.
# Environment: BALANCE_POLICY (straight-phase policy npz), WALK_POLICY (RL walking policy npz),
# WALK_POLICY_PERIOD (its gait-clock period, default 0.7), SIM_ASSIST=true (TEST ONLY base assist while balancing;
# SIM_ASSIST_PHASES=rise,straight also while getting up; SIM_ASSIST_WALK_S=3 keeps it 3 s into walking).
set -uo pipefail
OUT=${1:?OUT_DIR}; RTF=${2:-0.03}; THREADS=${3:-8}; POLICY=${4:-}
TOOLS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$OUT"
set +u; source scripts/activate_rosenv.sh || exit 1; set -u
ros2 launch humanoid_sim sim.launch.py robot:=full_body_arms controller:=model_walk auto_start:=true \
  sim_time:=true real_time_factor:="$RTF" walk_fall_recovery:=true walk_recovery_threads:="$THREADS" \
  walk_recovery_policy:="$POLICY" walk_balance_policy:="${BALANCE_POLICY:-}" \
  walk_rl_policy:="${WALK_POLICY:-}" walk_rl_policy_period:="${WALK_POLICY_PERIOD:-0.7}" \
  walk_sim_assist:="${SIM_ASSIST:-false}" walk_sim_assist_phases:="${SIM_ASSIST_PHASES:-straight}" \
  walk_sim_assist_walk_s:="${SIM_ASSIST_WALK_S:-0.0}" \
  fall_detection_enabled:=false walk_max_tilt_rad:=0.6 walk_cmd_vel_topic:=/ball_chase/cmd_vel \
  share_state:=true use_viewer:=false mode_manager:=true > "$OUT/launch.log" 2>&1 &
LAUNCH=$!
trap 'kill -INT $LAUNCH 2>/dev/null; sleep 8; kill -TERM $LAUNCH 2>/dev/null' EXIT
sleep 5
python3 "$TOOLS/record_scenario.py" "$OUT/scenario.npz" | tee "$OUT/drive.log"
MUJOCO_GL="${MUJOCO_GL:-egl}" python3 "$TOOLS/render_scenario.py" "$OUT/scenario.npz" "$OUT/scenario.mp4"
