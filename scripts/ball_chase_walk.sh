#!/usr/bin/env bash
# Camera ball chase with the model-based walking controller (no RL), headless.
# The robot walks to the football and stops ~0.5 m in front of it.
#   scripts/ball_chase_walk.sh                       # headless
#   scripts/ball_chase_walk.sh remote_viewer:=true   # + read-only 3D view on another computer
# View it from your own computer (only rendering runs there; see
# simulation/humanoid_sim/README.md "Render on another computer"):
#   ssh -N -L 8765:127.0.0.1:8765 USER@THIS_HOST
#   python remote_viewer.py --host 127.0.0.1 --port 8765
# Other arguments go to ball_chase.launch.py unchanged.
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir" || exit 1
[[ -f install/setup.bash ]] || { echo "Build first (see humanoid_control/walking_controller/README.md)" >&2; exit 1; }
# shellcheck source=/dev/null
source scripts/activate_rosenv.sh || exit 1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=1
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
  config=cyclonedds_sim.xml
  [[ "${ROS_DISTRO:-}" == humble ]] && config=cyclonedds_sim_humble.xml
  export CYCLONEDDS_URI="file://$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/$config"
fi
exec ros2 launch humanoid_ball_chase ball_chase.launch.py controller:=model_walk auto_start:=true \
  use_viewer:=false ros_domain_id:="$ROS_DOMAIN_ID" "$@"
