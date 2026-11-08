#!/usr/bin/env bash
# Scoped middleware workaround; does not change the system ROS installation.
set -eo pipefail
chase_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$chase_repo/scripts/activate_rosenv.sh"
source "$chase_repo/experimental/reference_warp/activate_cyclone_trial.sh"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
echo '[chase] RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; gantry held unless start_chase:=true' >&2
exec ros2 launch "$chase_repo/experimental/reference_warp/chase.launch.py" "$@"
