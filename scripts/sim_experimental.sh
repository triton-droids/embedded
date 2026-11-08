#!/usr/bin/env bash
# One-click start of the experimental simulation (sim_experimental.launch.py):
# CycloneDDS with a larger receive buffer and a 0.5 ms Python GIL switch
# interval in mujoco_sim. Arguments are passed to the launch unchanged:
#   scripts/sim_experimental.sh
#   scripts/sim_experimental.sh enable_vision:=true start_detector:=false
#   scripts/sim_experimental.sh auto_start:=true use_viewer:=false
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir" || exit 1

if [[ ! -f install/setup.bash ]]; then
  echo "No install/ yet. Build first, e.g.:" >&2
  echo "  source scripts/activate_rosenv.sh && colcon build --symlink-install --packages-up-to humanoid_sim" >&2
  exit 1
fi
# shellcheck source=/dev/null
source scripts/activate_rosenv.sh || exit 1

if ! ros2 pkg prefix humanoid_sim >/dev/null 2>&1 || [[ ! -f \
    "$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/launch/sim_experimental.launch.py" ]]; then
  echo "sim_experimental.launch.py is not installed. Rebuild humanoid_sim:" >&2
  echo "  colcon build --symlink-install --packages-select humanoid_sim" >&2
  exit 1
fi

# The launch selects the DDS of its nodes; use the same one for the launch
# process itself so the graph stays single-vendor.
rmw=rmw_cyclonedds_cpp
for arg in "$@"; do
  [[ "$arg" == rmw_implementation:=* ]] && rmw="${arg#rmw_implementation:=}"
done
if [[ "$rmw" == rmw_cyclonedds_cpp ]] && ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
  echo "CycloneDDS is not installed:" >&2
  echo "  sudo apt install ros-${ROS_DISTRO:-jazzy}-rmw-cyclonedds-cpp" >&2
  exit 1
fi
export RMW_IMPLEMENTATION="$rmw"
# The launch process itself is a DDS participant too; give it the same
# loopback-only CycloneDDS config as the nodes (unless the caller set one).
if [[ "$rmw" == rmw_cyclonedds_cpp && -z "${CYCLONEDDS_URI:-}" ]]; then
  # Humble's rmw already pins lo for ROS_LOCALHOST_ONLY=1; see cyclonedds_sim_humble.xml.
  config=cyclonedds_sim.xml
  [[ "${ROS_DISTRO:-}" == humble ]] && config=cyclonedds_sim_humble.xml
  export CYCLONEDDS_URI="file://$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/$config"
fi

# The camera images need a large socket receive buffer (see config/cyclonedds_sim.xml).
want=33554432
have="$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)"
if [[ "$rmw" == rmw_cyclonedds_cpp ]] && (( have < want )); then
  echo "WARNING: net.core.rmem_max=$have is below $want; camera frames may drop." >&2
  echo "  This boot:  sudo sysctl -w net.core.rmem_max=$want" >&2
  echo "  Permanent:  echo 'net.core.rmem_max=$want' | sudo tee /etc/sysctl.d/60-cyclonedds.conf" >&2
fi

exec ros2 launch humanoid_sim sim_experimental.launch.py "$@"
