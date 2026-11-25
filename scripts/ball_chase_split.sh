#!/usr/bin/env bash
# Ball chase split across two hosts over a direct wired link: the simulator
# (mujoco_sim + split IMU/stereo publishers, which share state_shm) on one host,
# the robot stack (control, safety, vision, localization, planning) on the other.
#   simulator host:  scripts/ball_chase_split.sh sim   LOCAL_IP PEER_IP [launch args]
#   robot host:      scripts/ball_chase_split.sh robot LOCAL_IP PEER_IP [launch args]
# Pass the same launch arguments (scene, enable_vio, real_time_factor, ...) on
# both hosts. Example (WSL 192.168.50.1, Jetson 192.168.50.2):
#   WSL:    scripts/ball_chase_split.sh sim   192.168.50.1 192.168.50.2 scene:=hard enable_vio:=true
#   Jetson: scripts/ball_chase_split.sh robot 192.168.50.2 192.168.50.1 scene:=hard enable_vio:=true
# The simulator host renders the stereo pair without shadows (stereo_shadows:=false):
# under WSL2 MuJoCo renders with Mesa llvmpipe on the CPU, ~4x faster without
# them; pass stereo_shadows:=true to override.
# Or keep the 200 Hz control loop next to the simulator and run only perception
# and planning on the other host, so the control loop never crosses the link:
#   WSL:    scripts/ball_chase_split.sh sim_control 192.168.50.1 192.168.50.2 scene:=hard enable_vio:=true
#   Jetson: scripts/ball_chase_split.sh perception  192.168.50.2 192.168.50.1 scene:=hard enable_vio:=true
# CONTROL_CPUS=0-3 pins this host's control processes to those CPUs and its
# simulator, renderers and other nodes to SIM_CPUS (default: all remaining CPUs;
# set it, e.g. SIM_CPUS=4-15, to leave CPUs free for other work on the host).
# LP_NUM_THREADS caps Mesa llvmpipe's render threads per context.
# Both hosts need the same ROS distro and ROS_DOMAIN_ID; raise net.core.rmem_max
# (e.g. 33554432) on both.
role="${1:-}"; local_ip="${2:-}"; peer_ip="${3:-}"
if [[ ! "$role" =~ ^(sim|robot|sim_control|perception)$ ]] || [[ -z "$local_ip" || -z "$peer_ip" ]]; then
  echo "Usage: $0 sim|robot|sim_control|perception LOCAL_IP PEER_IP [launch args]" >&2
  exit 2
fi
shift 3
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir" || exit 1
[[ -f install/setup.bash ]] || { echo "Build first (see control/walking_controller/README.md)" >&2; exit 1; }
# shellcheck source=/dev/null
source scripts/activate_rosenv.sh || exit 1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=0
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DDS_LOCAL_IP="$local_ip" DDS_PEER_IP="$peer_ip"
export CYCLONEDDS_URI="file://$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/cyclonedds_split.xml"
if [[ -n "${CONTROL_CPUS:-}" ]]; then
  other_cpus="${SIM_CPUS:-}"
  [[ -n "$other_cpus" ]] || other_cpus="$(python3 -c '
import os, sys
used = set()
for part in sys.argv[1].split(","):
    a, _, b = part.partition("-")
    used.update(range(int(a), int(b or a) + 1))
print(",".join(str(c) for c in sorted(os.sched_getaffinity(0) - used)))' "$CONTROL_CPUS")"
  # Re-applied for the first minute: nodes start at different times, and threads
  # created later inherit the affinity of the thread that creates them.
  (
    for _ in $(seq 30); do
      for p in $(pgrep -u "$USER" -f 'cpp_control_node|walking_controller|safety_node|safety_diagnostics_node|sim_autostart'); do
        taskset -a -cp "$CONTROL_CPUS" "$p" >/dev/null 2>&1
      done
      for p in $(pgrep -u "$USER" -f 'mujoco_sim|sim_stereo_publisher|sim_imu_publisher|leg_odometry|track_node|motor_web_node'); do
        taskset -a -cp "$other_cpus" "$p" >/dev/null 2>&1
      done
      sleep 2
    done
  ) &
fi
exec ros2 launch humanoid_ball_chase ball_chase.launch.py controller:=model_walk auto_start:=true \
  use_viewer:=false ros_domain_id:="$ROS_DOMAIN_ID" localhost_only:=0 role:="$role" stereo_shadows:=false "$@"
