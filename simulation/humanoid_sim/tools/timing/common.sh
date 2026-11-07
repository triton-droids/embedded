# Shared helpers for the simulation timing tools. Source it, do not run it.
#
#   LAUNCH=experimental (default) runs sim_experimental.launch.py on CycloneDDS;
#   LAUNCH=sim runs the original sim.launch.py on Fast DDS. Helper processes
#   (recorders, probes) use the same DDS as the launch.
#   OUT_ROOT selects where results go (default local_test/sim_timing, ignored by Git).
#   Each tool runs on its own ROS domain (81-84, override with DOMAIN=). Keep
#   domains <= 101: on Linux larger IDs map to UDP ports inside the ephemeral
#   range (32768-60999) and can collide with unrelated connections.

TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$TOOLS_DIR/../../../.." && pwd)"
OUT_ROOT="${OUT_ROOT:-$REPO_DIR/local_test/sim_timing}"
LAUNCH="${LAUNCH:-experimental}"
export ROS_LOCALHOST_ONLY=1

timing_setup() {
  cd "$REPO_DIR" || return 1
  if [[ ! -f install/setup.bash ]]; then
    echo "Build first: source scripts/activate_rosenv.sh && colcon build --symlink-install --packages-up-to humanoid_sim" >&2
    return 1
  fi
  # shellcheck source=/dev/null
  source scripts/activate_rosenv.sh || return 1
  case "$LAUNCH" in
    experimental)
      LAUNCH_FILE=sim_experimental.launch.py
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      # Helpers must use the launch's CycloneDDS config too (loopback only).
      export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/cyclonedds_sim.xml}"
      if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
        echo "CycloneDDS missing: sudo apt install ros-${ROS_DISTRO:-jazzy}-rmw-cyclonedds-cpp" >&2
        return 1
      fi
      if [[ ! -f "$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/launch/$LAUNCH_FILE" ]]; then
        echo "$LAUNCH_FILE not installed: colcon build --symlink-install --packages-select humanoid_sim" >&2
        return 1
      fi ;;
    sim)
      LAUNCH_FILE=sim.launch.py
      unset RMW_IMPLEMENTATION CYCLONEDDS_URI ;;
    *) echo "LAUNCH must be experimental or sim, not '$LAUNCH'" >&2; return 1 ;;
  esac
  export LAUNCH_FILE
}

# new_out_dir <tool name>: creates and prints a fresh result directory.
new_out_dir() {
  local dir="$OUT_ROOT/$1_${LAUNCH}_$(hostname)_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$dir" && echo "$dir"
}

# write_env <file>: host, CPU, renderer, DDS and Git revision of a measurement.
write_env() {
  {
    echo "date: $(date -Is)"
    echo "launch: $LAUNCH_FILE  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default: Fast DDS>}"
    grep -qi microsoft /proc/version && echo "platform: WSL" || echo "platform: native"
    echo "kernel: $(uname -r)"
    LC_ALL=C lscpu | grep -E '^Model name|^CPU\(s\)|^Thread|^Core'
    echo "net.core.rmem_max: $(cat /proc/sys/net/core/rmem_max 2>/dev/null)"
    echo "ROS_DISTRO=${ROS_DISTRO:-?} DISPLAY=${DISPLAY:-<unset>} MUJOCO_GL=${MUJOCO_GL:-<unset>}"
    echo "git: $(git -C "$REPO_DIR" rev-parse --abbrev-ref HEAD) $(git -C "$REPO_DIR" rev-parse --short HEAD)"
    git -C "$REPO_DIR" status --short --untracked-files=no | head -20
    if command -v glxinfo >/dev/null; then glxinfo -B 2>&1 | grep -E 'OpenGL renderer'
    else echo "glxinfo: not installed (apt install mesa-utils)"; fi
  } > "$1" 2>&1
}

# start_launch <domain> <log file> [launch arguments...]: starts the selected
# launch in the background on its own domain; results are read from the log.
start_launch() {
  local domain="$1" log="$2"; shift 2
  ROS_DOMAIN_ID="$domain" setsid ros2 launch humanoid_sim "$LAUNCH_FILE" \
    ros_domain_id:="$domain" localhost_only:=1 web_port:=$((18000 + domain)) "$@" > "$log" 2>&1 &
}

# stop_domain <domain>: stops every process of this user whose environment has
# ROS_DOMAIN_ID=<domain> (launch, nodes, helpers). Process IDs from setsid are
# not reliable, so the domain is the handle. SIGINT first, SIGKILL after 8 s.
stop_domain() {
  local domain="$1" sig pid pids
  for sig in INT KILL; do
    pids=()
    for pid in $(pgrep -u "$(id -u)" -f 'ros2|mujoco_sim|_node|python3'); do
      [[ "$pid" == "$$" || "$pid" == "$BASHPID" ]] && continue
      tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$domain" && pids+=("$pid")
    done
    (( ${#pids[@]} )) || return 0
    kill -"$sig" "${pids[@]}" 2>/dev/null
    [[ $sig == INT ]] && for _ in $(seq 16); do
      sleep 0.5
      local alive=0; for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
      (( alive )) || return 0
    done
  done
}
