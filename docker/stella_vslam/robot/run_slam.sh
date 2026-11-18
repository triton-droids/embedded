#!/bin/bash
# Runs inside the stella-vslam container (mounted at /robot). Picks mapping or localization from slam.env.
set -euo pipefail

STAMP=$(date +%Y%m%d_%H%M%S)
RUN_DIR=/maps/run_${STAMP}
mkdir -p "$RUN_DIR"

ARGS=(-v /opt/vocab/orb_vocab.fbow -c "$CONFIG" --viewer "$VIEWER" --eval-log-dir "$RUN_DIR")

case "$MODE" in
  mapping)
    ARGS+=(--map-db-out "/maps/map_${STAMP}.msg")
    ;;
  localization)
    if [ -z "${MAP_IN:-}" ] || [ ! -f "/maps/${MAP_IN}" ]; then
      echo "MODE=localization needs MAP_IN set to a file in the maps folder (got '${MAP_IN:-}')" >&2
      exit 1
    fi
    ARGS+=(--disable-mapping --map-db-in "/maps/${MAP_IN}")
    ;;
  *)
    echo "MODE must be mapping or localization (got '$MODE')" >&2
    exit 1
    ;;
esac

# publish_tf=true needs a TF chain camera -> base_link -> odom, which the robot does not publish yet.
# With it off, poses are still published on /run_slam/camera_pose.
ARGS+=(--ros-args -p publish_tf:="${PUBLISH_TF:-false}")

echo "run_slam ${ARGS[*]}"
# exec the binary directly (not 'ros2 run', which wraps it in a Python process) so the SIGINT from
# 'docker compose stop' reaches run_slam. It then saves the map and
# frame_trajectory.txt / keyframe_trajectory.txt / track_times.txt into $RUN_DIR.
exec /ros2_ws/install/stella_vslam_ros/lib/stella_vslam_ros/run_slam "${ARGS[@]}"
