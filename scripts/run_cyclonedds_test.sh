#!/usr/bin/env bash
# Local CycloneDDS overlay, confined to this process tree.
set -e
cyclone_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$cyclone_workspace"
export ROS_DISTRO="${ROS_DISTRO:-humble}"
source scripts/activate_rosenv.sh
cyclone_prefix="$cyclone_workspace/.deps/cyclonedds/root/opt/ros/$ROS_DISTRO"
if [[ ! -f "$cyclone_prefix/lib/librmw_cyclonedds_cpp.so" ]]; then
  echo "Local CycloneDDS overlay is missing: $cyclone_prefix" >&2
  exit 2
fi
export AMENT_PREFIX_PATH="$cyclone_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$cyclone_prefix/lib:$cyclone_prefix/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$cyclone_workspace/simulation/humanoid_sim/config/cyclonedds_local.xml"
exec python -u scripts/collect_fall_data.py --rmw rmw_cyclonedds_cpp "$@"
