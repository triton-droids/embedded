#!/usr/bin/env bash
# Source the existing workspace and replace this shell with the bounded runner.
set -e
collector_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$collector_workspace"
export ROS_DISTRO="${ROS_DISTRO:-humble}"
source scripts/activate_rosenv.sh
exec python -u scripts/collect_fall_data.py "$@"
