#!/usr/bin/env bash
# Run after the private Fast DDS install. Does not modify the system ROS install.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo"
source scripts/activate_rosenv.sh
work="$repo/experimental/reference_warp/generated/dds_strict"
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
export CMAKE_PREFIX_PATH="$work/install-fastdds:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$work/install-fastdds/lib:${LD_LIBRARY_PATH:-}"
colcon --log-base "$work/log-rmw" build \
  --base-paths "$work/rmw_fastrtps-8.4.3" \
  --packages-select rmw_fastrtps_shared_cpp rmw_fastrtps_cpp \
  --executor sequential --merge-install \
  --build-base "$work/build-rmw" --install-base "$work/install-rmw" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  "-Dfastrtps_DIR=$work/install-fastdds/share/fastrtps/cmake"
