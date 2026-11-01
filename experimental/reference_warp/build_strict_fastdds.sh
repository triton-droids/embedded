#!/usr/bin/env bash
# Sources/dependency extraction live under generated/dds_strict; no system installation.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo"
source scripts/activate_rosenv.sh
work="$repo/experimental/reference_warp/generated/dds_strict"
export CCACHE_DIR="$work/ccache"
cmake -G Ninja -S "$work/Fast-DDS-2.14.5" -B "$work/build-fastdds" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
  "-DCMAKE_INSTALL_PREFIX=$work/install-fastdds" \
  -DSTRICT_REALTIME=ON -DSECURITY=ON -DSQLITE3_SUPPORT=ON \
  -DFASTDDS_STATISTICS=ON -DCOMPILE_EXAMPLES=OFF -DCOMPILE_TOOLS=OFF \
  -DBUILD_TESTING=OFF -DINSTALL_EXAMPLES=OFF -DTHIRDPARTY=OFF \
  "-DAsio_INCLUDE_DIR=$work/asio-deb/usr/include"
cmake --build "$work/build-fastdds" -j1
cmake --install "$work/build-fastdds"
