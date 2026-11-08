#!/usr/bin/env bash
# Download/extract a private ROS Jazzy middleware trial; no apt install or system overwrite.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
trial="$repo/experimental/reference_warp/generated/cyclone_trial"
mkdir -p "$trial"
cd "$trial"
apt-get download ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cyclonedds \
  ros-jazzy-iceoryx-binding-c ros-jazzy-iceoryx-hoofs ros-jazzy-iceoryx-posh
for archive in ./ros-jazzy-*.deb; do
  dpkg-deb -x "$archive" root
done
