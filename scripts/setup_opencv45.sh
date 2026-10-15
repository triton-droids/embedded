#!/usr/bin/env bash
# Extract Ubuntu's OpenCV 4.5 SDK locally, retaining Jetson's system SDK.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_root="$repo_root/.deps/opencv45"
mkdir -p "$sdk_root/debs"
cd "$sdk_root/debs"
version=4.5.4+dfsg-9ubuntu4
packages=(libopencv-dev)
while read -r package; do
  packages+=("$package")
done < <(apt-cache depends "libopencv-dev=$version" | awk '/Depends: libopencv-.*-dev$/ {print $2}')
# Include runtime files: OpenCVConfig validates all exported module targets.
mapfile -t runtimes < <(apt-cache depends "${packages[@]/%/=$version}" |
  awk '/Depends: libopencv-.*4\.5d$/ {print $2}' | sort -u)
packages+=("${runtimes[@]}")
requests=()
for package in "${packages[@]}"; do
  requests+=("$package=$version")
done
apt-get download "${requests[@]}"
for archive in ./*.deb; do
  dpkg-deb -x "$archive" "$sdk_root"
done
multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
printf 'OpenCV_DIR=%s\n' "$sdk_root/usr/lib/$multiarch/cmake/opencv4"
