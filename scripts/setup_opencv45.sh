#!/usr/bin/env bash
# Extract Ubuntu's OpenCV 4.5 SDK locally, retaining Jetson's system SDK.
set -euo pipefail
export LC_ALL=C
trap 'echo "OpenCV setup failed at line $LINENO. Fix the error above and rerun this script." >&2' ERR
for command in apt-cache apt-get dpkg-deb dpkg-architecture cmake; do
  if ! command -v "$command" >/dev/null; then
    echo "Required command not found: $command" >&2
    exit 1
  fi
done
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_root="$repo_root/.deps/opencv45"
mkdir -p "$sdk_root/debs"
cd "$sdk_root/debs"
version=4.5.4+dfsg-9ubuntu4
packages=(libopencv-dev)
# Capture separately: failures inside process substitutions bypass set -e.
dependencies="$(apt-cache depends "libopencv-dev=$version")"
while read -r package; do
  packages+=("$package")
done < <(awk '/Depends: libopencv-.*-dev$/ {print $2}' <<< "$dependencies")
if ((${#packages[@]} == 1)); then
  echo "Could not resolve OpenCV $version development packages." >&2
  exit 1
fi
# Include runtime files: OpenCVConfig validates all exported module targets.
dependencies="$(apt-cache depends "${packages[@]/%/=$version}")"
mapfile -t runtimes < <(printf '%s\n' "$dependencies" |
  awk '/Depends: libopencv-.*4\.5d$/ {print $2}' | sort -u)
if ((${#runtimes[@]} == 0)); then
  echo "Could not resolve OpenCV $version runtime packages." >&2
  exit 1
fi
packages+=("${runtimes[@]}")
requests=()
archives=()
architecture="$(dpkg --print-architecture)"
for package in "${packages[@]}"; do
  archive="${package}_${version}_${architecture}.deb"
  archives+=("$archive")
  # Reuse only valid archives for the requested version and architecture.
  if [[ ! -f "$archive" ]] ||
    [[ "$(dpkg-deb -f "$archive" Package Version Architecture 2>/dev/null)" != \
       "$(printf 'Package: %s\nVersion: %s\nArchitecture: %s' "$package" "$version" "$architecture")" ]]; then
    requests+=("$package:$architecture=$version")
  fi
done
if ((${#requests[@]})); then
  apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 download "${requests[@]}"
fi
# Do not extract stale packages left over from another setup run.
for archive in "${archives[@]}"; do
  dpkg-deb -x "$archive" "$sdk_root"
done
multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
test -f "$sdk_root/usr/include/opencv4/opencv2/core.hpp"
test -f "$sdk_root/usr/lib/$multiarch/libopencv_core.so.4.5.4d"
# Exercise the same package lookup as humanoid_vision. This checks all exported
# library targets, rather than declaring success when only opencv_core exists.
validation_dir="$(mktemp -d)"
trap 'rm -rf -- "$validation_dir"' EXIT
cat > "$validation_dir/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.8)
project(validate_opencv45 LANGUAGES CXX)
find_package(OpenCV 4.5.4 EXACT REQUIRED COMPONENTS core dnn highgui imgcodecs imgproc)
CMAKE
cmake -S "$validation_dir" -B "$validation_dir/build" \
  -DOpenCV_DIR="$sdk_root/usr/lib/$multiarch/cmake/opencv4"
printf 'OpenCV_DIR=%s\n' "$sdk_root/usr/lib/$multiarch/cmake/opencv4"
