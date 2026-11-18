#!/bin/bash
# Build the arm64 images on the Mac and write everything the Pi needs to a folder (e.g. a USB stick).
#     ./make_usb_bundle.sh /Volumes/STICK                      # images + scripts + config
#     ./make_usb_bundle.sh /Volumes/STICK --with-docker-debs   # also Docker .debs, for a Pi with no internet
#
# Docker debs default to Ubuntu 24.04 (noble), the best guess for the Pi's OS. If pi_inventory shows
# Raspberry Pi OS instead, set DOCKER_DEB_DIST=debian/bookworm.
# Format the stick exFAT, not FAT32: FAT32 cannot hold files over 4 GB. Sizes are printed at the end.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
DEST="${1:?usage: $0 <dest-folder> [--with-docker-debs]}/alohamini-slam"
WITH_DEBS=0
[ "${2:-}" = "--with-docker-debs" ] && WITH_DEBS=1
DOCKER_DEB_DIST="${DOCKER_DEB_DIST:-ubuntu/noble}"
JOBS="${JOBS:-6}"

mkdir -p "$DEST/images" "$DEST/config"

echo "==> Building images (linux/arm64)"
docker build --platform linux/arm64 --build-arg JOBS="$JOBS" -t stella-vslam:arm64 "$HERE"
docker build --platform linux/arm64 -f "$HERE/Dockerfile.viewer" -t stella-viewer:arm64 "$HERE"

echo "==> Saving images"
docker save stella-vslam:arm64 | gzip > "$DEST/images/stella-vslam-arm64.tar.gz"
docker save stella-viewer:arm64 | gzip > "$DEST/images/stella-viewer-arm64.tar.gz"

echo "==> Copying scripts and config"
cp "$HERE"/robot/{install.sh,run_slam.sh,compose.yaml,slam.env,alohamini-slam.service} "$DEST/"
cp "$HERE"/config/*.yaml "$DEST/config/"
cp "$HERE/../pi-replica/pi_inventory.sh" "$DEST/"

if [ "$WITH_DEBS" -eq 1 ]; then
  echo "==> Downloading Docker .debs for $DOCKER_DEB_DIST arm64"
  base="https://download.docker.com/linux/${DOCKER_DEB_DIST%/*}/dists/${DOCKER_DEB_DIST#*/}/pool/stable/arm64/"
  index=$(curl -fsSL "$base")
  mkdir -p "$DEST/docker-debs"
  for pkg in containerd.io docker-ce docker-ce-cli docker-buildx-plugin docker-compose-plugin; do
    f=$(printf '%s\n' "$index" | grep -oE "href=\"${pkg}_[^\"]+_arm64\.deb\"" | sed -E 's/href="(.*)"/\1/' | sort -V | tail -1)
    [ -n "$f" ] || { echo "could not find $pkg in $base"; exit 1; }
    curl -fsSL "$base$f" -o "$DEST/docker-debs/$f"
    echo "    $f"
  done
fi

du -sh "$DEST"/images/* "$DEST"
echo "Done. On the Pi: cd <stick>/alohamini-slam && sudo bash install.sh"
