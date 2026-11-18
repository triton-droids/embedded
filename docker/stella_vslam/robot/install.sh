#!/bin/bash
# Install the SLAM stack on the AlohaMini Pi from the USB bundle. Run AT THE PI'S CONSOLE from the stick:
#     cd /media/$USER/<stick>/alohamini-slam && sudo bash install.sh            # install, don't autostart
#     cd /media/$USER/<stick>/alohamini-slam && sudo bash install.sh --enable   # also start SLAM at every boot
#
# What it does, in order:
#   1. runs pi_inventory.sh (read-only) and writes pi_inventory.txt back to the stick
#   2. installs Docker if missing (from docker-debs/ on the stick, else from the internet)
#   3. loads the two images (stella-vslam:arm64, stella-viewer:arm64)
#   4. copies compose/config/scripts to /opt/alohamini-slam (keeps an existing slam.env and maps/)
#   5. picks a camera device and installs alohamini-slam.service
set -euo pipefail

BUNDLE="$(cd "$(dirname "$0")" && pwd)"
DEST=/opt/alohamini-slam
ENABLE=0
[ "${1:-}" = "--enable" ] && ENABLE=1

say()  { printf '\n==> %s\n' "$*"; }
warn() { printf '\n!!! %s\n' "$*"; }

[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }
[ "$(uname -m)" = "aarch64" ] || { echo "expected an arm64 Pi, got $(uname -m)"; exit 1; }
REAL_USER="${SUDO_USER:-root}"

# 1. Inventory (as the normal user, so it sees their conda/lerobot setup)
if [ -f "$BUNDLE/pi_inventory.sh" ]; then
  say "Running pi_inventory.sh -> $BUNDLE/pi_inventory.txt"
  # Run in a temp dir the user owns (the stick may be mounted root-only), then copy the result back.
  tmp=$(mktemp -d); chown "$REAL_USER" "$tmp"
  ( cd "$tmp" && sudo -u "$REAL_USER" -H bash "$BUNDLE/pi_inventory.sh" ) || warn "inventory failed (continuing)"
  cp "$tmp/pi_inventory.txt" "$BUNDLE/" 2>/dev/null || warn "could not copy pi_inventory.txt to the stick; it is in $tmp"
fi

# 2. Docker
if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
  if ls "$BUNDLE"/docker-debs/*.deb >/dev/null 2>&1; then
    say "Installing Docker from docker-debs/ on the stick"
    dpkg -i "$BUNDLE"/docker-debs/*.deb || { apt-get -f install -y || true; }
  elif curl -fsS --max-time 5 https://get.docker.com -o /tmp/get-docker.sh; then
    say "Installing Docker from the internet"
    sh /tmp/get-docker.sh
  else
    echo "Docker is not installed, there is no docker-debs/ folder and no internet."
    echo "Re-make the bundle with --with-docker-debs (see README)."
    exit 1
  fi
  systemctl enable --now docker
fi
docker compose version

# 3. Images
for img in stella-vslam stella-viewer; do
  say "Loading $img (this takes a few minutes for stella-vslam)"
  gunzip -c "$BUNDLE/images/$img-arm64.tar.gz" | docker load
done

# 4. Files
say "Installing files to $DEST"
mkdir -p "$DEST/maps" "$DEST/config"
install -m 644 "$BUNDLE/compose.yaml" "$DEST/compose.yaml"
install -m 755 "$BUNDLE/run_slam.sh" "$DEST/run_slam.sh"
cp -r "$BUNDLE/config/." "$DEST/config/"
[ -f "$DEST/slam.env" ] || install -m 644 "$BUNDLE/slam.env" "$DEST/slam.env"

# 5. Camera device
cur=$(grep '^CAMERA_DEVICE=' "$DEST/slam.env" | cut -d= -f2-)
if [ ! -e "$cur" ]; then
  alt=$(ls /dev/v4l/by-id/*-video-index0 2>/dev/null | head -1 || true)
  if [ -n "$alt" ]; then
    warn "$cur does not exist; using $alt"
    sed -i "s#^CAMERA_DEVICE=.*#CAMERA_DEVICE=$alt#" "$DEST/slam.env"
  else
    warn "No camera found ($cur missing, nothing in /dev/v4l/by-id). Set CAMERA_DEVICE in $DEST/slam.env."
  fi
fi
say "Camera: $(grep '^CAMERA_DEVICE=' "$DEST/slam.env")"
command -v v4l2-ctl >/dev/null && v4l2-ctl --list-devices 2>/dev/null || true

if grep -q PLACEHOLDER "$DEST/config/elp_fisheye.yaml"; then
  warn "config/elp_fisheye.yaml still has PLACEHOLDER intrinsics. Expect poor or no tracking until it is calibrated."
fi

install -m 644 "$BUNDLE/alohamini-slam.service" /etc/systemd/system/alohamini-slam.service
systemctl daemon-reload
if [ "$ENABLE" -eq 1 ]; then
  systemctl enable alohamini-slam
  say "Enabled: SLAM starts at every boot."
else
  say "Installed, not enabled. Start now: sudo systemctl start alohamini-slam ; at boot: sudo systemctl enable alohamini-slam"
fi

cat <<EOF

Done.
  - Stop the LeRobot host before starting SLAM; both cannot open the camera.
  - Viewer: http://$(hostname -I | awk '{print $1}'):3001 from a laptop on the same network.
  - Stop cleanly (saves the map): sudo systemctl stop alohamini-slam   -> $DEST/maps/
  - Logs: journalctl -u alohamini-slam -f
  - Copy pi_inventory.txt from the stick back to the Mac (for issue #41).
  - While you are at the console: consider 'sudo tailscale up' on your account so next time you can SSH.
EOF
