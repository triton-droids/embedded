#!/usr/bin/env bash
# Read-only inventory of the AlohaMini Pi's software environment.
# Run ON THE PI:   bash pi_inventory.sh
# Output: ./pi_inventory.txt  (copy to your Mac with scp, then share it)
# It changes nothing and does not dump environment variables, SSH keys,
# or Tailscale auth state. Git remote URLs are scrubbed of embedded credentials.

OUT="pi_inventory.txt"
: > "$OUT"
sec() { printf '\n===== %s =====\n' "$1" >> "$OUT"; }
run() { sec "$*"; { eval "$@"; } >> "$OUT" 2>&1 || echo "(command failed or not found)" >> "$OUT"; }
# Drop lines that look like secrets (tokens, keys, passwords) from free-form output.
scrub() { grep -viE 'token|secret|passw|api[_-]?key|auth[_-]?key|HF_|WANDB_'; }
run_scrubbed() { sec "$*"; { eval "$@"; } 2>&1 | scrub >> "$OUT"; }

run 'cat /etc/os-release'
run 'uname -a'
run 'dpkg --print-architecture'
run 'cat /proc/device-tree/model; echo'
run 'free -h'
run 'df -h /'

# Hardware visible to the OS
run 'lsusb'
run 'v4l2-ctl --list-devices'
run 'ls -l /dev/video* /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/ 2>/dev/null'
run 'groups'

# Python / conda
run 'which python python3 conda; python3 --version'
run 'conda env list'
run 'conda env export -n lerobot --no-builds'
run 'conda run -n lerobot python --version'
run 'conda run -n lerobot pip freeze'

# LeRobot checkouts (search common locations)
sec 'lerobot checkouts'
for d in "$HOME"/lerobot* "$HOME"/*/lerobot* "$HOME"/AlohaMini* "$HOME"/*/AlohaMini*; do
  [ -d "$d/.git" ] || continue
  echo "--- $d" >> "$OUT"
  git -C "$d" remote -v 2>&1 | sed -E 's#(https?://)[^/@]+@#\1<redacted>@#' >> "$OUT"
  git -C "$d" rev-parse HEAD >> "$OUT" 2>&1
  git -C "$d" branch --show-current >> "$OUT" 2>&1
  git -C "$d" status --short | head -20 >> "$OUT" 2>&1
  git -C "$d" diff --stat | tail -20 >> "$OUT" 2>&1
done

# ROS2
run 'ls /opt/ros 2>/dev/null'
run 'dpkg -l | grep -i "ros-" | head -40'

# What starts on its own
run 'systemctl list-unit-files --state=enabled --no-pager'
run 'systemctl list-units --type=service --state=running --no-pager'
run_scrubbed 'crontab -l'
run_scrubbed 'tail -20 "$HOME/.bashrc"'

# Manually installed apt packages (short form)
run 'apt-mark showmanual'

echo "Wrote $OUT ($(wc -l < "$OUT") lines). Review it before sharing."
