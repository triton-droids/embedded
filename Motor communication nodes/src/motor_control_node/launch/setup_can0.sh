sudo tee /usr/local/sbin/setup_can0.sh > /dev/null <<'EOF'
#!/usr/bin/env bash
set -euo pipefail


modprobe can
modprobe can_raw
modprobe can_dev


if ip link show can0 &>/dev/null; then
  ip link set can0 down 2>/dev/null || true
else
  echo "ERROR: can0 doesn't exist" >&2
  exit 1
fi

ip link set can0 type can bitrate 1000000
ip link set can0 up
ip link set can0 txqueuelen 100

echo "[setup_can0] can0 is up @ 1,000,000 bps, txqueuelen=100"
EOF

sudo chmod +x /usr/local/sbin/setup_can0.sh
