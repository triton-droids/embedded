#!/usr/bin/env python3
"""
Passive CAN listener (no transmit). Verifies python-can + socketcan sees traffic.

Run:
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set up can0
  python3 can_listen.py --channel can0 --seconds 5
"""

import argparse
import time
import can

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    bus = can.interface.Bus(interface="socketcan", channel=args.channel, bitrate=args.bitrate)

    print(f"[+] Listening on {args.channel} for {args.seconds:.1f}s (no transmit)")
    print("[+] If motors/adapter are talking, you'll see frames here.")
    t_end = time.time() + args.seconds

    count = 0
    ext = 0
    try:
        while time.time() < t_end:
            msg = bus.recv(timeout=0.5)
            if msg is None:
                continue
            count += 1
            if msg.is_extended_id:
                ext += 1

            arb = msg.arbitration_id
            print(f"{msg.timestamp:.6f}  id=0x{arb:08X}  ext={msg.is_extended_id}  dlc={msg.dlc}  data={msg.data.hex()}")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    print(f"[+] Done. Received {count} frames ({ext} extended).")
    if count == 0:
        print("[!] No traffic seen. That's OK if nothing is transmitting yet.")
        print("    Next: try the active ping script (GET_DEVICE_ID).")

if __name__ == "__main__":
    main()
