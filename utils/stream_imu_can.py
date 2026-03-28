#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any


ACC_LSB_PER_G = 16384.0   # MPU6050 accel +/-2g
GYRO_LSB_PER_DPS = 131.0  # MPU6050 gyro +/-250 dps


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Poll an IMU responder over CAN using the 0x100/0x101/0x102 request/reply protocol.")
    p.add_argument("--interface", choices=("socketcan", "slcan"), default="socketcan",
                   help="Use socketcan with a preconfigured can0, or slcan directly on /dev/ttyACM*")
    p.add_argument("--channel", default="can0",
                   help="CAN channel. Examples: can0 for socketcan, /dev/ttyACM1 for slcan")
    p.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate in bits/s")
    p.add_argument("--poll-hz", type=float, default=50.0, help="Request rate")
    p.add_argument("--timeout", type=float, default=0.2, help="Seconds to wait for each response frame")
    p.add_argument("--req-id", type=lambda x: int(x, 0), default=0x100, help="Request CAN ID")
    p.add_argument("--rsp1-id", type=lambda x: int(x, 0), default=0x101, help="Response frame 1 CAN ID")
    p.add_argument("--rsp2-id", type=lambda x: int(x, 0), default=0x102, help="Response frame 2 CAN ID")
    p.add_argument("--samples", type=int, default=0, help="Stop after N samples; 0 means stream forever")
    p.add_argument("--json", action="store_true", help="Emit one JSON object per line")
    p.add_argument("--raw-only", action="store_true", help="Print raw int16 values only")
    return p.parse_args()


def be_i16(buf: bytes, offset: int) -> int:
    return int.from_bytes(buf[offset:offset + 2], byteorder="big", signed=True)


def decode_frames(rsp1: bytes, rsp2: bytes) -> dict[str, Any]:
    ax = be_i16(rsp1, 0)
    ay = be_i16(rsp1, 2)
    az = be_i16(rsp1, 4)
    gx = be_i16(rsp1, 6)
    gy = be_i16(rsp2, 0)
    gz = be_i16(rsp2, 2)
    seq = rsp2[4]

    out: dict[str, Any] = {
        "seq": seq,
        "ax_raw": ax,
        "ay_raw": ay,
        "az_raw": az,
        "gx_raw": gx,
        "gy_raw": gy,
        "gz_raw": gz,
    }
    out["acc_g"] = (ax / ACC_LSB_PER_G, ay / ACC_LSB_PER_G, az / ACC_LSB_PER_G)
    out["gyro_dps"] = (gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS)
    return out


def recv_until(bus: Any, arb_id: int, timeout_s: float) -> Any:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            continue
        if msg.is_extended_id:
            continue
        if msg.arbitration_id == arb_id:
            return msg
    return None


def format_compact(sample: dict[str, Any]) -> str:
    if "acc_g" in sample and "gyro_dps" in sample:
        return (
            f"seq={sample['seq']} "
            f"acc_g={tuple(round(v, 4) for v in sample['acc_g'])} "
            f"gyro_dps={tuple(round(v, 3) for v in sample['gyro_dps'])} "
            f"raw=({sample['ax_raw']},{sample['ay_raw']},{sample['az_raw']},"
            f"{sample['gx_raw']},{sample['gy_raw']},{sample['gz_raw']})"
        )
    return str(sample)


def main() -> int:
    args = parse_args()
    try:
        import can
    except ImportError:
        print("Missing dependency 'python-can'. Install it with: python3 -m pip install python-can", file=sys.stderr)
        return 2

    bus_kwargs: dict[str, Any] = {"interface": args.interface, "channel": args.channel}
    if args.interface == "socketcan":
        bus_kwargs["bitrate"] = args.bitrate
    else:
        bus_kwargs["bitrate"] = args.bitrate

    bus = can.interface.Bus(**bus_kwargs)
    period = 1.0 / args.poll_hz if args.poll_hz > 0 else 0.0
    count = 0

    try:
        while True:
            t0 = time.time()
            req = can.Message(
                arbitration_id=args.req_id,
                is_extended_id=False,
                is_remote_frame=False,
                data=b"",
            )
            bus.send(req, timeout=args.timeout)

            msg1 = recv_until(bus, args.rsp1_id, args.timeout)
            msg2 = recv_until(bus, args.rsp2_id, args.timeout)
            if msg1 is None or msg2 is None:
                missing = []
                if msg1 is None:
                    missing.append(f"0x{args.rsp1_id:X}")
                if msg2 is None:
                    missing.append(f"0x{args.rsp2_id:X}")
                print(f"timeout waiting for response frame(s): {', '.join(missing)}", file=sys.stderr)
            else:
                sample = decode_frames(bytes(msg1.data), bytes(msg2.data))
                if args.raw_only:
                    sample = {k: v for k, v in sample.items() if k.endswith("_raw") or k == "seq"}
                if args.json:
                    print(json.dumps(sample))
                else:
                    print(format_compact(sample))
                count += 1
                if args.samples > 0 and count >= args.samples:
                    break

            if period > 0:
                dt = time.time() - t0
                sleep_s = period - dt
                if sleep_s > 0:
                    time.sleep(sleep_s)
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
