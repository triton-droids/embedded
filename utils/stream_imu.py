#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from utils.imu_read import RK4DeadReckoner, iter_imu_samples


ACC_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _v_norm3(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


class LowPassVec3:
    def __init__(self, alpha: float):
        self.alpha = _clamp(float(alpha), 0.0, 1.0)
        self.state: tuple[float, float, float] | None = None

    def update(self, vec: tuple[float, float, float]) -> tuple[float, float, float]:
        if self.state is None:
            self.state = vec
        else:
            a = self.alpha
            self.state = tuple((1.0 - a) * s + a * v for s, v in zip(self.state, vec))
        return self.state


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Stream IMU samples over serial/I2C or poll an IMU responder over CAN.")
    p.add_argument("--source", choices=("serial", "i2c", "can"), default="serial")
    p.add_argument("--port", default="/dev/ttyACM3", help='Serial port, e.g. /dev/ttyACM1 or COM13')
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--rate", type=float, default=50.0, help="Requested output rate in Hz")
    p.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout in seconds")
    p.add_argument("--i2c-bus", type=int, default=1)
    p.add_argument("--i2c-addr", type=lambda x: int(x, 0), default=0x68)
    p.add_argument("--can-interface", choices=("slcan", "socketcan"), default="socketcan")
    p.add_argument("--can-channel", default=None,
                   help="CAN channel. Defaults to --port for slcan, or can0 for socketcan")
    p.add_argument("--can-bitrate", type=int, default=500000)
    p.add_argument("--req-id", type=lambda x: int(x, 0), default=0x100)
    p.add_argument("--rsp1-id", type=lambda x: int(x, 0), default=0x101)
    p.add_argument("--rsp2-id", type=lambda x: int(x, 0), default=0x102)
    p.add_argument("--accel-alpha", type=float, default=0.2, help="Low-pass alpha for accel smoothing")
    p.add_argument("--gyro-alpha", type=float, default=0.2, help="Low-pass alpha for gyro smoothing")
    p.add_argument("--raw-only", action="store_true", help="Print raw accel/gyro fields only")
    p.add_argument("--include-all", action="store_true", help="Include all parsed fields in the output")
    p.add_argument("--samples", type=int, default=0, help="Stop after N samples; 0 means stream forever")
    p.add_argument("--json", action="store_true", help="Emit one JSON object per line")
    return p.parse_args()


def format_sample(sample: dict[str, Any]) -> str:
    compact = {
        "seq": sample.get("seq"),
        "t_ms": sample.get("t_ms"),
        "acc_g": sample.get("acc_g"),
        "acc_g_filt": sample.get("acc_g_filt"),
        "gyro_dps": sample.get("gyro_dps"),
        "gyro_dps_filt": sample.get("gyro_dps_filt"),
        "roll_deg": sample.get("roll_deg"),
        "pitch_deg": sample.get("pitch_deg"),
        "rpy_deg": sample.get("rpy_deg"),
        "up_body": sample.get("up_body"),
        "stationary": sample.get("stationary"),
        "warning": sample.get("warning"),
    }
    return " ".join(f"{k}={v}" for k, v in compact.items() if v is not None)


def be_i16(buf: bytes, offset: int) -> int:
    return int.from_bytes(buf[offset:offset + 2], byteorder="big", signed=True)


def decode_can_sample(rsp1: bytes, rsp2: bytes) -> dict[str, Any]:
    ax = be_i16(rsp1, 0)
    ay = be_i16(rsp1, 2)
    az = be_i16(rsp1, 4)
    gx = be_i16(rsp1, 6)
    gy = be_i16(rsp2, 0)
    gz = be_i16(rsp2, 2)
    seq = rsp2[4]
    return {
        "seq": seq,
        "ax_raw": ax,
        "ay_raw": ay,
        "az_raw": az,
        "gx_raw": gx,
        "gy_raw": gy,
        "gz_raw": gz,
        "acc_g": (ax / ACC_LSB_PER_G, ay / ACC_LSB_PER_G, az / ACC_LSB_PER_G),
        "gyro_dps": (gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS),
    }


def decode_can_sample_single(rsp1: bytes) -> dict[str, Any]:
    ax = be_i16(rsp1, 0)
    ay = be_i16(rsp1, 2)
    az = be_i16(rsp1, 4)
    gx = be_i16(rsp1, 6)
    return {
        "ax_raw": ax,
        "ay_raw": ay,
        "az_raw": az,
        "gx_raw": gx,
        "gy_raw": None,
        "gz_raw": None,
        "acc_g": (ax / ACC_LSB_PER_G, ay / ACC_LSB_PER_G, az / ACC_LSB_PER_G),
        "gyro_dps": (gx / GYRO_LSB_PER_DPS, None, None),
        "warning": "received only rsp1 frame; gy/gz unavailable",
    }


def add_clean_readings(
    sample: dict[str, Any],
    *,
    accel_filter: LowPassVec3,
    gyro_filter: LowPassVec3,
    reckoner: RK4DeadReckoner | None,
    start_time_s: float,
    sample_idx: int,
) -> dict[str, Any]:
    acc_g_raw = sample.get("acc_g")
    gyro_dps_raw = sample.get("gyro_dps")
    if acc_g_raw is None or gyro_dps_raw is None:
        return sample

    acc_g = tuple(float(v) for v in acc_g_raw)
    gyro_missing = any(v is None for v in gyro_dps_raw)
    gyro_dps_full = tuple(0.0 if v is None else float(v) for v in gyro_dps_raw)

    acc_g_filt = accel_filter.update(acc_g)
    gyro_dps_filt = gyro_filter.update(gyro_dps_full)

    sample["t_s"] = time.time() - start_time_s
    sample["t_ms"] = sample["t_s"] * 1e3
    sample["sample_idx"] = sample_idx
    sample["acc_g_filt"] = acc_g_filt
    sample["gyro_dps_filt"] = tuple(None if gyro_missing and i > 0 else gyro_dps_filt[i] for i in range(3))
    sample["acc_norm_g"] = _v_norm3(acc_g_filt)
    sample["gyro_norm_dps"] = _v_norm3(gyro_dps_filt)

    ax, ay, az = acc_g_filt
    denom = math.sqrt(ay * ay + az * az)
    sample["roll_deg"] = math.degrees(math.atan2(ay, az if abs(az) > 1e-9 else 1e-9))
    sample["pitch_deg"] = math.degrees(math.atan2(-ax, denom if denom > 1e-9 else 1e-9))
    sample["acc_ms2"] = tuple(v * 9.80665 for v in acc_g_filt)
    sample["gyro_rads"] = tuple(math.radians(v) for v in gyro_dps_filt)

    if reckoner is not None and not gyro_missing:
        sample.update(
            reckoner.update(
                {
                    "t_s": sample["t_s"],
                    "acc_ms2": sample["acc_ms2"],
                    "gyro_rads": sample["gyro_rads"],
                }
            )
        )
    return sample


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


def recv_pair(bus: Any, id_a: int, id_b: int, timeout_s: float) -> tuple[Any | None, Any | None]:
    deadline = time.time() + timeout_s
    found: dict[int, Any] = {}
    while time.time() < deadline and (id_a not in found or id_b not in found):
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            continue
        if msg.is_extended_id:
            continue
        if msg.arbitration_id == id_a or msg.arbitration_id == id_b:
            found[msg.arbitration_id] = msg
            # If rsp1 arrives but rsp2 does not exist on this firmware variant,
            # don't burn the full timeout every cycle waiting for it.
            if id_a in found and id_b not in found:
                deadline = min(deadline, time.time() + min(0.005, timeout_s))
    return found.get(id_a), found.get(id_b)


def iter_can_samples(args: argparse.Namespace):
    try:
        import can
    except ImportError as e:
        raise RuntimeError("Missing dependency 'python-can'. Install it with: python3 -m pip install python-can") from e

    channel = args.can_channel or (args.port if args.can_interface == "slcan" else "can0")
    bus = can.interface.Bus(interface=args.can_interface, channel=channel, bitrate=args.can_bitrate)
    period = 1.0 / args.rate if args.rate > 0 else 0.0
    accel_filter = LowPassVec3(args.accel_alpha)
    gyro_filter = LowPassVec3(args.gyro_alpha)
    reckoner = None if args.raw_only else RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665))
    start_time_s = time.time()
    sample_idx = 0
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

            msg1, msg2 = recv_pair(bus, args.rsp1_id, args.rsp2_id, args.timeout)
            if msg1 is None:
                missing = []
                missing.append(f"0x{args.rsp1_id:X}")
                if msg2 is None:
                    missing.append(f"0x{args.rsp2_id:X}")
                raise TimeoutError(f"timeout waiting for response frame(s): {', '.join(missing)}")
            if msg2 is None:
                sample = decode_can_sample_single(bytes(msg1.data))
            else:
                sample = decode_can_sample(bytes(msg1.data), bytes(msg2.data))
            if args.raw_only:
                sample = {k: v for k, v in sample.items() if k.endswith("_raw") or k == "seq"}
            else:
                sample = add_clean_readings(
                    sample,
                    accel_filter=accel_filter,
                    gyro_filter=gyro_filter,
                    reckoner=reckoner,
                    start_time_s=start_time_s,
                    sample_idx=sample_idx,
                )
            yield sample
            sample_idx += 1

            if period > 0:
                sleep_s = period - (time.time() - t0)
                if sleep_s > 0:
                    time.sleep(sleep_s)
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


def main() -> int:
    args = parse_args()

    count = 0
    try:
        if args.source == "can":
            iterator = iter_can_samples(args)
        else:
            integrator = None if args.raw_only else RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665))
            kwargs = {
                "source": args.source,
                "baud": args.baud,
                "timeout": args.timeout,
                "rate_hz": args.rate,
                "include_all": args.include_all or (not args.raw_only),
                "integrator": integrator,
            }
            if args.source == "serial":
                kwargs["port"] = args.port
            else:
                kwargs["i2c_bus"] = args.i2c_bus
                kwargs["i2c_addr"] = args.i2c_addr
            iterator = iter_imu_samples(**kwargs)

        for sample in iterator:
            if args.json:
                print(json.dumps(sample, default=str))
            else:
                print(format_sample(sample))
            count += 1
            if args.samples > 0 and count >= args.samples:
                break
    except KeyboardInterrupt:
        return 0
    except Exception as e:
        print(str(e), file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
