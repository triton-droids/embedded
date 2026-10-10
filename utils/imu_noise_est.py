#!/usr/bin/env python3
"""
imu_noise_est.py

Collect IMU samples for a fixed duration while stationary and report:
  - roll / pitch / yaw noise (mean, std)
  - up_body (projected gravity) noise per axis + norm
  - raw accel / gyro noise
  - sample-to-sample latency statistics
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from utils.imu_read import RK4DeadReckoner, iter_imu_samples

ACC_LSB_PER_G    = 16384.0
GYRO_LSB_PER_DPS = 131.0


# ---------- helpers (mirrors stream_imu_can.py) ----------

def _be_i16(buf: bytes, offset: int) -> int:
    return int.from_bytes(buf[offset:offset + 2], byteorder="big", signed=True)


def _patch_slcan():
    """
    Fix python-can 4.x slcan parser for CANable2 firmware.
    The 0x102 response arrives as a 't' frame with a hex DLC char ('F'),
    which standard SLCAN parses as decimal-only and raises ValueError.
    Patch: intercept _read() to rewrite 't' frames with non-decimal DLC
    to 'd' (FD standard) so python-can's existing FD path handles them.
    """
    try:
        import can.interfaces.slcan as _sl
        _orig_read = _sl.slcanBus._read

        def _fixed_read(self, timeout):
            s = _orig_read(self, timeout)
            if s and len(s) >= 5 and s[0] == "t":
                try:
                    int(s[4])  # valid decimal DLC — leave as-is
                except ValueError:
                    # non-decimal DLC: rewrite as FD standard frame 'd'
                    s = "d" + s[1:]
            return s

        _sl.slcanBus._read = _fixed_read
    except Exception:
        pass


def _recv_pair(bus: Any, id_a: int, id_b: int, timeout_s: float):
    """Collect rsp1 and (if present) rsp2 within timeout."""
    deadline = time.time() + timeout_s
    found: dict[int, Any] = {}
    while time.time() < deadline and id_a not in found:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            continue
        if msg.is_extended_id:
            continue
        if msg.arbitration_id in (id_a, id_b):
            found[msg.arbitration_id] = msg
    # short extra wait for rsp2 once rsp1 is in hand
    if id_a in found and id_b not in found:
        extra = min(0.01, timeout_s)
        deadline2 = time.time() + extra
        while time.time() < deadline2:
            msg = bus.recv(timeout=max(0.0, deadline2 - time.time()))
            if msg and not msg.is_extended_id and msg.arbitration_id == id_b:
                found[id_b] = msg
                break
    return found.get(id_a), found.get(id_b)


def _decode_can(rsp1: bytes, rsp2: bytes) -> dict[str, Any]:
    ax = _be_i16(rsp1, 0); ay = _be_i16(rsp1, 2); az = _be_i16(rsp1, 4)
    gx = _be_i16(rsp1, 6); gy = _be_i16(rsp2, 0); gz = _be_i16(rsp2, 2)
    seq = rsp2[4]
    return {
        "seq": seq,
        "acc_g":    (ax / ACC_LSB_PER_G,    ay / ACC_LSB_PER_G,    az / ACC_LSB_PER_G),
        "gyro_dps": (gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS),
    }


def _decode_can_single(rsp1: bytes) -> dict[str, Any]:
    ax = _be_i16(rsp1, 0); ay = _be_i16(rsp1, 2); az = _be_i16(rsp1, 4)
    gx = _be_i16(rsp1, 6)
    return {
        "acc_g":    (ax / ACC_LSB_PER_G,    ay / ACC_LSB_PER_G,    az / ACC_LSB_PER_G),
        "gyro_dps": (gx / GYRO_LSB_PER_DPS, None, None),
    }


def _iter_can(args: argparse.Namespace, reckoner: RK4DeadReckoner):
    try:
        import can
    except ImportError:
        raise RuntimeError("Install python-can: pip install python-can")

    _patch_slcan()

    channel = args.can_channel
    bus = can.interface.Bus(
        interface=args.can_interface,
        channel=channel,
        bitrate=args.can_bitrate,
    )
    period  = 1.0 / args.rate if args.rate > 0 else 0.0
    start_s = time.time()
    try:
        while True:
            t0  = time.time()
            req = can.Message(
                arbitration_id=args.req_id,
                is_extended_id=False,
                is_remote_frame=False,
                data=b"",
            )
            bus.send(req, timeout=args.timeout)

            msg1, msg2 = _recv_pair(bus, args.rsp1_id, args.rsp2_id, args.timeout)
            if msg1 is None:
                raise TimeoutError(f"timeout waiting for 0x{args.rsp1_id:X}")

            if msg2 is not None:
                sample = _decode_can(bytes(msg1.data), bytes(msg2.data))
            else:
                sample = _decode_can_single(bytes(msg1.data))

            t_s = time.time() - start_s
            sample["t_s"] = t_s

            gd = sample["gyro_dps"]
            has_full_gyro = all(v is not None for v in gd)

            if has_full_gyro:
                # full 6-axis: use RK4 for roll/pitch/yaw + up_body
                sample["acc_ms2"]   = tuple(v * 9.80665 for v in sample["acc_g"])
                sample["gyro_rads"] = tuple(math.radians(v) for v in gd)
                sample.update(reckoner.update(sample))
            else:
                # accel-only roll/pitch (same formula as stream_imu.py)
                ax, ay, az = sample["acc_g"]
                denom = math.sqrt(ay * ay + az * az)
                roll_deg  = math.degrees(math.atan2(ay, az if abs(az) > 1e-9 else 1e-9))
                pitch_deg = math.degrees(math.atan2(-ax, denom if denom > 1e-9 else 1e-9))
                r, p = math.radians(roll_deg), math.radians(pitch_deg)
                up_body = (-math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r))
                sample["roll_deg"]  = roll_deg
                sample["pitch_deg"] = pitch_deg
                sample["rpy_deg"]   = (roll_deg, pitch_deg, None)
                sample["up_body"]   = up_body

            yield sample

            if period > 0:
                sleep_s = period - (time.time() - t0)
                if sleep_s > 0:
                    time.sleep(sleep_s)
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


# ---------- Welford online stats ----------

class _RS:
    def __init__(self):
        self.n = 0; self._m = 0.0; self._M2 = 0.0
        self.lo = float("inf"); self.hi = float("-inf")

    def push(self, x: float):
        self.n += 1
        d = x - self._m; self._m += d / self.n; self._M2 += d * (x - self._m)
        if x < self.lo: self.lo = x
        if x > self.hi: self.hi = x

    @property
    def mean(self): return self._m
    @property
    def std(self): return math.sqrt(self._M2 / self.n) if self.n > 1 else 0.0


def _fmt(label: str, s: "_RS", unit: str = "") -> str:
    if s.n == 0:
        return f"  {label:34s}  no data"
    suffix = f"  {unit}" if unit else ""
    return (
        f"  {label:34s}  mean={s.mean:+9.5f}  std={s.std:8.5f}"
        f"  [{s.lo:+9.5f}, {s.hi:+9.5f}]{suffix}"
    )


# ---------- args ----------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="IMU noise and latency estimator")
    p.add_argument("--source", choices=("can", "serial", "i2c"), default="can")
    # serial / i2c
    p.add_argument("--port",    default="/dev/ttyACM3")
    p.add_argument("--baud",    type=int, default=115200)
    p.add_argument("--i2c-bus", type=int, default=1)
    p.add_argument("--i2c-addr", type=lambda x: int(x, 0), default=0x68)
    # CAN
    p.add_argument("--can-interface", choices=("slcan", "socketcan"), default="slcan")
    p.add_argument("--can-channel",   default="/dev/ttyACM3")
    p.add_argument("--can-bitrate",   type=int, default=500000)
    p.add_argument("--req-id",  type=lambda x: int(x, 0), default=0x100)
    p.add_argument("--rsp1-id", type=lambda x: int(x, 0), default=0x101)
    p.add_argument("--rsp2-id", type=lambda x: int(x, 0), default=0x102)
    # common
    p.add_argument("--rate",     type=float, default=50.0, help="Poll rate Hz (0=unlimited for serial/i2c)")
    p.add_argument("--duration", type=float, default=10.0, help="Collection time in seconds — keep IMU stationary")
    p.add_argument("--timeout",  type=float, default=0.2)
    return p.parse_args()


# ---------- main ----------

def main() -> int:
    args = parse_args()

    reckoner = RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665))

    if args.source == "can":
        iterator = _iter_can(args, reckoner)
    else:
        rate_hz = args.rate if args.rate > 0 else None
        kw = dict(source=args.source, baud=args.baud, timeout=args.timeout,
                  rate_hz=rate_hz, include_all=True, integrator=reckoner)
        if args.source == "serial":
            kw["port"] = args.port
        else:
            kw["i2c_bus"] = args.i2c_bus
            kw["i2c_addr"] = args.i2c_addr
        iterator = iter_imu_samples(**kw)

    roll_s  = _RS(); pitch_s = _RS(); yaw_s   = _RS()
    up0_s   = _RS(); up1_s   = _RS(); up2_s   = _RS(); up_norm_s = _RS()
    acc0_s  = _RS(); acc1_s  = _RS(); acc2_s  = _RS()
    gyro0_s = _RS(); gyro1_s = _RS(); gyro2_s = _RS()
    hdt_s   = _RS()   # host dt
    bdt_s   = _RS()   # board dt (serial only)

    t_host_prev: Optional[float] = None
    t_board_prev: Optional[float] = None
    n = 0
    deadline = time.time() + args.duration

    rate_str = f"{args.rate:.0f} Hz" if args.rate > 0 else "unlimited"
    print(f"Collecting {args.duration:.0f}s via {args.source} @ {rate_str} — keep IMU stationary ...")

    try:
        for sample in iterator:
            now = time.time()
            if now > deadline:
                break

            rpy = sample.get("rpy_deg")
            if rpy is not None:
                if rpy[0] is not None: roll_s.push(rpy[0])
                if rpy[1] is not None: pitch_s.push(rpy[1])
                if rpy[2] is not None: yaw_s.push(rpy[2])

            ub = sample.get("up_body")
            if ub is not None:
                up0_s.push(ub[0]); up1_s.push(ub[1]); up2_s.push(ub[2])
                up_norm_s.push(math.sqrt(ub[0]**2 + ub[1]**2 + ub[2]**2))

            ag = sample.get("acc_g")
            if ag is not None:
                acc0_s.push(ag[0]); acc1_s.push(ag[1]); acc2_s.push(ag[2])

            gd = sample.get("gyro_dps")
            if gd is not None:
                if gd[0] is not None: gyro0_s.push(gd[0])
                if gd[1] is not None: gyro1_s.push(gd[1])
                if gd[2] is not None: gyro2_s.push(gd[2])

            if t_host_prev is not None:
                hdt_s.push(now - t_host_prev)
            t_host_prev = now

            t_ms = sample.get("t_ms")
            if t_ms is not None:
                tb = t_ms * 1e-3
                if t_board_prev is not None:
                    d = tb - t_board_prev
                    if 0.0 < d < 1.0:
                        bdt_s.push(d)
                t_board_prev = tb

            n += 1

    except KeyboardInterrupt:
        pass

    W = 74
    print(f"\n{'='*W}")
    print(f"  Samples : {n}    source : {args.source}")
    print(f"{'='*W}")

    print("\n--- Latency (host wall-clock between yielded samples) ---")
    if hdt_s.n > 0:
        eff = 1.0 / hdt_s.mean if hdt_s.mean > 0 else 0.0
        print(f"  {'dt mean':<34s}  {hdt_s.mean*1e3:.3f} ms   eff {eff:.1f} Hz  (req {args.rate:.0f} Hz)")
        print(f"  {'dt std (jitter)':<34s}  {hdt_s.std*1e3:.3f} ms")
        print(f"  {'dt min / max':<34s}  [{hdt_s.lo*1e3:.3f},  {hdt_s.hi*1e3:.3f}] ms")

    if bdt_s.n > 0:
        print(f"\n--- Latency (board millis() dt) ---")
        eff_b = 1.0 / bdt_s.mean if bdt_s.mean > 0 else 0.0
        print(f"  {'dt mean':<34s}  {bdt_s.mean*1e3:.3f} ms   eff {eff_b:.1f} Hz")
        print(f"  {'dt std (jitter)':<34s}  {bdt_s.std*1e3:.3f} ms")
        pj = math.sqrt(max(0.0, hdt_s.std**2 - bdt_s.std**2))
        print(f"  {'pipeline jitter (host-board)':<34s}  {pj*1e3:.3f} ms")

    print("\n--- Roll / Pitch / Yaw  [deg]  (RK4 integrator) ---")
    print(_fmt("roll ", roll_s,  "deg"))
    print(_fmt("pitch", pitch_s, "deg"))
    print(_fmt("yaw  ", yaw_s,   "deg"))

    print("\n--- up_body  (projected gravity, body frame) ---")
    print(_fmt("up_body[0]  x", up0_s))
    print(_fmt("up_body[1]  y", up1_s))
    print(_fmt("up_body[2]  z", up2_s))
    bias = up_norm_s.mean - 1.0
    print(_fmt("|up_body| norm", up_norm_s, f"ideal 1.0  bias {bias:+.5f}"))

    print("\n--- Raw accel  [g] ---")
    print(_fmt("acc_g[0]  x", acc0_s, "g"))
    print(_fmt("acc_g[1]  y", acc1_s, "g"))
    print(_fmt("acc_g[2]  z", acc2_s, "g"))
    rss_acc = math.sqrt(acc0_s.std**2 + acc1_s.std**2 + acc2_s.std**2)
    print(f"  {'total acc noise (rss)':<34s}  {rss_acc:.5f} g")

    print("\n--- Raw gyro  [deg/s] ---")
    print(_fmt("gyro_dps[0]  x", gyro0_s, "deg/s"))
    print(_fmt("gyro_dps[1]  y", gyro1_s, "deg/s"))
    print(_fmt("gyro_dps[2]  z", gyro2_s, "deg/s"))
    rss_gyro = math.sqrt(gyro0_s.std**2 + gyro1_s.std**2 + gyro2_s.std**2)
    print(f"  {'total gyro noise (rss)':<34s}  {rss_gyro:.5f} deg/s")

    print(f"\n{'='*W}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
