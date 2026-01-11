import argparse
import threading
import time
from collections import deque
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from imu_read import iter_imu_samples

# 如果你用的是“带积分器版本”的 imu_stream.py，下面 import 会成功
try:
    from imu_read import RK4DeadReckoner
    HAS_INTEGRATOR = True
except Exception:
    RK4DeadReckoner = None
    HAS_INTEGRATOR = False


def build_signal_dict(sample: dict) -> Dict[str, Optional[float]]:
    """
    Convert one sample dict into a flat dict of scalar signals.
    Keys here are what you pass via --signals.
    """
    out: Dict[str, Optional[float]] = {}

    # time
    out["t_ms"] = float(sample["t_ms"]) if sample.get("t_ms") is not None else None
    out["t_s"] = float(sample["t_s"]) if sample.get("t_s") is not None else None
    out["host_time_s"] = float(sample["host_time_s"]) if sample.get("host_time_s") is not None else None

    # accel/gyro (default available)
    if sample.get("acc_g") is not None:
        ax, ay, az = sample["acc_g"]
        out["ax_g"], out["ay_g"], out["az_g"] = ax, ay, az
    if sample.get("gyro_dps") is not None:
        gx, gy, gz = sample["gyro_dps"]
        out["gx_dps"], out["gy_dps"], out["gz_dps"] = gx, gy, gz

    # accel-only angles from firmware (need include_all=True or keys=None)
    out["roll_acc"] = sample.get("roll_deg")
    out["pitch_acc"] = sample.get("pitch_deg")

    # temperature (optional)
    out["temp_C"] = sample.get("temp_C")

    # norms (available in full output)
    out["acc_norm_g"] = sample.get("acc_norm_g")
    out["gyro_norm_dps"] = sample.get("gyro_norm_dps")

    # integrated angles (need integrator enabled)
    rpy = sample.get("rpy_deg")
    if rpy is not None:
        r, p, y = rpy
        out["roll_int"], out["pitch_int"], out["yaw_int"] = r, p, y
    else:
        out["roll_int"] = out["pitch_int"] = out["yaw_int"] = None

    # integrated position/velocity (if you want them)
    pos = sample.get("lin_pos_m")
    if pos is not None:
        out["px_m"], out["py_m"], out["pz_m"] = pos
    else:
        out["px_m"] = out["py_m"] = out["pz_m"] = None

    vel = sample.get("lin_vel_ms")
    if vel is not None:
        out["vx_ms"], out["vy_ms"], out["vz_ms"] = vel
    else:
        out["vx_ms"] = out["vy_ms"] = out["vz_ms"] = None

    out["stationary"] = 1.0 if sample.get("stationary") else 0.0 if sample.get("stationary") is not None else None
    out["dt_s_int"] = sample.get("dt_s")

    return out


def needs_integration(signals: List[str]) -> bool:
    for s in signals:
        if s.endswith("_int") or s in ("roll_int", "pitch_int", "yaw_int", "px_m", "py_m", "pz_m", "vx_ms", "vy_ms", "vz_ms"):
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description="Real-time plot selected IMU signals from imu_stream.iter_imu_samples()")
    parser.add_argument("--port", default="/dev/ttyUSB0", help='Serial port (Windows: "COM5")')
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=50.0, help="Output/plot rate (Hz)")
    parser.add_argument("--window", type=float, default=10.0, help="Rolling window length (seconds)")
    parser.add_argument("--signals", nargs="+", default=["roll_acc", "pitch_acc"],
                        help="Signals to plot (e.g. roll_acc pitch_acc roll_int pitch_int yaw_int ax_g ay_g az_g gx_dps ...)")
    parser.add_argument("--source", choices=["serial", "i2c"], default="serial")
    parser.add_argument("--i2c_bus", type=int, default=1)
    parser.add_argument("--i2c_addr", type=lambda x: int(x, 0), default=0x68, help="I2C addr (e.g. 0x68)")
    args = parser.parse_args()

    sigs = args.signals
    use_int = needs_integration(sigs)

    integrator = None
    if use_int:
        if not HAS_INTEGRATOR:
            raise RuntimeError("You requested *_int signals but RK4DeadReckoner is not available in imu_stream.py.")
        integrator = RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665))

    # ring buffers
    maxlen = max(10, int(args.window * args.rate * 1.2))
    t_buf = deque(maxlen=maxlen)
    y_bufs = {s: deque(maxlen=maxlen) for s in sigs}

    lock = threading.Lock()
    stop_flag = {"stop": False}

    def reader_thread():
        # We want all fields available for extraction (roll/pitch/temp/norms/etc),
        # so set include_all=True and then the plotting code picks what it needs.
        gen = iter_imu_samples(
            source=args.source,
            port=args.port,
            baud=args.baud,
            rate_hz=args.rate,
            include_all=True,
            integrator=integrator,
            i2c_bus=args.i2c_bus,
            i2c_addr=args.i2c_addr,
        )

        t0 = None
        for sample in gen:
            if stop_flag["stop"]:
                break

            sd = build_signal_dict(sample)

            # choose time axis: prefer board time, fallback to host time
            t = sd.get("t_s")
            if t is None:
                ht = sd.get("host_time_s")
                if ht is None:
                    continue
                t = ht
                if t0 is None:
                    t0 = t
                t = t - t0  # make it relative
            else:
                # board time already relative-ish; still normalize to start at 0
                if t0 is None:
                    t0 = t
                t = t - t0

            with lock:
                t_buf.append(t)
                for name in sigs:
                    y_bufs[name].append(sd.get(name))

    th = threading.Thread(target=reader_thread, daemon=True)
    th.start()

    # matplotlib setup (single plot, multiple lines)
    fig, ax = plt.subplots()
    lines = {}
    for name in sigs:
        (ln,) = ax.plot([], [], label=name)
        lines[name] = ln

    ax.set_xlabel("time (s)")
    ax.set_ylabel("value")
    ax.legend(loc="upper right")
    ax.grid(True)

    def update(_):
        with lock:
            if len(t_buf) < 2:
                return list(lines.values())

            xs = list(t_buf)

            # update each line
            ymin = None
            ymax = None
            for name in sigs:
                ys_raw = list(y_bufs[name])
                # drop Nones for plotting: keep alignment by mapping None->nan
                ys = [float("nan") if v is None else v for v in ys_raw]
                lines[name].set_data(xs, ys)

                # compute y limits robustly
                for v in ys:
                    if v == v:  # not nan
                        ymin = v if ymin is None else min(ymin, v)
                        ymax = v if ymax is None else max(ymax, v)

            ax.set_xlim(xs[0], xs[-1])
            if ymin is not None and ymax is not None and ymin != ymax:
                pad = 0.05 * (ymax - ymin)
                ax.set_ylim(ymin - pad, ymax + pad)

        return list(lines.values())

    interval_ms = max(10, int(1000.0 / max(1.0, args.rate)))
    ani = FuncAnimation(fig, update, interval=interval_ms, blit=False)

    try:
        plt.show()
    finally:
        stop_flag["stop"] = True


if __name__ == "__main__":
    main()
