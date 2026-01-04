#!/usr/bin/env python3
"""
RobStride MIT-Mode Motor Tuner (10 motors) with live plotting.

Requires:
  pip install numpy matplotlib
  (and your robstride python package providing robstride_dynamics.RobstrideBus)

Usage examples:
  python3 robstride_tuner.py --iface can0
  python3 robstride_tuner.py --iface can0 --hz 100
  python3 robstride_tuner.py --iface can0 --log_csv tune_log.csv

Keyboard (focus the plot window):
  q / ESC : quit (safe coast attempt)
  SPACE   : pause/resume sending commands
  [ / ]   : previous/next motor
  m       : cycle mode: HOLD -> SINE -> STEP
  r       : re-zero base position to current position (hold around current)
  c       : clear plot buffer

  UP/DOWN : increase/decrease KP
  LEFT/RIGHT: increase/decrease KD

  a / z   : increase/decrease sine amplitude (rad)
  f / v   : increase/decrease sine frequency (Hz)

  s       : apply STEP (in STEP mode): toggles step sign (+/- step_amp)
  e       : set step_amp to current sine amp (quick convenience)

Notes:
- Default motor_id mapping is 1..10 in the order you provided. Edit MOTOR_MAP if needed.
- Clamps KP/KD to per-model maximums from your table.
"""

import argparse
import math
import sys
import time
import threading
from collections import deque
from dataclasses import dataclass

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---- RobStride bus import (as per your docs) ----
try:
    from robstride_dynamics import RobstrideBus
except Exception as e:
    RobstrideBus = None
    _import_error = e


# ----------------------- User motor ordering -----------------------
# IMPORTANT: Update motor_id values if your CAN motor IDs differ.
MOTOR_MAP = [
    ("left_hip1_joint",   1, "RS-04"),
    ("left_hip2_joint",   2, "RS-03"),
    ("left_thigh_joint",  3, "RS-03"),
    ("left_knee_joint",   4, "RS-04"),
    ("left_ankle_joint",  5, "RS-02"),
    ("right_hip1_joint",  6, "RS-04"),
    ("right_hip2_joint",  7, "RS-03"),
    ("right_thigh_joint", 8, "RS-03"),
    ("right_knee_joint",  9, "RS-04"),
    ("right_ankle_joint", 10, "RS-02"),
]

# Per your table
MODEL_LIMITS = {
    "RS-00": {"kp_max": 500.0,  "kd_max": 5.0},
    "RS-01": {"kp_max": 500.0,  "kd_max": 5.0},
    "RS-02": {"kp_max": 500.0,  "kd_max": 5.0},
    "RS-03": {"kp_max": 5000.0, "kd_max": 100.0},
    "RS-04": {"kp_max": 5000.0, "kd_max": 100.0},
    "RS-05": {"kp_max": 500.0,  "kd_max": 5.0},
    "RS-06": {"kp_max": 5000.0, "kd_max": 100.0},
}


@dataclass
class TunerParams:
    kp: float = 30.0
    kd: float = 0.5
    t_ff: float = 0.0

    mode: str = "HOLD"     # HOLD | SINE | STEP
    sine_amp: float = 0.10 # rad
    sine_hz: float = 0.50  # Hz

    step_amp: float = 0.10 # rad
    step_sign: int = 1     # +1 or -1
    paused: bool = False


class RingBuffer:
    def __init__(self, seconds: float, hz: float):
        self.maxlen = int(max(1, seconds * hz))
        self.t = deque(maxlen=self.maxlen)
        self.p = deque(maxlen=self.maxlen)
        self.p_cmd = deque(maxlen=self.maxlen)
        self.v = deque(maxlen=self.maxlen)
        self.tau = deque(maxlen=self.maxlen)

    def append(self, t, p, p_cmd, v, tau):
        self.t.append(t)
        self.p.append(p)
        self.p_cmd.append(p_cmd)
        self.v.append(v)
        self.tau.append(tau)

    def clear(self):
        self.t.clear()
        self.p.clear()
        self.p_cmd.clear()
        self.v.clear()
        self.tau.clear()


class RobStrideMitTuner:
    def __init__(self, iface: str, hz: float, window_s: float, log_csv: str | None):
        self.iface = iface
        self.hz = float(hz)
        self.dt = 1.0 / self.hz
        self.window_s = float(window_s)
        self.log_csv = log_csv

        self.params = TunerParams()
        self.selected_idx = 0

        self.running = False
        self._lock = threading.Lock()

        self.base_pos = 0.0
        self.last_status = None

        self.buf = RingBuffer(seconds=self.window_s, hz=self.hz)

        self.csv_rows = []  # optionally saved at end

        if RobstrideBus is None:
            raise RuntimeError(
                "Failed to import robstride_dynamics.RobstrideBus. "
                f"Import error: {_import_error}"
            )
        self.bus = RobstrideBus(self.iface)

        self._sync_base_pos_on_start = True

    @property
    def selected_motor(self):
        name, mid, model = MOTOR_MAP[self.selected_idx]
        return name, mid, model

    def clamp_gains(self, kp: float, kd: float, model: str):
        lim = MODEL_LIMITS.get(model, {"kp_max": 500.0, "kd_max": 5.0})
        kp = float(np.clip(kp, 0.0, lim["kp_max"]))
        kd = float(np.clip(kd, 0.0, lim["kd_max"]))
        return kp, kd

    def try_enable_all(self):
        ids = [mid for _, mid, _ in MOTOR_MAP]
        if hasattr(self.bus, "enable_motors"):
            self.bus.enable_motors(ids)
        else:
            # Best effort: some libs enable implicitly; if yours requires explicit enable,
            # use enable_motors from the official python implementation.
            pass

    def try_disable_all(self):
        ids = [mid for _, mid, _ in MOTOR_MAP]
        if hasattr(self.bus, "disable_motors"):
            self.bus.disable_motors(ids)

    def read_status(self, motor_id: int):
        # Your doc suggests bus.read_frame(motor_id) -> dict with position/velocity/torque
        try:
            return self.bus.read_frame(motor_id)
        except Exception:
            return None

    def write_cmd(self, motor_id: int, p_des: float, v_des: float, kp: float, kd: float, t_ff: float):
        # Your doc: bus.write_operation_frame(motor_id, p_des, v_des, kp, kd, t_ff)
        self.bus.write_operation_frame(
            motor_id=motor_id,
            p_des=float(p_des),
            v_des=float(v_des),
            kp=float(kp),
            kd=float(kd),
            t_ff=float(t_ff),
        )

    def compute_command(self, t_now: float, base_pos: float, params: TunerParams):
        if params.mode == "HOLD":
            return base_pos
        if params.mode == "SINE":
            return base_pos + params.sine_amp * math.sin(2.0 * math.pi * params.sine_hz * t_now)
        if params.mode == "STEP":
            return base_pos + params.step_sign * params.step_amp
        return base_pos

    def set_selected_motor(self, new_idx: int):
        with self._lock:
            self.selected_idx = int(np.clip(new_idx, 0, len(MOTOR_MAP) - 1))
            self._sync_base_pos_on_start = True
            self.buf.clear()

    def re_zero_base_pos(self):
        with self._lock:
            st = self.last_status
            if st and "position" in st:
                self.base_pos = float(st["position"])

    def coast_selected(self, seconds: float = 0.25):
        """Best-effort 'coast': kp=0,kd=0,t_ff=0 around current pos."""
        name, mid, model = self.selected_motor
        st = self.read_status(mid)
        p0 = self.base_pos
        if st and "position" in st:
            p0 = float(st["position"])
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end:
            try:
                self.write_cmd(mid, p0, 0.0, 0.0, 0.0, 0.0)
            except Exception:
                break
            time.sleep(0.01)

    def control_loop(self):
        self.running = True
        self.try_enable_all()

        t0 = time.perf_counter()
        next_time = time.perf_counter()

        while self.running:
            next_time += self.dt
            now = time.perf_counter()
            t_rel = now - t0

            with self._lock:
                name, mid, model = self.selected_motor
                params = self.params

            # read status
            st = self.read_status(mid)
            if st is not None:
                self.last_status = st
                if self._sync_base_pos_on_start and "position" in st:
                    with self._lock:
                        self.base_pos = float(st["position"])
                        self._sync_base_pos_on_start = False

            with self._lock:
                base_pos = float(self.base_pos)
                kp, kd = self.clamp_gains(params.kp, params.kd, model)
                mode = params.mode
                paused = params.paused
                t_ff = params.t_ff

            p_cmd = self.compute_command(t_rel, base_pos, self.params)

            if not paused:
                try:
                    self.write_cmd(mid, p_cmd, 0.0, kp, kd, t_ff)
                except Exception:
                    pass

            # append to buffer
            if st is not None:
                p = float(st.get("position", np.nan))
                v = float(st.get("velocity", np.nan))
                tau = float(st.get("torque", np.nan))
            else:
                p, v, tau = np.nan, np.nan, np.nan

            self.buf.append(t_rel, p, p_cmd, v, tau)

            if self.log_csv is not None:
                self.csv_rows.append((t_rel, name, mid, model, mode, kp, kd, t_ff, p_cmd, p, v, tau))

            # timing
            sleep_for = next_time - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # overrun: don't sleep
                pass

        # try to coast on exit
        try:
            self.coast_selected()
        except Exception:
            pass
        try:
            self.try_disable_all()
        except Exception:
            pass

    def save_csv(self):
        if not self.log_csv:
            return
        import csv
        header = ["t", "joint", "motor_id", "model", "mode", "kp", "kd", "t_ff", "p_cmd", "p", "v", "tau"]
        with open(self.log_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.csv_rows)

    def start(self):
        th = threading.Thread(target=self.control_loop, daemon=True)
        th.start()
        return th


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0", help="SocketCAN interface, e.g., can0")
    ap.add_argument("--hz", type=float, default=100.0, help="Control frequency (Hz)")
    ap.add_argument("--window_s", type=float, default=10.0, help="Plot time window (seconds)")
    ap.add_argument("--log_csv", default=None, help="Optional path to save CSV log on exit")
    args = ap.parse_args()

    tuner = RobStrideMitTuner(args.iface, args.hz, args.window_s, args.log_csv)
    ctrl_thread = tuner.start()

    # ---- Matplotlib live plot ----
    plt.rcParams["toolbar"] = "toolmanager"
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.canvas.manager.set_window_title("RobStride MIT Tuner")

    # Lines
    (line_p,) = ax1.plot([], [], label="position (meas)")
    (line_pc,) = ax1.plot([], [], label="position (cmd)")
    ax1.set_ylabel("rad")
    ax1.legend(loc="upper right")
    ax1.grid(True)

    (line_v,) = ax2.plot([], [], label="velocity (meas)")
    ax2.set_ylabel("rad/s")
    ax2.legend(loc="upper right")
    ax2.grid(True)

    (line_tau,) = ax3.plot([], [], label="torque (meas)")
    ax3.set_ylabel("Nm")
    ax3.set_xlabel("time (s)")
    ax3.legend(loc="upper right")
    ax3.grid(True)

    def update_title():
        name, mid, model = tuner.selected_motor
        with tuner._lock:
            p = tuner.params
            kp, kd = tuner.clamp_gains(p.kp, p.kd, model)
            mode = p.mode
            amp = p.sine_amp
            hz = p.sine_hz
            paused = p.paused
            step_amp = p.step_amp
            step_sign = p.step_sign
        fig.suptitle(
            f"{name} (id={mid}, {model})  |  mode={mode}  |  kp={kp:.2f} kd={kd:.3f}  "
            f"|  sine amp={amp:.3f}rad f={hz:.2f}Hz  |  step={step_sign*step_amp:.3f}rad  "
            f"|  {'PAUSED' if paused else 'RUNNING'}"
        )

    def animate(_):
        # pull buffer data
        t = np.array(tuner.buf.t, dtype=float)
        if t.size == 0:
            return (line_p, line_pc, line_v, line_tau)

        p = np.array(tuner.buf.p, dtype=float)
        pc = np.array(tuner.buf.p_cmd, dtype=float)
        v = np.array(tuner.buf.v, dtype=float)
        tau = np.array(tuner.buf.tau, dtype=float)

        line_p.set_data(t, p)
        line_pc.set_data(t, pc)
        line_v.set_data(t, v)
        line_tau.set_data(t, tau)

        # autoscale x to window
        ax1.set_xlim(max(0.0, t[-1] - tuner.window_s), t[-1])

        # autoscale y with some padding (robust to NaNs)
        def autoscale(ax, y):
            y = y[np.isfinite(y)]
            if y.size < 2:
                return
            ymin, ymax = float(np.min(y)), float(np.max(y))
            if abs(ymax - ymin) < 1e-6:
                ymin -= 1.0
                ymax += 1.0
            pad = 0.1 * (ymax - ymin)
            ax.set_ylim(ymin - pad, ymax + pad)

        autoscale(ax1, np.concatenate([p, pc]))
        autoscale(ax2, v)
        autoscale(ax3, tau)

        update_title()
        return (line_p, line_pc, line_v, line_tau)

    ani = FuncAnimation(fig, animate, interval=50, blit=False)

    # ---- Keyboard controls ----
    def on_key(event):
        key = event.key
        with tuner._lock:
            name, mid, model = tuner.selected_motor
            p = tuner.params

            def set_kp(new_kp):
                p.kp = float(new_kp)

            def set_kd(new_kd):
                p.kd = float(new_kd)

            def clamp_now():
                p.kp, p.kd = tuner.clamp_gains(p.kp, p.kd, model)

            # Gain step sizes (tweak as you like)
            kp_step = 10.0 if (event.shift is False) else 50.0
            kd_step = 0.05 if (event.shift is False) else 0.25

            if key in ["escape", "q"]:
                tuner.running = False
                plt.close(fig)
                return

            if key == " ":
                p.paused = not p.paused
                return

            if key == "[":
                tuner.set_selected_motor(tuner.selected_idx - 1)
                return
            if key == "]":
                tuner.set_selected_motor(tuner.selected_idx + 1)
                return

            if key == "m":
                p.mode = {"HOLD": "SINE", "SINE": "STEP", "STEP": "HOLD"}.get(p.mode, "HOLD")
                return

            if key == "r":
                # base position = current measured
                tuner.re_zero_base_pos()
                return

            if key == "c":
                tuner.buf.clear()
                return

            # Gains
            if key == "up":
                set_kp(p.kp + kp_step)
                clamp_now()
                return
            if key == "down":
                set_kp(p.kp - kp_step)
                clamp_now()
                return
            if key == "right":
                set_kd(p.kd + kd_step)
                clamp_now()
                return
            if key == "left":
                set_kd(p.kd - kd_step)
                clamp_now()
                return

            # Sine amplitude / frequency
            if key == "a":
                p.sine_amp = float(np.clip(p.sine_amp + 0.01, 0.0, 1.5))
                return
            if key == "z":
                p.sine_amp = float(np.clip(p.sine_amp - 0.01, 0.0, 1.5))
                return
            if key == "f":
                p.sine_hz = float(np.clip(p.sine_hz + 0.05, 0.01, 5.0))
                return
            if key == "v":
                p.sine_hz = float(np.clip(p.sine_hz - 0.05, 0.01, 5.0))
                return

            # Step helpers
            if key == "s":
                p.step_sign *= -1
                return
            if key == "e":
                p.step_amp = p.sine_amp
                return

    fig.canvas.mpl_connect("key_press_event", on_key)

    # Console hint
    print("\nRobStride MIT Tuner running.")
    print("Focus the plot window and use keys: q/ESC quit, SPACE pause, [/] motor, m mode, arrows kp/kd, a/z amp, f/v freq.\n")

    try:
        plt.show()
    finally:
        tuner.running = False
        try:
            ctrl_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            tuner.save_csv()
            if args.log_csv:
                print(f"Saved log: {args.log_csv}")
        except Exception as e:
            print(f"Failed to save CSV: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
