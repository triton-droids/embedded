#!/usr/bin/env python3
"""
Dummy live plot + interactive commands (NO motors) — mac-safe.

Adds live plots:
- position & command
- error
- velocity
- torque (dummy effort)

Run:
  python3 dummy_live_cmd_4plots.py
"""

import time
import math
import threading
from dataclasses import dataclass, field
from typing import Optional
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class Excitation:
    mode: str = "none"            # "none" | "sine"
    amp_deg: float = 0.0
    freq_hz: float = 0.0
    t0: float = 0.0
    duration_s: Optional[float] = None
    center_deg: float = 0.0


@dataclass
class PlantState:
    pos_deg: float = 0.0
    vel_deg_s: float = 0.0

    target_deg: float = 0.0
    cmd_deg: float = 0.0

    kp: float = 20.0
    kd: float = 0.5

    ex: Excitation = field(default_factory=Excitation)


class DummyPlant:
    """
    A stable toy plant:
      accel = kp_eff*(cmd-pos) - kd_eff*vel

    Also reports a "torque" proxy:
      torque = kp_eff*(cmd-pos) - kd_eff*vel   (PD effort)
    """
    def __init__(self, ramp_deg_s: float = 120.0):
        self.ramp_deg_s = float(ramp_deg_s)
        self.state = PlantState()
        self.lock = threading.Lock()
        self.running = True

        # last computed effort (torque proxy)
        self._torque_nm = 0.0

    def _kp_eff(self, kp: float) -> float:
        # scale to keep numbers sane in dummy world
        return clamp(kp, 0.0, 5000.0) * 0.03

    def _kd_eff(self, kd: float) -> float:
        return clamp(kd, 0.0, 100.0) * 0.25 + 0.2

    def integrate(self, dt: float):
        """Advance excitation, ramp cmd, integrate dynamics (called from main thread)."""
        now = time.time()
        with self.lock:
            s = self.state

            # excitation -> updates target_deg
            if s.ex.mode == "sine":
                ex = s.ex
                if ex.duration_s is not None and (now - ex.t0) >= ex.duration_s:
                    s.ex = Excitation()
                    s.target_deg = ex.center_deg
                else:
                    s.target_deg = ex.center_deg + ex.amp_deg * math.sin(
                        2 * math.pi * ex.freq_hz * (now - ex.t0)
                    )

            # ramp cmd_deg toward target_deg
            max_step = self.ramp_deg_s * dt
            delta = s.target_deg - s.cmd_deg
            if abs(delta) <= max_step:
                s.cmd_deg = s.target_deg
            else:
                s.cmd_deg += math.copysign(max_step, delta)

            # dynamics
            err = s.cmd_deg - s.pos_deg
            kp_eff = self._kp_eff(s.kp)
            kd_eff = self._kd_eff(s.kd)

            # PD "effort" proxy (treat as torque in Nm for plotting)
            effort = kp_eff * err - kd_eff * s.vel_deg_s
            self._torque_nm = effort

            accel = effort
            s.vel_deg_s += accel * dt
            s.pos_deg += s.vel_deg_s * dt

    # --- controls (thread-safe) ---
    def set_kp(self, kp: float):
        with self.lock:
            self.state.kp = float(kp)

    def set_kd(self, kd: float):
        with self.lock:
            self.state.kd = float(kd)

    def hold(self):
        with self.lock:
            s = self.state
            s.ex = Excitation()
            s.target_deg = s.pos_deg
            s.cmd_deg = s.pos_deg

    def step(self, delta_deg: float):
        with self.lock:
            s = self.state
            s.ex = Excitation()
            s.target_deg += float(delta_deg)

    def goto(self, angle_deg: float):
        with self.lock:
            s = self.state
            s.ex = Excitation()
            s.target_deg = float(angle_deg)

    def sine(self, amp_deg: float, freq_hz: float, duration_s: Optional[float]):
        with self.lock:
            s = self.state
            center = s.target_deg
            s.ex = Excitation(
                mode="sine",
                amp_deg=float(amp_deg),
                freq_hz=float(freq_hz),
                t0=time.time(),
                duration_s=duration_s,
                center_deg=center,
            )

    def stop_excitation(self):
        with self.lock:
            self.state.ex = Excitation()

    def snapshot(self):
        with self.lock:
            s = self.state
            torque = float(self._torque_nm)
            err = s.cmd_deg - s.pos_deg
            return (s.pos_deg, s.cmd_deg, err, s.vel_deg_s, torque, s.kp, s.kd, s.ex.mode)


def command_loop(plant: DummyPlant):
    print("\nCommands:")
    print("  sine <amp_deg> <freq_hz> [duration_s]")
    print("  stop")
    print("  hold")
    print("  step <deg>")
    print("  goto <deg>")
    print("  kp <value>")
    print("  kd <value>")
    print("  q\n")

    while True:
        try:
            cmd = input(">> ").strip().lower()
            if not cmd:
                continue

            if cmd in ("q", "quit", "exit"):
                plant.running = False
                print("Exiting.")
                return

            if cmd == "hold":
                plant.hold()
                print("OK: hold")
                continue

            if cmd == "stop":
                plant.stop_excitation()
                print("OK: stop")
                continue

            if cmd.startswith("kp "):
                plant.set_kp(float(cmd.split()[1]))
                print("OK: kp")
                continue

            if cmd.startswith("kd "):
                plant.set_kd(float(cmd.split()[1]))
                print("OK: kd")
                continue

            if cmd.startswith("step "):
                plant.step(float(cmd.split()[1]))
                print("OK: step")
                continue

            if cmd.startswith("goto "):
                plant.goto(float(cmd.split()[1]))
                print("OK: goto")
                continue

            if cmd.startswith("sine "):
                parts = cmd.split()
                if len(parts) not in (3, 4):
                    print("Usage: sine <amp_deg> <freq_hz> [duration_s]")
                    continue
                amp = float(parts[1])
                freq = float(parts[2])
                dur = float(parts[3]) if len(parts) == 4 else None
                plant.sine(amp, freq, dur)
                dstr = f"{dur}s" if dur is not None else "inf"
                print(f"OK: sine amp={amp}deg freq={freq}Hz dur={dstr}")
                continue

            print("Unknown command.")

        except KeyboardInterrupt:
            plant.running = False
            print("Exiting.")
            return
        except Exception as e:
            print(f"Command error: {e}")


def main():
    plant = DummyPlant(ramp_deg_s=120.0)

    # Start CLI thread so plot remains responsive
    threading.Thread(target=command_loop, args=(plant,), daemon=True).start()

    # Plot buffers
    window_s = 10.0
    maxlen = int(window_s * 60) + 200
    t0 = time.time()

    tt = deque(maxlen=maxlen)
    pos = deque(maxlen=maxlen)
    cmd = deque(maxlen=maxlen)
    err = deque(maxlen=maxlen)
    vel = deque(maxlen=maxlen)
    tq = deque(maxlen=maxlen)

    # 4 stacked plots
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    ax_p, ax_e, ax_v, ax_t = axes

    (l_pos,) = ax_p.plot([], [], label="pos (deg)")
    (l_cmd,) = ax_p.plot([], [], label="cmd (deg)")
    ax_p.set_ylabel("deg")
    ax_p.legend(loc="upper right")
    ax_p.grid(True, alpha=0.3)

    (l_err,) = ax_e.plot([], [], label="error = cmd-pos (deg)")
    ax_e.set_ylabel("deg")
    ax_e.legend(loc="upper right")
    ax_e.grid(True, alpha=0.3)

    (l_vel,) = ax_v.plot([], [], label="vel (deg/s)")
    ax_v.set_ylabel("deg/s")
    ax_v.legend(loc="upper right")
    ax_v.grid(True, alpha=0.3)

    (l_tq,) = ax_t.plot([], [], label="torque proxy (Nm)")
    ax_t.set_ylabel("Nm")
    ax_t.set_xlabel("time (s)")
    ax_t.legend(loc="upper right")
    ax_t.grid(True, alpha=0.3)

    last_t = time.time()

    def update(_frame):
        nonlocal last_t
        now = time.time()
        dt = now - last_t
        last_t = now

        if plant.running:
            plant.integrate(min(dt, 0.05))

        p, c, e, v, torque, kp, kd, exmode = plant.snapshot()

        t = now - t0
        tt.append(t)
        pos.append(p)
        cmd.append(c)
        err.append(e)
        vel.append(v)
        tq.append(torque)

        x = list(tt)

        l_pos.set_data(x, list(pos))
        l_cmd.set_data(x, list(cmd))
        l_err.set_data(x, list(err))
        l_vel.set_data(x, list(vel))
        l_tq.set_data(x, list(tq))

        if len(x) >= 2:
            xmax = x[-1]
            xmin = max(0.0, xmax - window_s)
            ax_t.set_xlim(xmin, xmax)

        # autoscale each axis' y, keep x fixed
        for ax in axes:
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        fig.suptitle(f"Dummy live tuning | kp={kp:.2f} kd={kd:.2f} ex={exmode}", y=0.995)
        return (l_pos, l_cmd, l_err, l_vel, l_tq)

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    plant.running = False


if __name__ == "__main__":
    main()
