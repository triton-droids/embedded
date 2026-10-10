#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Run the mjlab / rsl_rl motion-tracking ONNX policy on the real robot at 50 Hz.

Serves a BeyondMimic-style tracking policy. (The older 60 Hz rl-games runner,
run_policy.py, had a different observation contract; it is in the branch history.)

Observation (56), order fixed by mjlab tracking_env_cfg + the task's overrides:
    [ 0:10]  command: reference joint_pos at time_step
    [10:20]  command: reference joint_vel at time_step
    [20:23]  base_ang_vel        rad/s, body frame, UNSCALED
    [23:33]  joint_pos - default_joint_pos      (default is all zeros)
    [33:43]  joint_vel
    [43:53]  last action
    [53:56]  projected_gravity   unit vector, body frame

Joint order is REAL_JOINT_ORDER (left block then right block), which is what the
sim uses. No interleaving.

Action: joint_target = default_joint_pos + action_scale * action, scale = 0.2.

Control rate 50 Hz (sim timestep 0.005 x decimation 4), matching the reference
motion's 50 fps.

The ONNX is self-contained: the observation normalizer and the 299-frame
reference motion are both baked in. Feeding it `time_step` returns the reference
pose for that frame, which is fed back into obs[0:20] on the next step.

Modes:
    --offline   No CAN at all. IMU and policy only. Nothing can move.
    --dry-run   Connects to motors and ENABLES THEM, reads state, writes no
                targets. RobotInterface.connect() energizes motors regardless of
                dry_run, so this is not a no-torque mode.
    (default)   Full control.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_hardware import REAL_JOINT_ORDER, apply_shared_hardware_config

N_JOINTS = 10
OBS_DIM = 56
GRAVITY_WORLD = (0.0, 0.0, 9.80665)


def require_key(cfg: dict[str, Any], key: str, ctx: str = "config") -> Any:
    if key not in cfg:
        raise KeyError(f"Missing required key '{ctx}.{key}'")
    return cfg[key]


class ImuSource:
    """Serial/CAN IMU reader producing policy-frame ang_vel and projected gravity.

    The sensor is mounted +X forward, +Y left, +Z up. The policy's base frame is
    +X right, +Y forward, +Z up. `axis_map` carries that permutation as
    (source_index, sign) per policy axis; it is an exact integer swap, so it
    introduces no interpolation error.
    """

    def __init__(self, cfg: dict[str, Any]):
        self.enabled = bool(require_key(cfg, "enabled", "imu"))
        self.source = str(cfg.get("source", "serial"))
        self.port = str(require_key(cfg, "port", "imu"))
        self.baud = int(cfg.get("baud", 115200))
        self.rate_hz = float(cfg.get("rate_hz", 100.0))
        self.wait_s = float(cfg.get("wait_for_first_sample_s", 2.0))
        self.zupt_gyro_dps = float(cfg.get("zupt_gyro_dps", 6.0))
        self.gyro_bias_dps = np.asarray(cfg.get("gyro_bias_dps", [0.0, 0.0, 0.0]), dtype=float)

        amap = cfg.get("axis_map", [[0, 1], [1, 1], [2, 1]])
        self.src_idx = np.asarray([int(a[0]) for a in amap], dtype=int)
        self.src_sign = np.asarray([float(a[1]) for a in amap], dtype=float)

        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._has_sample = False
        self._ang_vel = np.zeros(3, dtype=float)
        self._up = np.array([0.0, 0.0, 1.0], dtype=float)
        self.last_error: str | None = None

    def _to_policy_frame(self, v_sensor: np.ndarray) -> np.ndarray:
        return self.src_sign * np.asarray(v_sensor, dtype=float)[self.src_idx]

    def start(self) -> None:
        if not self.enabled:
            print("[IMU] disabled: ang_vel=0, projected_gravity=(0,0,-1)")
            return
        from utils.imu_read import RK4DeadReckoner, iter_imu_samples  # noqa: F401

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="imu")
        self._thread.start()
        t0 = time.time()
        while time.time() - t0 < self.wait_s:
            with self._lock:
                if self._has_sample:
                    print(f"[IMU] streaming from {self.port}")
                    return
            time.sleep(0.01)
        raise RuntimeError(
            f"No IMU sample within {self.wait_s:.1f}s from {self.port}. "
            "Refusing to run a tracking policy on a zeroed gravity vector."
        )

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def get(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (ang_vel_rad_s, projected_gravity) in the policy base frame."""
        with self._lock:
            ang = self._ang_vel.copy()
            up = self._up.copy()
        return self._to_policy_frame(ang), -self._to_policy_frame(up)

    def _loop(self) -> None:
        from utils.imu_read import RK4DeadReckoner, iter_imu_samples

        try:
            integrator = RK4DeadReckoner(
                gravity_world=GRAVITY_WORLD,
                zupt_gyro_dps=self.zupt_gyro_dps,
            )
            kwargs: dict[str, Any] = dict(
                source=self.source,
                rate_hz=self.rate_hz,
                include_all=True,
                integrator=integrator,
            )
            if self.source == "can":
                kwargs.update(can_interface="socketcan", can_channel=self.port,
                              can_bitrate=self.baud)
            else:
                kwargs.update(port=self.port, baud=self.baud)

            for sample in iter_imu_samples(**kwargs):
                if not self._running:
                    break
                gyro_dps = sample.get("gyro_dps")
                if gyro_dps is None:
                    continue
                # Subtract the measured constant bias in the SENSOR frame, before
                # the axis permutation. imu_read never bias-corrects gyro_dps
                # itself; only the integrator's internal omega is corrected.
                gyro = np.asarray(gyro_dps, dtype=float) - self.gyro_bias_dps
                up = sample.get("up_body")
                with self._lock:
                    self._ang_vel = np.radians(gyro)
                    if up is not None:
                        self._up = np.asarray(up, dtype=float)
                    self._has_sample = True
        except Exception as exc:  # surfaced by the control loop
            with self._lock:
                self.last_error = str(exc)
            print(f"[IMU] reader stopped: {exc}")


class TrackingPolicy:
    """ONNX tracking policy with its baked-in normalizer and reference motion."""

    def __init__(self, path: Path):
        import onnxruntime as ort

        self.session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        self.out_names = [o.name for o in self.session.get_outputs()]
        shape = self.session.get_inputs()[0].shape
        if int(shape[-1]) != OBS_DIM:
            raise ValueError(f"Policy expects obs dim {shape[-1]}, this runner builds {OBS_DIM}")
        self._zero = np.zeros((1, OBS_DIM), dtype=np.float32)

    def _run(self, obs: np.ndarray, time_step: int) -> dict[str, np.ndarray]:
        feed = {
            "obs": obs.astype(np.float32).reshape(1, OBS_DIM),
            "time_step": np.array([[float(time_step)]], dtype=np.float32),
        }
        return dict(zip(self.out_names, self.session.run(None, feed)))

    def reference(self, time_step: int) -> tuple[np.ndarray, np.ndarray]:
        """Reference joint_pos / joint_vel at a frame. Actions here are ignored."""
        out = self._run(self._zero, time_step)
        return (out["joint_pos"][0].astype(float), out["joint_vel"][0].astype(float))

    def act(self, obs: np.ndarray, time_step: int) -> np.ndarray:
        return self._run(obs, time_step)["actions"][0].astype(float)


class TrackingObsBuilder:
    def __init__(self, default_joint_pos: np.ndarray):
        self.default_joint_pos = np.asarray(default_joint_pos, dtype=float)
        self.last_action = np.zeros(N_JOINTS, dtype=float)

    def build(
        self,
        ref_pos: np.ndarray,
        ref_vel: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        ang_vel: np.ndarray,
        proj_gravity: np.ndarray,
    ) -> np.ndarray:
        obs = np.concatenate([
            ref_pos,                                  # [ 0:10]
            ref_vel,                                  # [10:20]
            ang_vel,                                  # [20:23]
            joint_pos - self.default_joint_pos,       # [23:33]
            joint_vel,                                # [33:43]
            self.last_action,                         # [43:53]
            proj_gravity,                             # [53:56]
        ])
        if obs.shape != (OBS_DIM,):
            raise RuntimeError(f"built obs of shape {obs.shape}, expected ({OBS_DIM},)")
        return obs

    def note_action(self, action: np.ndarray) -> None:
        self.last_action = np.asarray(action, dtype=float).copy()


class StepLogger:
    """Buffers one row per control step and writes them all to a .npz on close.

    A run is at most a few hundred steps, so everything stays in memory and the
    file is written once, from shutdown(), which also runs after Ctrl+C or a
    safety trip. Each field becomes one array of shape (steps, ...); `meta` is a
    JSON string with the joint order, gains, limits and how the run ended.
    """

    def __init__(self, path: Path, meta: dict[str, Any]):
        self.path = path
        self.meta = meta
        self.rows: dict[str, list[np.ndarray]] = {}

    def add(self, **fields: Any) -> None:
        for key, value in fields.items():
            self.rows.setdefault(key, []).append(np.asarray(value, dtype=float))

    def close(self) -> None:
        if not self.rows:
            print("[LOG] no steps recorded; nothing written")
            return
        arrays = {k: np.stack(v) for k, v in self.rows.items()}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(self.path, meta=np.array(json.dumps(self.meta)), **arrays)
        n = len(next(iter(arrays.values())))
        print(f"[LOG] {n} steps -> {self.path}")
        print(f"[LOG] inspect: ./.venv/bin/python utils/plot_tracking_log.py {self.path}")


class TorqueGuard:
    """Keeps every motor inside the torque ratings from the RobStride manuals.

    Two jobs, both in MOTOR space (the ankles go through a linkage, and torque is
    a motor-side quantity):

    limit()  Before each write, bound the spring part of the MIT law,
             |kp * (motor_target - motor_pos)| <= command_cap, by pulling the
             target toward the current position. The damping part (-kd * vel) is
             left alone so braking is never weakened.

    check()  After each read, stop the run if a motor
               - sits near its peak torque for several consecutive replies,
               - averages more than its stalled rated torque over a window
                 (it would overheat if that continued),
               - is too hot, or reports an impossible temperature,
               - misses several replies in a row (the policy would be running blind).

    Limits by model, from the RS02/RS03/RS04 manuals:
        peak 17 / 60 / 120 Nm, rated stalled 6 / 15 / 28.5 Nm, overtemp fault 135-145 C.
    The motors' own TORQUE_LIMIT is deliberately not touched: the manuals say not
    to change it, so it stays as the last line of defence.
    """

    DEFAULTS: dict[str, Any] = {
        "enabled": True,
        "command_cap_nm": {"rs-04": 80.0, "rs-03": 40.0, "rs-02": 11.0},
        "abs_stop_nm": {"rs-04": 108.0, "rs-03": 54.0, "rs-02": 15.0},
        "abs_stop_consecutive": 3,
        "sustained_rms_stop_nm": {"rs-04": 28.5, "rs-03": 15.0, "rs-02": 6.0},
        "sustained_window_s": 2.0,
        "temp_warn_c": 70.0,
        "temp_stop_c": 85.0,
        "max_consecutive_missed": 5,
    }

    def __init__(self, cfg: dict[str, Any], control_hz: float, states: list) -> None:
        c = {k: (dict(v) if isinstance(v, dict) else v) for k, v in self.DEFAULTS.items()}
        for k, v in (cfg or {}).items():
            if isinstance(v, dict) and isinstance(c.get(k), dict):
                c[k].update(v)
            else:
                c[k] = v
        self.cfg = c
        self.enabled = bool(c["enabled"])
        self.states = states
        n = len(states)
        models = [st.model for st in states]
        for table in ("command_cap_nm", "abs_stop_nm", "sustained_rms_stop_nm"):
            missing = sorted({m for m in models if m not in c[table]})
            if missing:
                raise ValueError(f"torque_guard.{table} has no entry for motor model(s) {missing}")
        self.cap = np.array([c["command_cap_nm"][m] for m in models], dtype=float)
        self.abs_stop = np.array([c["abs_stop_nm"][m] for m in models], dtype=float)
        self.sustained = np.array([c["sustained_rms_stop_nm"][m] for m in models], dtype=float)
        self.window = max(1, int(round(float(c["sustained_window_s"]) * control_hz)))
        self._sq = np.full((self.window, n), np.nan)
        self._sq_idx = 0
        self._missed = np.zeros(n, dtype=int)
        self._over = np.zeros(n, dtype=int)
        self._warned: set[tuple[str, int]] = set()

    def describe(self) -> str:
        if not self.enabled:
            return "[GUARD] DISABLED (torque_guard.enabled = false)"
        c = self.cfg
        return (f"[GUARD] command cap {c['command_cap_nm']} Nm | stop if |tau| >= {c['abs_stop_nm']} "
                f"x{c['abs_stop_consecutive']}, {c['sustained_window_s']:.1f}s RMS > "
                f"{c['sustained_rms_stop_nm']}, temp >= {c['temp_stop_c']:.0f} C, "
                f"{c['max_consecutive_missed']} missed replies in a row")

    def _warn_once(self, kind: str, i: int, msg: str) -> None:
        if (kind, i) not in self._warned:
            self._warned.add((kind, i))
            print(f"[GUARD] warning: {msg}")

    def check(self, fresh: np.ndarray, step_idx: int) -> str | None:
        """Return a stop reason, or None if every motor is within limits."""
        if not self.enabled:
            return None
        c = self.cfg
        row = np.full(len(self.states), np.nan)
        for i, st in enumerate(self.states):
            name = f"motor {st.motor_id} ({st.joint_name}, {st.model})"
            if not fresh[i]:
                if step_idx > 0:   # the first cycle routinely misses replies
                    self._missed[i] += 1
                if self._missed[i] >= int(c["max_consecutive_missed"]):
                    return f"{name} missed {self._missed[i]} replies in a row"
                continue
            self._missed[i] = 0

            temp = float(st.temp_c)
            if not -40.0 <= temp <= 200.0:
                return f"{name} reports {temp:.0f} C, not a real temperature (thermistor fault?)"
            if temp >= float(c["temp_stop_c"]):
                return f"{name} at {temp:.0f} C (stop threshold {c['temp_stop_c']:.0f} C)"
            if temp >= float(c["temp_warn_c"]):
                self._warn_once("temp", i, f"{name} at {temp:.0f} C")

            tau = abs(float(st.torque_nm))
            self._over[i] = self._over[i] + 1 if tau >= self.abs_stop[i] else 0
            if self._over[i] >= int(c["abs_stop_consecutive"]):
                return (f"{name} at {tau:.1f} Nm for {self._over[i]} replies "
                        f"(stop threshold {self.abs_stop[i]:.0f} Nm)")
            row[i] = tau * tau

        self._sq[self._sq_idx % self.window] = row
        self._sq_idx += 1
        if self._sq_idx >= self.window // 2:
            counts = np.sum(~np.isnan(self._sq), axis=0)
            rms = np.sqrt(np.nanmean(np.where(counts > 0, self._sq, 0.0), axis=0))
            for i, st in enumerate(self.states):
                if counts[i] < self.window // 2:
                    continue
                if rms[i] > self.sustained[i]:
                    return (f"motor {st.motor_id} ({st.joint_name}, {st.model}) averaged {rms[i]:.1f} Nm "
                            f"over {self.cfg['sustained_window_s']:.1f}s (rated {self.sustained[i]:.1f} Nm)")
                if rms[i] > 0.8 * self.sustained[i]:
                    self._warn_once("rms", i, f"motor {st.motor_id} averaging {rms[i]:.1f} Nm, "
                                              f"80% of its {self.sustained[i]:.1f} Nm rating")
        return None

    def limit(self, robot, targets: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Bound the commanded spring torque. Returns (targets, clipped mask, estimated torque)."""
        out = np.asarray(targets, dtype=float).copy()
        clipped = np.zeros(len(out))
        tau_est = np.zeros(len(out))
        for i, st in enumerate(self.states):
            m = float(st.position_phys)
            mt = robot.joint_to_motor_physical(st, out[i])
            if self.enabled and st.kp > 0.0:
                span = self.cap[i] / st.kp
                if abs(mt - m) > span:
                    out[i] = robot.motor_physical_to_joint(st, m + math.copysign(span, mt - m))
                    mt = robot.joint_to_motor_physical(st, out[i])
                    clipped[i] = 1.0
            tau_est[i] = st.kp * (mt - m) - st.kd * float(st.velocity_phys)
        return out, clipped, tau_est


def load_config(path: Path) -> dict[str, Any]:
    cfg = json.loads(path.read_text(encoding="utf-8"))
    apply_shared_hardware_config(cfg)
    # apply_shared_hardware_config fills kp/kd_by_joint from robot_hardware's
    # POLICY_* tables (from the older 60 Hz runner). This policy was trained with
    # different, softer gains, so the tracking config can override them here
    # without touching the other runner.
    for key in ("kp", "kd"):
        override = cfg.get(f"tracking_{key}_by_joint")
        if override:
            unknown = sorted(set(override) - set(REAL_JOINT_ORDER))
            if unknown:
                raise ValueError(f"tracking_{key}_by_joint has unknown joints: {unknown}")
            cfg[f"{key}_by_joint"].update({j: float(v) for j, v in override.items()})
    # The tracking policy's default joint pose is all zeros: the sim model has no
    # keyframe and InitialStateCfg.joint_pos defaults to {".*": 0.0}. This must
    # override the crouched default pose that robot_hardware supplies for the
    # locomotion stack, or every action is offset by the difference.
    cfg["default_joint_pos_real_rad_by_joint"] = {j: 0.0 for j in REAL_JOINT_ORDER}
    if list(cfg["real_joint_order"]) != list(REAL_JOINT_ORDER):
        raise ValueError("real_joint_order drifted from robot_hardware.REAL_JOINT_ORDER")
    return cfg


class TrackingController:
    def __init__(self, cfg: dict[str, Any], mode: str):
        self.cfg = cfg
        self.mode = mode  # "offline" | "dry-run" | "live"

        self.control_hz = float(require_key(cfg, "control_hz"))
        self.dt = 1.0 / self.control_hz
        self.action_scale = float(require_key(cfg, "action_scale"))
        self.motion_frames = int(require_key(cfg, "motion_frames"))
        self.loop_motion = bool(cfg.get("loop_motion", False))
        self.time_step = int(cfg.get("start_time_step", 0))

        self.joint_names = list(REAL_JOINT_ORDER)
        self.default_joint_pos = np.zeros(N_JOINTS, dtype=float)

        # Per-joint sign bridging the hardware convention to the sim convention.
        # Measured by hand back-driving each joint (joint_direction_monitor.py):
        # both thigh joints turn opposite to the sim. INVERSION_ARRAY has them
        # correct relative to each other but flipped in absolute terms. Fixed
        # here rather than in robot_hardware.py, which other tools share.
        sign_cfg = cfg.get("joint_sign_by_joint", {})
        self.joint_sign = np.array(
            [float(sign_cfg.get(j, 1.0)) for j in self.joint_names], dtype=float
        )
        flipped = [j for j, sgn in zip(self.joint_names, self.joint_sign) if sgn < 0]
        if flipped:
            print(f"[SIGN] inverted vs sim, corrected here: {flipped}")

        # Joint limits come from robot_hardware, but those values were
        # transcribed from the sim model, so they are in the SIM convention.
        # For a sign-flipped joint the hardware's reachable range is the
        # negated-and-swapped interval, so map the limits through joint_sign
        # rather than trusting the stored pair. Symmetric ranges are unaffected;
        # left_hip2 (-1.5700, +0.4363) is the one that actually moves.
        limits = cfg["joint_limits_rad_by_joint"]
        lo_sim = np.array([limits[j][0] for j in self.joint_names], dtype=float)
        hi_sim = np.array([limits[j][1] for j in self.joint_names], dtype=float)
        lo = np.where(self.joint_sign > 0, lo_sim, -hi_sim)
        hi = np.where(self.joint_sign > 0, hi_sim, -lo_sim)
        for j, sgn, a, b in zip(self.joint_names, self.joint_sign, lo, hi):
            if sgn < 0 and abs(a + b) > 1e-9:
                print(f"[LIMITS] {j} flipped: hardware range ({a:+.4f}, {b:+.4f})")
        if bool(cfg.get("use_soft_joint_limits", True)):
            f = float(cfg.get("soft_joint_limit_factor", 0.95))
            c, h = 0.5 * (lo + hi), 0.5 * (hi - lo)
            lo, hi = c - f * h, c + f * h

        # Extra per-joint clamp, intersected with the limits above so it can
        # only ever tighten. Used to bound hip2, whose limit interval was
        # derived from the sign flip rather than measured on hardware.
        clamp = cfg.get("joint_clamp_rad_by_joint", {})
        for i, j in enumerate(self.joint_names):
            if j in clamp:
                c_lo, c_hi = float(clamp[j][0]), float(clamp[j][1])
                lo[i], hi[i] = max(lo[i], c_lo), min(hi[i], c_hi)
                print(f"[CLAMP] {j} restricted to ({lo[i]:+.4f}, {hi[i]:+.4f})")
        self.clip_lo, self.clip_hi = lo, hi

        mv = float(cfg.get("max_vel_rad_s", 4.5))
        by = cfg.get("max_vel_rad_s_by_joint", {})
        self.max_vel = np.array([float(by.get(j, mv)) for j in self.joint_names], dtype=float)

        self.policy = TrackingPolicy(Path(require_key(cfg, "policy_path")).expanduser())
        self.obs_builder = TrackingObsBuilder(self.default_joint_pos)
        self.imu = ImuSource(dict(require_key(cfg, "imu")))

        # RobotInterface's safety check uses the raw hard limits. The knee's
        # upper limit is exactly 0 and the policy's zero pose is legs-straight,
        # so the joint rests ON its limit and sensor noise trips it instantly.
        # Widen only the limits the safety checker sees; self.clip_lo/hi above
        # were already computed from the true sim limits and stay tight.
        margin = float(cfg.get("safety_limit_margin_rad", 0.05))
        if margin > 0.0:
            widened = {}
            for j in self.joint_names:
                j_lo, j_hi = cfg["joint_limits_rad_by_joint"][j]
                widened[j] = [float(j_lo) - margin, float(j_hi) + margin]
            cfg["joint_limits_rad_by_joint"] = widened
            print(f"[SAFETY] limit margin {margin:.3f} rad ({math.degrees(margin):.1f} deg) "
                  f"applied to the safety checker only")

        self.robot = None
        if self.mode != "offline":
            from robot_interface import RobotInterface
            self.robot = RobotInterface(cfg, dry_run=(self.mode == "dry-run"))
        self.hold_target: np.ndarray | None = None

        self.guard: TorqueGuard | None = None
        if self.robot is not None:
            self.guard = TorqueGuard(dict(cfg.get("torque_guard", {})), self.control_hz, self.robot.states)
            print("[GAINS] " + "  ".join(f"m{st.motor_id} {st.kp:g}/{st.kd:g}" for st in self.robot.states)
                  + "   (kp/kd per motor)")
            print(self.guard.describe())

        self.batched_feedback = bool(cfg.get("use_batched_feedback", True))
        self.feedback_timeout = float(cfg.get("feedback_timeout_s", 0.015))
        self._fb_updated = 0
        self._fb_cycles = 0
        self._fb_short = 0

        self.commanded = self.default_joint_pos.copy()
        self.running = False
        self.step_idx = 0
        self.log: StepLogger | None = None
        self.stop_reason = "completed"
        self._t0 = time.perf_counter()
        self._t_prev = self._t0
        hz = float(cfg.get("status_print_hz", 2.0))
        self.status_interval = (1.0 / hz) if hz > 0 else None
        self._last_status = 0.0

    def connect(self) -> bool:
        self.imu.start()
        if self.robot is not None:
            if not self.robot.connect():
                return False
            joint_pos, _ = self.robot.joint_vectors_real()
            self.commanded = joint_pos.copy()
            self.hold_target = joint_pos.copy()
        self.running = True
        print(
            f"Tracking controller ready. mode={self.mode} {self.control_hz:.0f} Hz "
            f"obs_dim={OBS_DIM} frames={self.motion_frames} loop={self.loop_motion}"
        )
        return True

    def request_stop(self) -> None:
        self.running = False
        self.stop_reason = "stop requested (Ctrl+C / SIGTERM)"

    def enable_logging(self, log_dir: Path, config_path: Path) -> None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        meta: dict[str, Any] = {
            "mode": self.mode,
            "started": stamp,
            "config_path": str(config_path),
            "policy_path": str(self.cfg.get("policy_path")),
            "control_hz": self.control_hz,
            "action_scale": self.action_scale,
            "joint_names": self.joint_names,
            "joint_sign": self.joint_sign.tolist(),
            "clip_lo": self.clip_lo.tolist(),
            "clip_hi": self.clip_hi.tolist(),
            "max_vel_rad_s": self.max_vel.tolist(),
        }
        if self.robot is not None:
            states = self.robot.states
            meta.update({
                "motor_ids": [st.motor_id for st in states],
                "motor_direction": [st.direction for st in states],
                "kp": [st.kp for st in states],
                "kd": [st.kd for st in states],
                "robot_limit_lo": [st.limit_lo for st in states],
                "robot_limit_hi": [st.limit_hi for st in states],
                "motor_models": [st.model for st in states],
            })
        if self.guard is not None:
            meta["torque_guard"] = self.guard.cfg
        self.log = StepLogger(log_dir / f"tracking_{self.mode}_{stamp}.npz", meta)
        print(f"[LOG] recording every step to {self.log.path}")

    def shutdown(self) -> None:
        self.running = False
        self.imu.stop()
        if self.robot is not None:
            self.robot.shutdown()
        if self.log is not None:
            self.log.meta["stop_reason"] = self.stop_reason
            self.log.meta["steps"] = self.step_idx
            try:
                self.log.close()
            except Exception as exc:
                print(f"[LOG] failed to write {self.log.path}: {exc}")
        print("Tracking controller shutdown complete.")

    def _joint_state(self) -> tuple[np.ndarray, np.ndarray]:
        if self.robot is None:
            return self.default_joint_pos.copy(), np.zeros(N_JOINTS, dtype=float)
        if self.batched_feedback:
            got = self.robot.read_feedback_batched(timeout=self.feedback_timeout)
            self._fb_updated += got
            self._fb_cycles += 1
            if got < N_JOINTS:
                self._fb_short += 1
        else:
            self.robot.read_feedback()
        return self.robot.joint_vectors_real()

    def step(self) -> None:
        t_step = time.perf_counter()
        ref_pos, ref_vel = self.policy.reference(self.time_step)
        joint_pos_hw, joint_vel_hw = self._joint_state()
        # Captured before write_joint_targets(), which clears last_error.
        fresh = (np.array([st.last_error is None for st in self.robot.states], dtype=float)
                 if self.robot is not None else np.ones(N_JOINTS))
        ang_vel, proj_gravity = self.imu.get()

        if self.guard is not None:
            reason = self.guard.check(fresh, self.step_idx)
            if reason is not None:
                # No new target is written. shutdown() holds the current position
                # briefly and then disables every motor.
                print(f"[GUARD] STOP: {reason}")
                self.stop_reason = f"torque guard: {reason}"
                self.running = False
                return

        # Hardware convention -> sim convention for the observation.
        joint_pos = self.joint_sign * joint_pos_hw
        joint_vel = self.joint_sign * joint_vel_hw

        obs = self.obs_builder.build(ref_pos, ref_vel, joint_pos, joint_vel, ang_vel, proj_gravity)
        action = self.policy.act(obs, self.time_step)
        self.obs_builder.note_action(action)

        # Sim convention -> hardware convention before clipping and commanding.
        # Joint limits in robot_hardware are in hardware convention; the flipped
        # joints (thighs) have symmetric ranges, so the range is unchanged.
        targets_raw = self.joint_sign * (self.default_joint_pos + self.action_scale * action)
        targets = np.clip(targets_raw, self.clip_lo, self.clip_hi)
        max_delta = self.max_vel * self.dt
        delta = np.clip(targets - self.commanded, -max_delta, max_delta)
        self.commanded = np.clip(self.commanded + delta, self.clip_lo, self.clip_hi)

        sent = self.commanded
        guard_clipped = np.zeros(N_JOINTS)
        cmd_torque_est = np.full(N_JOINTS, np.nan)
        if self.robot is not None:
            if self.mode == "hold" and self.hold_target is not None:
                # Full read/write CAN path at the real rate, but the command is
                # frozen at the startup pose: exercises timing and feedback
                # without commanding any motion.
                sent = self.hold_target
            if self.guard is not None:
                sent, guard_clipped, cmd_torque_est = self.guard.limit(self.robot, sent)
                if self.mode != "hold":
                    # Keep the rate limiter starting from what was actually sent.
                    self.commanded = sent.copy()
            self.robot.write_joint_targets(sent)

        if self.log is not None:
            self._log_step(t_step, ref_pos, ref_vel, obs, action, targets_raw, targets, sent,
                           joint_pos_hw, joint_vel_hw, fresh, ang_vel, proj_gravity,
                           guard_clipped, cmd_torque_est)

        self._print_status(action, proj_gravity, ang_vel)
        self.step_idx += 1
        self.time_step += 1
        if self.time_step >= self.motion_frames:
            if self.loop_motion:
                self.time_step = 0
            else:
                # The reference does not loop: frames beyond the last clamp to it,
                # and the start and end poses differ, so wrapping would be a jump.
                print(f"[MOTION] reached final frame {self.motion_frames - 1}; stopping.")
                self.running = False

    def _log_step(self, t_step, ref_pos, ref_vel, obs, action, targets_raw, targets, sent,
                  joint_pos_hw, joint_vel_hw, fresh, ang_vel, proj_gravity,
                  guard_clipped, cmd_torque_est) -> None:
        """One row per step. Joint-space values are in the HARDWARE convention
        unless named *_sim; motor_* fields are raw motor space (before
        INVERSION_ARRAY and the startup offset)."""
        assert self.log is not None
        nan = np.full(N_JOINTS, np.nan)
        if self.robot is not None:
            states = self.robot.states
            motor = {
                "final_cmd": [st.commanded_joint for st in states],   # after RobotInterface clamp
                "motor_cmd": [st.last_cmd_phys for st in states],
                "motor_pos": [st.position_phys for st in states],
                "motor_vel": [st.velocity_phys for st in states],
                "motor_torque": [st.torque_nm for st in states],
                "motor_temp": [st.temp_c for st in states],
                # Unclamped joint angle: joint_pos is clamped to the limits
                # before it reaches the policy, which hides overshoot.
                # Ankles go through the linkage, so convert with the real mapping.
                "joint_pos_unclamped": [self.robot.motor_physical_to_joint(st, st.position_phys)
                                        for st in states],
            }
        else:
            motor = {k: nan for k in ("final_cmd", "motor_cmd", "motor_pos", "motor_vel",
                                      "motor_torque", "motor_temp", "joint_pos_unclamped")}
        self.log.add(
            t=t_step - self._t0,
            loop_dt=t_step - self._t_prev,
            time_step=self.time_step,
            ref_pos_sim=ref_pos,
            ref_vel_sim=ref_vel,
            obs=obs,
            action=action,
            target_raw=targets_raw,        # sign * (default + scale * action), before clipping
            target_clipped=targets,        # after joint limits / clamps
            commanded=self.commanded,      # after the max_vel rate limit
            sent=sent,                     # what was written (hold target in --hold)
            joint_pos=joint_pos_hw,
            joint_vel=joint_vel_hw,
            feedback_fresh=fresh,          # 1 = this motor replied this cycle
            ang_vel=ang_vel,
            proj_gravity=proj_gravity,
            guard_clipped=guard_clipped,   # 1 = torque guard pulled this target in
            cmd_torque_est=cmd_torque_est, # kp*(target-pos) - kd*vel, motor space, Nm
            **motor,
        )
        self._t_prev = t_step

    def _print_status(self, action, proj_gravity, ang_vel) -> None:
        if self.status_interval is None:
            return
        now = time.time()
        if (now - self._last_status) < self.status_interval:
            return
        self._last_status = now
        msg = (
            f"[t={self.time_step:3d}] |a|max={np.abs(action).max():5.2f} "
            f"cmd=[{self.commanded.min():+.2f},{self.commanded.max():+.2f}] "
            f"g=({proj_gravity[0]:+.2f},{proj_gravity[1]:+.2f},{proj_gravity[2]:+.2f}) "
            f"w=({ang_vel[0]:+.2f},{ang_vel[1]:+.2f},{ang_vel[2]:+.2f})"
        )
        if self.robot is not None:
            msg += f" Tmax={float(np.max(self.robot.temperatures_real())):.0f}C"
        print(msg)

    def run(self, max_steps: int | None = None) -> None:
        next_tick = time.perf_counter()
        t_start = next_tick
        self._t0 = self._t_prev = t_start
        overruns = 0
        while self.running and (self.robot is None or self.robot.connected):
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(next_tick - now)
            else:
                if self.step_idx:
                    overruns += 1
                next_tick = now
            self.step()
            if max_steps is not None and self.step_idx >= max_steps:
                break
            next_tick += self.dt
        if self._fb_cycles:
            print(f"[FEEDBACK] {self._fb_updated / self._fb_cycles:.2f} of {N_JOINTS} motors "
                  f"replied per cycle; {self._fb_short} of {self._fb_cycles} cycles incomplete")
        elapsed = time.perf_counter() - t_start
        if self.step_idx:
            print(f"[TIMING] {self.step_idx} steps in {elapsed:.2f}s "
                  f"= {self.step_idx/elapsed:.1f} Hz achieved (target {self.control_hz:.0f} Hz), "
                  f"{overruns} overran the {self.dt*1000:.1f} ms budget")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(Path(__file__).with_name("run_tracking_config.json")))
    p.add_argument("--policy", default=None, help="Override policy_path from config.")
    p.add_argument("--steps", type=int, default=None)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--offline", action="store_true",
                   help="No CAN at all. IMU and policy only; nothing can move.")
    g.add_argument("--dry-run", action="store_true",
                   help="Connect and ENABLE motors, read state, write no targets. "
                        "NOTE: motors send status frames only in reply to a command, "
                        "so every read times out and the loop runs far below rate.")
    g.add_argument("--hold", action="store_true",
                   help="Full CAN read/write at rate, but command is frozen at the "
                        "startup pose. Validates timing and feedback without motion.")
    p.add_argument("--log-dir", default=None,
                   help="Where to write the per-step .npz log (default: config log_dir, "
                        "else logs/tracking under the repo).")
    p.add_argument("--no-log", action="store_true", help="Do not record a per-step log.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).expanduser().resolve()
    cfg = load_config(config_path)
    if args.policy:
        cfg["policy_path"] = args.policy
    mode = ("offline" if args.offline else
            "dry-run" if args.dry_run else
            "hold" if args.hold else "live")

    if mode == "dry-run":
        print("[WARN] --dry-run ENABLES motor torque: RobotInterface.connect() calls "
              "bus.enable() regardless of dry_run. Only target writes are suppressed.")
    if mode == "hold":
        print("[WARN] --hold energizes motors and commands them to hold their startup "
              "pose. They will become stiff. No trajectory is played.")
    if mode == "live":
        print("[WARN] LIVE mode. The tracking policy walks from step 0; there is no "
              "stand-still command. Support the robot and keep power within reach.")

    ctl = TrackingController(cfg, mode)
    signal.signal(signal.SIGINT, lambda *_: ctl.request_stop())
    signal.signal(signal.SIGTERM, lambda *_: ctl.request_stop())
    try:
        if not ctl.connect():
            raise SystemExit(1)
        if not args.no_log:
            log_dir = Path(args.log_dir or cfg.get("log_dir") or "logs/tracking").expanduser()
            if not log_dir.is_absolute():
                log_dir = REPO_ROOT / log_dir
            ctl.enable_logging(log_dir, config_path)
        ctl.run(max_steps=args.steps)
    except BaseException as exc:
        if not isinstance(exc, SystemExit):
            ctl.stop_reason = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        ctl.shutdown()


if __name__ == "__main__":
    main()
