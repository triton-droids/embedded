#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
System-ID and comms logger for the deployed 50 Hz tracking path.

It drives the motors through exactly the deployed command path:
- run_tracking_config.json and load_config
- the joint signs, 0.95 soft limits and joint clamps
- the max_vel_rad_s slew
- TorqueGuard caps and stops
- RobotInterface connect / limits / shutdown

It does this without running a policy. Instead it records what the deployed
runner throws away:
- monotonic and wall time before and after every command frame
- the kernel receive time of every reply (python-can Message.timestamp), giving
  per-motor reply latency and state age
- mode and fault bits from every status frame (ID bits 16-23), plus any type-21
  fault frames
- VBUS, read as parameter 0x701C from one motor per cycle in round-robin, with
  the same type-17 read frame that bus.read() and motor_param_diff.py use
- every received frame, raw

Modes:
    --check              No enable and no transmit. Prints the resolved config,
                         file hashes and CAN link state, listens passively for
                         1 s, checks the IMU, and writes a manifest stub.
    --hold --seconds N   Energises the motors and holds the startup pose at
                         50 Hz. Same as run_tracking_policy.py --hold.
    --trial NAME --yes   One joint follows a small trajectory from
                         sysid_trials.json while the other nine hold. Asks for
                         Enter first.
    --list               Lists the trials. No CAN.

Output, one directory per session (default logs/system_id/<YYYYmmdd_HHMMSS>):
    <run>.npz        raw arrays, one row per control step, plus the raw frames
    manifest.json    one record per run: git state, file hashes, motor IDs and
                     models, gains, limits, guard config, trial spec, stop
                     reason, events, and `ip -s -d link` before and after.
Pass --session DIR to put several runs in the same directory.
Summarise with: ./.venv/bin/python utils/sysid_report.py <session dir>
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import signal
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from run_tracking_policy import N_JOINTS, TrackingController, load_config  # noqa: E402

TRIALS_PATH = Path(__file__).with_name("sysid_trials.json")
DEFAULT_CONFIG = Path(__file__).with_name("run_tracking_config.json")

TYPE_STATUS = 2
TYPE_READ_PARAM = 17
TYPE_FAULT = 21
VBUS_PARAM = 0x701C
MODE_RUN = 2  # status ID bits 22-23: 0 reset, 1 calibration, 2 run

# Status ID bits 16-21, from bus.receive_status_frame, as bit index -> name.
STATUS_FAULT_BITS = {
    0: "undervoltage",
    1: "overcurrent",
    2: "overtemperature",
    3: "magnetic encoder fault",
    4: "stall",
    5: "uncalibrated",
}
STOP_FAULT_MASK = 0x1F  # every bit except "uncalibrated", which is logged only

# Type-21 fault frame, from bus.receive_status_frame.
FAULT_FRAME_BITS = {
    0: "motor overtemperature",
    1: "drive gate fault",
    2: "undervoltage",
    3: "overvoltage",
    7: "encoder uncalibrated",
}

FILES_TO_HASH = [
    "ctrl_scripts/run_tracking_policy.py",
    "ctrl_scripts/run_tracking_config.json",
    "ctrl_scripts/sysid_logger.py",
    "ctrl_scripts/sysid_trials.json",
    "ctrl_scripts/robot_interface.py",
    "robot_hardware.py",
    "robstride_dynamics/bus.py",
    "robstride_dynamics/protocol.py",
    "robstride_dynamics/table.py",
    "utils/sysid_report.py",
]


def physical_leg(motor_id: int) -> str:
    # Joint names are mirrored: left_* (motors 1-5) is the physical right leg.
    return "physical RIGHT leg" if motor_id <= 5 else "physical LEFT leg"


def decode_bits(value: int, names: dict[int, str]) -> list[str]:
    return [name for bit, name in names.items() if (value >> bit) & 1]


# ---------------------------------------------------------------- provenance


def sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_cmd(cmd: list[str], timeout: float = 5.0) -> dict[str, Any]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return {"cmd": " ".join(cmd), "rc": p.returncode, "out": p.stdout, "err": p.stderr.strip()}
    except Exception as exc:
        return {"cmd": " ".join(cmd), "rc": None, "out": "", "err": str(exc)}


def git_state() -> dict[str, Any]:
    head = run_cmd(["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"])
    branch = run_cmd(["git", "-C", str(REPO_ROOT), "rev-parse", "--abbrev-ref", "HEAD"])
    status = run_cmd(["git", "-C", str(REPO_ROOT), "status", "--porcelain"])
    diff = subprocess.run(["git", "-C", str(REPO_ROOT), "diff", "HEAD"], capture_output=True)
    return {
        "head": head["out"].strip(),
        "branch": branch["out"].strip(),
        "dirty_files": [ln for ln in status["out"].splitlines() if ln.strip()],
        "diff_sha256": hashlib.sha256(diff.stdout).hexdigest(),
        "diff_bytes": len(diff.stdout),
    }


def provenance(cfg: dict[str, Any]) -> dict[str, Any]:
    hashes = {rel: sha256_file(REPO_ROOT / rel) for rel in FILES_TO_HASH}
    policy = Path(str(cfg.get("policy_path", ""))).expanduser()
    hashes[str(policy)] = sha256_file(policy)
    return {"host": socket.gethostname(), "git": git_state(), "sha256": hashes}


def link_state(channel: str) -> dict[str, Any]:
    return run_cmd(["ip", "-s", "-d", "link", "show", channel])


def kernel_log_since(epoch: float) -> dict[str, Any]:
    out = run_cmd(["journalctl", "-k", "--no-pager", "-o", "short-precise",
                   f"--since=@{int(epoch) - 2}"], timeout=10.0)
    if out["rc"] != 0 or not out["out"].strip():
        out["fallback"] = run_cmd(["dmesg", "-T", "--level=err,warn,info"], timeout=5.0)
        out["fallback"]["out"] = "\n".join(out["fallback"]["out"].splitlines()[-60:])
    return out


class Manifest:
    """manifest.json for one session directory; every run appends a record."""

    def __init__(self, session_dir: Path):
        self.path = session_dir / "manifest.json"
        if self.path.is_file():
            self.data = json.loads(self.path.read_text(encoding="utf-8"))
        else:
            self.data = {"session": session_dir.name, "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                         "runs": []}

    def add_run(self, record: dict[str, Any]) -> None:
        self.data["runs"].append(record)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(self.data, indent=2, default=str), encoding="utf-8")
        tmp.chmod(0o444)
        tmp.replace(self.path)    # a rename, so it works on the read-only previous copy
        print(f"[MANIFEST] {self.path} ({len(self.data['runs'])} runs)")


# ---------------------------------------------------------------- trials


def load_trials(path: Path = TRIALS_PATH) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    spec = json.loads(path.read_text(encoding="utf-8"))
    trials: dict[str, dict[str, Any]] = {}
    for joint in spec["joints"]:
        for pname, pattern in spec["patterns"].items():
            trials[f"m{int(joint['motor_id'])}_{pname}"] = {
                "motor_id": int(joint["motor_id"]),
                "one_sided": bool(joint.get("one_sided", False)),
                "pattern": pname,
                **pattern,
            }
    return spec, trials


def smoothstep(n: int) -> np.ndarray:
    if n <= 1:
        return np.ones(max(n, 0))
    s = np.linspace(0.0, 1.0, n)
    return s * s * (3.0 - 2.0 * s)


def fade_envelope(t: np.ndarray, duration: float, fade_s: float) -> np.ndarray:
    """Cosine fade in and out, so oscillations start and end at rest."""
    fade_s = max(1e-6, min(fade_s, duration / 4.0))
    env = np.ones_like(t)
    a = t < fade_s
    env[a] = 0.5 - 0.5 * np.cos(math.pi * t[a] / fade_s)
    b = t > duration - fade_s
    env[b] = 0.5 - 0.5 * np.cos(math.pi * (duration - t[b]) / fade_s)
    return env


def trial_offsets(trial: dict[str, Any], dt: float) -> np.ndarray:
    """Offset in rad from the trial's base pose, one sample per control step.
    For one-sided joints the result is a magnitude; the caller applies the sign."""
    kind = trial["kind"]
    if kind == "steps":
        levels = trial["one_sided_levels_deg"] if trial["one_sided"] else trial["levels_deg"]
        n = max(1, int(round(float(trial["hold_s"]) / dt)))
        return np.repeat(np.radians(np.asarray(levels, dtype=float)), n)
    if kind == "sine":
        duration = float(trial["cycles"]) / float(trial["freq_hz"])
        t = np.arange(int(round(duration / dt))) * dt
        phase = 2.0 * math.pi * float(trial["freq_hz"]) * t
    elif kind == "chirp":
        duration = float(trial["duration_s"])
        t = np.arange(int(round(duration / dt))) * dt
        f0, f1 = float(trial["f0_hz"]), float(trial["f1_hz"])
        phase = 2.0 * math.pi * (f0 * t + 0.5 * (f1 - f0) / duration * t * t)
    else:
        raise ValueError(f"unknown trial kind '{kind}'")
    amp = math.radians(float(trial["amp_deg"]))
    env = fade_envelope(t, duration, 0.5)
    if trial["one_sided"]:
        return amp * (1.0 - np.cos(phase)) * env
    return amp * np.sin(phase) * env


def build_trial_targets(
    trial: dict[str, Any], spec: dict[str, Any], hold: float, lo: float, hi: float, dt: float,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Absolute joint targets for the moving joint (hardware convention).

    The base pose is the startup pose pulled inside the deployed clip range. The
    knee rests on its 0 rad limit, which the 0.95 soft limit puts about 3 deg
    inside the range, so the knee first ramps to that boundary. Returns
    (targets, offsets, base, direction) and raises if any target falls outside
    [lo, hi] or the trial is faster or larger than the limits in the JSON.
    """
    base = float(np.clip(hold, lo, hi))
    direction = 1.0
    if trial["one_sided"]:
        direction = 1.0 if 0.5 * (lo + hi) >= base else -1.0
    offsets = trial_offsets(trial, dt) * direction

    max_off = math.radians(float(spec.get("max_offset_deg", 15.0)))
    if np.max(np.abs(offsets)) > max_off + 1e-9:
        raise ValueError(f"offset {math.degrees(np.max(np.abs(offsets))):.1f} deg exceeds "
                         f"max_offset_deg {spec.get('max_offset_deg')}")
    if trial["kind"] != "steps":
        peak_vel = float(np.max(np.abs(np.diff(offsets))) / dt) if len(offsets) > 1 else 0.0
        if peak_vel > float(spec.get("max_peak_vel_rad_s", 0.9)) + 1e-9:
            raise ValueError(f"peak velocity {peak_vel:.2f} rad/s exceeds max_peak_vel_rad_s "
                             f"{spec.get('max_peak_vel_rad_s')}")

    ramp = hold + (base - hold) * smoothstep(int(round(float(spec.get("ramp_s", 1.0)) / dt)))
    settle = np.full(int(round(float(spec.get("settle_s", 1.0)) / dt)), base)
    tail = np.full(int(round(float(spec.get("tail_s", 1.5)) / dt)), base)
    targets = np.concatenate([ramp, settle, base + offsets, tail])
    body = targets[len(ramp):]
    if np.any(body < lo - 1e-9) or np.any(body > hi + 1e-9):
        raise ValueError(f"targets span [{body.min():+.4f}, {body.max():+.4f}] rad, outside the "
                         f"deployed clip range [{lo:+.4f}, {hi:+.4f}]; refusing to clip a trial")
    full_offsets = np.concatenate([np.zeros(len(ramp) + len(settle)), offsets, np.zeros(len(tail))])
    return targets, full_offsets, base, direction


# ---------------------------------------------------------------- run log


class RunLog:
    """One row per control step plus the raw frame streams, written once on close."""

    def __init__(self, path: Path, meta: dict[str, Any]):
        self.path = path
        self.meta = meta
        self.rows: dict[str, list[np.ndarray]] = {}
        self.rx: dict[str, list] = {k: [] for k in ("ts_kernel", "host_wall", "host_mono", "step",
                                                    "arb_id", "extended", "dlc", "data")}
        self.tx: dict[str, list] = {k: [] for k in ("wall_before", "mono_before", "mono_after",
                                                    "step", "type", "motor_id", "ok")}
        self.events: list[dict[str, Any]] = []

    def add(self, **fields: Any) -> None:
        for key, value in fields.items():
            self.rows.setdefault(key, []).append(np.asarray(value, dtype=float))

    def frame_in(self, msg, step: int, host_wall: float, host_mono: float) -> None:
        data = bytes(msg.data)[:8].ljust(8, b"\x00")
        self.rx["ts_kernel"].append(float(msg.timestamp or 0.0))
        self.rx["host_wall"].append(host_wall)
        self.rx["host_mono"].append(host_mono)
        self.rx["step"].append(step)
        self.rx["arb_id"].append(int(msg.arbitration_id))
        self.rx["extended"].append(bool(msg.is_extended_id))
        self.rx["dlc"].append(int(msg.dlc))
        self.rx["data"].append(np.frombuffer(data, dtype=np.uint8))

    def frame_out(self, wall_before: float, mono_before: float, mono_after: float, step: int,
                  ctype: int, motor_id: int, ok: bool) -> None:
        for key, val in zip(self.tx, (wall_before, mono_before, mono_after, step, ctype, motor_id, ok)):
            self.tx[key].append(val)

    def event(self, step: int, t: float, kind: str, detail: str, motor_id: int | None = None) -> None:
        ev = {"step": step, "t": round(t, 4), "kind": kind, "motor_id": motor_id, "detail": detail}
        self.events.append(ev)
        who = f" m{motor_id}" if motor_id is not None else ""
        print(f"[EVENT] step {step} t={t:.3f}s {kind}{who}: {detail}")

    def close(self) -> int:
        arrays = {k: np.stack(v) for k, v in self.rows.items()} if self.rows else {}
        dtypes = {"step": np.int32, "arb_id": np.uint32, "extended": bool, "dlc": np.uint8,
                  "type": np.uint8, "motor_id": np.uint8, "ok": bool}
        for prefix, stream in (("rx_", self.rx), ("tx_", self.tx)):
            for key, vals in stream.items():
                if key == "data":
                    arrays[prefix + key] = (np.stack(vals) if vals else np.zeros((0, 8), np.uint8))
                else:
                    arrays[prefix + key] = np.asarray(vals, dtype=dtypes.get(key, float))
        self.meta["events"] = self.events
        self.path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(self.path, meta=np.array(json.dumps(self.meta, default=str)), **arrays)
        # Read-only, so a stray shell redirect onto this path (e.g. log output pasted
        # back into a terminal) fails instead of emptying the file.
        self.path.chmod(0o444)
        n = len(self.rows.get("t", []))
        print(f"[LOG] {n} steps, {len(self.rx['step'])} rx and {len(self.tx['step'])} tx frames saved to {self.path}")
        return n


# ---------------------------------------------------------------- session


class SysIdRun:
    def __init__(self, cfg: dict[str, Any], args: argparse.Namespace, session_dir: Path,
                 run_name: str, trial: dict[str, Any] | None, trials_spec: dict[str, Any]):
        self.cfg = cfg
        self.args = args
        self.session_dir = session_dir
        self.run_name = run_name
        self.trial = trial
        self.trials_spec = trials_spec

        # The deployed controller supplies the limits, signs, slew, guard, IMU and
        # RobotInterface. Its policy is loaded but never run.
        self.ctl = TrackingController(cfg, "hold")
        self.robot = self.ctl.robot
        self.guard = self.ctl.guard
        self.states = self.robot.states
        self.dt = self.ctl.dt
        self.idx_by_mid = {st.motor_id: i for i, st in enumerate(self.states)}

        self.running = False
        self.stop_reason = "completed"
        self.step_idx = 0
        self.log: RunLog | None = None

        n = N_JOINTS
        self.last_rx_ts = np.full(n, np.nan)
        self.last_tx_wall = np.full(n, np.nan)
        self.vbus_last = np.full(n, np.nan)
        self.vbus_baseline = np.full(n, np.nan)   # per motor, from the first second of the run
        self._vbus_settle: list[np.ndarray] = []
        self.miss_streak = np.zeros(n, dtype=int)
        self.tx_fail_streak = 0
        self.vbus_next = 0
        self._warned: set[tuple[str, int]] = set()

        self.active_idx: int | None = None
        self.trial_targets: np.ndarray | None = None
        self.trial_offsets: np.ndarray | None = None
        self.trial_info: dict[str, Any] = {}

    # --- setup -------------------------------------------------------------

    def describe(self) -> None:
        ctl = self.ctl
        print(f"\n{'joint':<18} {'id':>2} {'model':<6} {'leg':<18} {'sign':>4} {'dir':>3} "
              f"{'kp':>6} {'kd':>5} {'clip lo/hi (deg)':>18} {'cap Nm':>6}")
        for i, st in enumerate(self.states):
            print(f"{st.joint_name:<18} {st.motor_id:>2} {st.model:<6} {physical_leg(st.motor_id):<18} "
                  f"{ctl.joint_sign[i]:>+4.0f} {st.direction:>+3d} {st.kp:>6.1f} {st.kd:>5.2f} "
                  f"{math.degrees(ctl.clip_lo[i]):>+8.1f}/{math.degrees(ctl.clip_hi[i]):<+8.1f} "
                  f"{self.guard.cap[i]:>6.1f}")
        print(f"slew max_vel_rad_s {ctl.max_vel.tolist()}  control {ctl.control_hz:.0f} Hz  "
              f"feedback timeout {ctl.feedback_timeout * 1000:.0f} ms")

    def config_record(self) -> dict[str, Any]:
        ctl = self.ctl
        return {
            "control_hz": ctl.control_hz,
            "feedback_timeout_s": ctl.feedback_timeout,
            "joint_names": ctl.joint_names,
            "motor_ids": [st.motor_id for st in self.states],
            "motor_models": [st.model for st in self.states],
            "physical_leg": [physical_leg(st.motor_id) for st in self.states],
            "motor_direction": [st.direction for st in self.states],
            "joint_sign": ctl.joint_sign.tolist(),
            "kp": [st.kp for st in self.states],
            "kd": [st.kd for st in self.states],
            "clip_lo": ctl.clip_lo.tolist(),
            "clip_hi": ctl.clip_hi.tolist(),
            "robot_limit_lo": [st.limit_lo for st in self.states],
            "robot_limit_hi": [st.limit_hi for st in self.states],
            "max_vel_rad_s": ctl.max_vel.tolist(),
            "torque_guard": self.guard.cfg,
            "vbus_polling": not self.args.no_vbus,
            "vbus_min_v": self.args.vbus_min,
            "vbus_sag_v": self.args.vbus_sag,
            "stop_on_mode_change": not self.args.no_mode_stop,
            "imu_enabled": bool(self.cfg["imu"]["enabled"]),
            "policy_path": str(self.cfg.get("policy_path")),
        }

    def prepare_trial(self) -> None:
        """After connect: build the moving joint's targets from the real startup pose."""
        assert self.trial is not None and self.ctl.hold_target is not None
        i = self.idx_by_mid[self.trial["motor_id"]]
        targets, offsets, base, direction = build_trial_targets(
            self.trial, self.trials_spec, float(self.ctl.hold_target[i]),
            float(self.ctl.clip_lo[i]), float(self.ctl.clip_hi[i]), self.dt)
        self.active_idx = i
        self.trial_targets = targets
        self.trial_offsets = offsets
        self.trial_info = {
            "active_index": i,
            "joint": self.states[i].joint_name,
            "hold_rad": float(self.ctl.hold_target[i]),
            "base_rad": base,
            "direction": direction,
            "target_min_rad": float(targets.min()),
            "target_max_rad": float(targets.max()),
            "steps": len(targets),
            "duration_s": len(targets) * self.dt,
        }
        st = self.states[i]
        print(f"[TRIAL] {self.run_name}: motor {st.motor_id} {st.joint_name} ({physical_leg(st.motor_id)}) "
              f"hold {math.degrees(self.trial_info['hold_rad']):+.2f} deg, base {math.degrees(base):+.2f} deg, "
              f"targets [{math.degrees(targets.min()):+.2f}, {math.degrees(targets.max()):+.2f}] deg, "
              f"{self.trial_info['duration_s']:.1f} s")

    # --- CAN ---------------------------------------------------------------

    def _drain(self, deadline: float, t_rel: float) -> dict[str, Any]:
        """Collect replies until every motor has sent a status frame or the
        deadline passes, then take whatever else is already queued."""
        from robstride_dynamics.table import (
            MODEL_MIT_POSITION_TABLE,
            MODEL_MIT_TORQUE_TABLE,
            MODEL_MIT_VELOCITY_TABLE,
        )

        handler = self.robot.bus.channel_handler
        n = N_JOINTS
        got = np.zeros(n, dtype=bool)
        rx_ts = np.full(n, np.nan)
        latency = np.full(n, np.nan)
        fault_bits = np.full(n, -1.0)
        mode = np.full(n, -1.0)
        dup = np.zeros(n)
        vbus = np.full(n, np.nan)
        now = time.time()   # one timestamp per cycle, as read_feedback_batched uses

        while True:
            remaining = deadline - time.monotonic()
            if got.all() or remaining <= 0.0:
                remaining = 0.0
            msg = handler.recv(timeout=remaining)
            if msg is None:
                break
            host_wall, host_mono = time.time(), time.monotonic()
            self.log.frame_in(msg, self.step_idx, host_wall, host_mono)
            if msg.is_error_frame:
                self.log.event(self.step_idx, t_rel, "CAN error frame",
                               f"id=0x{msg.arbitration_id:X} data={bytes(msg.data).hex()}")
                continue
            if not msg.is_extended_id:
                self.log.event(self.step_idx, t_rel, "non-extended frame",
                               f"id=0x{msg.arbitration_id:X} data={bytes(msg.data).hex()} "
                               "(a motor announcing itself after a reset?)")
                continue
            arb = int(msg.arbitration_id)
            ctype = (arb >> 24) & 0x1F
            extra = (arb >> 8) & 0xFFFF
            i = self.idx_by_mid.get(extra & 0xFF)
            ts = float(msg.timestamp) if msg.timestamp else host_wall
            if i is None:
                continue
            st = self.states[i]
            data = bytes(msg.data)

            if ctype == TYPE_STATUS and len(data) == 8:
                if got[i]:
                    dup[i] += 1
                got[i] = True
                rx_ts[i] = ts
                self.last_rx_ts[i] = ts
                latency[i] = ts - self.last_tx_wall[i]
                fault_bits[i] = (extra >> 8) & 0x3F
                mode[i] = (extra >> 14) & 0x03
                pos_u16, vel_u16, tq_u16, temp_u16 = struct.unpack(">HHHH", data)
                st.position_phys = (pos_u16 / 0x7FFF - 1.0) * MODEL_MIT_POSITION_TABLE[st.model]
                st.velocity_phys = (vel_u16 / 0x7FFF - 1.0) * MODEL_MIT_VELOCITY_TABLE[st.model]
                st.torque_nm = (tq_u16 / 0x7FFF - 1.0) * MODEL_MIT_TORQUE_TABLE[st.model]
                st.temp_c = temp_u16 * 0.1
                prev_joint_pos = float(st.joint_pos)
                had_prev = st.last_read_time > 0.0
                measured = self.robot._update_joint_from_motor(st, now, initialize=False)
                self.robot._check_joint_state_safety(st, measured, prev_joint_pos, had_prev)
                st.last_error = None
            elif ctype == TYPE_READ_PARAM and len(data) == 8:
                param = struct.unpack("<H", data[0:2])[0]
                if param == VBUS_PARAM:
                    vbus[i] = struct.unpack("<f", data[4:8])[0]
                    self.vbus_last[i] = vbus[i]
            elif ctype == TYPE_FAULT:
                fault_value, warning_value = struct.unpack("<LL", data.ljust(8, b"\x00"))
                names = decode_bits(fault_value, FAULT_FRAME_BITS)
                if (warning_value >> 14) & 1:
                    names.append("stall current")
                self.log.event(self.step_idx, t_rel, "fault frame",
                               f"type 21 fault=0x{fault_value:08X} warning=0x{warning_value:08X} "
                               f"{names or ['(no known bits)']}", st.motor_id)
                self._stop(f"motor {st.motor_id} ({st.joint_name}) sent a type-21 fault frame: {names}")

        for i, st in enumerate(self.states):
            if not got[i]:
                st.last_error = "no status frame this cycle"
        return {"got": got, "rx_ts": rx_ts, "latency": latency, "fault_bits": fault_bits,
                "mode": mode, "dup": dup, "vbus": vbus}

    def _write(self, sent: np.ndarray, t_rel: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """write_joint_targets with a timestamp around every frame."""
        bus = self.robot.bus
        n = N_JOINTS
        wall_before = np.full(n, np.nan)
        mono_after = np.full(n, np.nan)
        tx_ok = np.zeros(n)
        failures = []
        for i, st in enumerate(self.states):
            joint_cmd = min(max(float(sent[i]), st.limit_lo), st.limit_hi)
            phys = self.robot.joint_to_motor_physical(st, joint_cmd)
            wb, mb = time.time(), time.monotonic()
            ok = True
            try:
                bus.write_operation_frame(st.motor_name, phys, st.kp, st.kd, 0.0, 0.0)
                st.commanded_joint = joint_cmd
                st.last_cmd_phys = phys
            except Exception as exc:
                ok = False
                failures.append(f"m{st.motor_id}: {exc}")
            ma = time.monotonic()
            self.log.frame_out(wb, mb, ma, self.step_idx, 1, st.motor_id, ok)
            wall_before[i], mono_after[i], tx_ok[i] = wb, ma, float(ok)
            if ok:
                self.last_tx_wall[i] = wb
        if failures:
            self.tx_fail_streak += 1
            self.log.event(self.step_idx, t_rel, "transmit error", "; ".join(failures))
            if self.tx_fail_streak >= 3:
                self._stop(f"CAN transmit failed {self.tx_fail_streak} cycles in a row: {failures[0]}")
        else:
            self.tx_fail_streak = 0
        return wall_before, mono_after, tx_ok

    def _request_vbus(self) -> float:
        """Round-robin VBUS read: the same type-17 frame bus.read() sends."""
        if self.args.no_vbus:
            return 0.0
        st = self.states[self.vbus_next % N_JOINTS]
        self.vbus_next += 1
        bus = self.robot.bus
        wb, mb = time.time(), time.monotonic()
        ok = True
        try:
            bus.transmit(17, bus.host_id, st.motor_id, struct.pack("<HHL", VBUS_PARAM, 0x00, 0x00))
        except Exception:
            ok = False
        self.log.frame_out(wb, mb, time.monotonic(), self.step_idx, 17, st.motor_id, ok)
        return float(st.motor_id)

    # --- control loop ------------------------------------------------------

    def _stop(self, reason: str) -> None:
        if self.running:
            print(f"[STOP] {reason}")
            self.stop_reason = reason
        self.running = False

    def request_stop(self) -> None:
        self._stop("stop requested (Ctrl+C / SIGTERM)")

    def _check_status(self, fb: dict[str, Any], t_rel: float) -> None:
        for i, st in enumerate(self.states):
            if not fb["got"][i]:
                continue
            bits = int(fb["fault_bits"][i])
            if bits & STOP_FAULT_MASK:
                names = decode_bits(bits & STOP_FAULT_MASK, STATUS_FAULT_BITS)
                self.log.event(self.step_idx, t_rel, "status fault bits", f"0x{bits:02X} {names}", st.motor_id)
                self._stop(f"motor {st.motor_id} ({st.joint_name}) status fault: {names}")
            elif bits and ("uncal", i) not in self._warned:
                self._warned.add(("uncal", i))
                self.log.event(self.step_idx, t_rel, "status flag", f"0x{bits:02X} uncalibrated", st.motor_id)
            md = int(fb["mode"][i])
            if md != MODE_RUN:
                key = ("mode", i)
                if key not in self._warned:
                    self._warned.add(key)
                    self.log.event(self.step_idx, t_rel, "mode change",
                                   f"status mode {md} (expected {MODE_RUN} = run); "
                                   f"torque {st.torque_nm:+.2f} Nm", st.motor_id)
                if not self.args.no_mode_stop:
                    self._stop(f"motor {st.motor_id} ({st.joint_name}) left run mode (mode {md})")

        self._check_vbus(fb["vbus"], t_rel)

        missed = ~fb["got"]
        if self.step_idx > 0:
            if missed.all():
                self.log.event(self.step_idx, t_rel, "bus-wide miss", "no status frame from any motor")
            else:
                for i in np.flatnonzero(missed & (self.miss_streak == 0)):
                    self.log.event(self.step_idx, t_rel, "missed reply", "first miss of a streak",
                                   self.states[i].motor_id)
            self.miss_streak = np.where(missed, self.miss_streak + 1, 0)

    def _check_vbus(self, vbus: np.ndarray, t_rel: float) -> None:
        """Stop below an absolute floor, or on a sag relative to this run's own
        idle reading. The motors run from a battery: each motor's ADC reads a
        little differently, so the sag check compares a motor only to itself."""
        if not np.isfinite(vbus).any():
            return
        settle_steps = int(round(self.ctl.control_hz))
        if self.step_idx < settle_steps:
            self._vbus_settle.append(vbus)
            return
        if np.isnan(self.vbus_baseline).all() and self._vbus_settle:
            with np.errstate(all="ignore"):
                self.vbus_baseline = np.nanmedian(np.stack(self._vbus_settle), axis=0)
        # The floor is about the pack, so use the median over motors: the RS-02
        # ankles read about 0.8 V below the other motors on the same bus.
        known = self.vbus_last[np.isfinite(self.vbus_last)]
        if self.args.vbus_min > 0 and known.size >= N_JOINTS // 2:
            pack = float(np.median(known))
            if pack < self.args.vbus_min:
                self.log.event(self.step_idx, t_rel, "VBUS floor",
                               f"median {pack:.2f} V < {self.args.vbus_min:.2f} V")
                self._stop(f"battery at {pack:.2f} V (median over motors) is below the floor "
                           f"{self.args.vbus_min:.2f} V (charge the battery)")
        for i in np.flatnonzero(np.isfinite(vbus)):
            st, v = self.states[i], float(vbus[i])
            base = self.vbus_baseline[i]
            if self.args.vbus_sag > 0 and np.isfinite(base) and base - v > self.args.vbus_sag:
                self.log.event(self.step_idx, t_rel, "VBUS sag",
                               f"{v:.2f} V, {base - v:.2f} V below its idle {base:.2f} V", st.motor_id)
                self._stop(f"VBUS at motor {st.motor_id} sagged {base - v:.2f} V under load "
                           f"(limit {self.args.vbus_sag:.1f} V)")

    def step(self, t_loop: float, t0: float) -> None:
        t_rel = t_loop - t0
        ctl = self.ctl
        fb = self._drain(t_loop + ctl.feedback_timeout, t_rel)
        if self.robot.safety_tripped:
            self.log.event(self.step_idx, t_rel, "safety", str(self.robot.safety_reason))
            self._stop(str(self.robot.safety_reason))
        self._check_status(fb, t_rel)
        fresh = fb["got"].astype(float)
        joint_pos_hw, joint_vel_hw = self.robot.joint_vectors_real()
        ang_vel, proj_gravity = ctl.imu.get()
        if ctl.imu.enabled and ctl.imu.last_error:
            self._stop(f"IMU reader stopped: {ctl.imu.last_error}")

        reason = self.guard.check(fresh, self.step_idx)
        if reason is not None:
            self.log.event(self.step_idx, t_rel, "torque guard", reason)
            self._stop(f"torque guard: {reason}")
        if not self.running:
            self._log_row(t_loop, t_rel, fb, None, joint_pos_hw, joint_vel_hw, ang_vel, proj_gravity)
            return

        # Held joints get the startup pose unclipped, as run_tracking_policy.py --hold
        # sends it. The moving joint goes through clip and slew like a policy target.
        target_raw = ctl.hold_target.copy()
        offset = 0.0
        if self.active_idx is not None:
            k = min(self.step_idx, len(self.trial_targets) - 1)
            target_raw[self.active_idx] = self.trial_targets[k]
            offset = float(self.trial_offsets[k])
        target_clipped = np.clip(target_raw, ctl.clip_lo, ctl.clip_hi)
        sent = ctl.hold_target.copy()
        if self.active_idx is not None:
            i = self.active_idx
            max_delta = ctl.max_vel[i] * self.dt
            delta = np.clip(target_clipped[i] - ctl.commanded[i], -max_delta, max_delta)
            ctl.commanded[i] = np.clip(ctl.commanded[i] + delta, ctl.clip_lo[i], ctl.clip_hi[i])
            sent[i] = ctl.commanded[i]
        commanded = sent.copy()
        sent, guard_clipped, cmd_torque_est = self.guard.limit(self.robot, sent)
        if self.active_idx is not None:
            ctl.commanded[self.active_idx] = sent[self.active_idx]

        self.robot._assert_safe()
        tx_wall, tx_mono_after, tx_ok = self._write(sent, t_rel)
        vbus_req = self._request_vbus()
        targets = {
            "target_raw": target_raw, "target_clipped": target_clipped, "commanded": commanded,
            "sent": sent, "guard_clipped": guard_clipped, "cmd_torque_est": cmd_torque_est,
            "trial_offset": offset, "tx_wall": tx_wall, "tx_mono_after": tx_mono_after,
            "tx_ok": tx_ok, "vbus_req_motor": vbus_req,
        }
        self._log_row(t_loop, t_rel, fb, targets, joint_pos_hw, joint_vel_hw, ang_vel, proj_gravity)

        if self.trial_targets is not None and self.step_idx + 1 >= len(self.trial_targets):
            self.running = False   # completed

    def _log_row(self, t_loop, t_rel, fb, targets, joint_pos_hw, joint_vel_hw, ang_vel, proj_gravity) -> None:
        nan = np.full(N_JOINTS, np.nan)
        if targets is None:   # stop cycle: nothing written
            targets = {k: nan for k in ("target_raw", "target_clipped", "commanded", "sent",
                                        "guard_clipped", "cmd_torque_est", "tx_wall",
                                        "tx_mono_after", "tx_ok")}
            targets.update(trial_offset=np.nan, vbus_req_motor=0.0)
        states = self.states
        now_wall = time.time()
        self.log.add(
            t=t_rel,
            t_mono=t_loop,
            loop_dt=t_loop - self._t_prev,
            n_status=float(fb["got"].sum()),
            feedback_fresh=fb["got"].astype(float),   # 1 = status frame this cycle
            rx_ts=fb["rx_ts"],                        # kernel receive time (epoch s)
            reply_latency=fb["latency"],              # rx_ts minus the previous command's tx time
            state_age=now_wall - self.last_rx_ts,     # age of the newest status at this step
            status_fault_bits=fb["fault_bits"],       # ID bits 16-21, -1 = no reply
            status_mode=fb["mode"],                   # ID bits 22-23, 2 = run
            status_dup=fb["dup"],
            vbus=fb["vbus"],                          # NaN except motors whose reply arrived this cycle
            vbus_last=self.vbus_last.copy(),
            joint_pos=joint_pos_hw,
            joint_vel=joint_vel_hw,
            joint_pos_unclamped=[self.robot.motor_physical_to_joint(st, st.position_phys) for st in states],
            motor_pos=[st.position_phys for st in states],
            motor_vel=[st.velocity_phys for st in states],
            motor_torque=[st.torque_nm for st in states],
            motor_temp=[st.temp_c for st in states],
            final_cmd=[st.commanded_joint for st in states],
            motor_cmd=[st.last_cmd_phys for st in states],
            ang_vel=ang_vel,
            proj_gravity=proj_gravity,
            compute_s=time.monotonic() - t_loop,
            **targets,
        )
        self._t_prev = t_loop

    def run(self, max_steps: int | None) -> None:
        ctl = self.ctl
        t0 = time.monotonic()
        self._t_prev = t0
        next_tick = t0
        overruns = 0
        last_status = 0.0
        while self.running and self.robot.connected:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            else:
                if self.step_idx:
                    overruns += 1
                next_tick = now
            t_loop = time.monotonic()
            self.step(t_loop, t0)
            self.step_idx += 1
            if max_steps is not None and self.step_idx >= max_steps:
                break
            next_tick += self.dt
            if t_loop - last_status >= 0.5:
                last_status = t_loop
                self._print_status(t_loop - t0)
        elapsed = time.monotonic() - t0
        if self.step_idx:
            print(f"[TIMING] {self.step_idx} steps in {elapsed:.2f}s = {self.step_idx / elapsed:.1f} Hz "
                  f"(target {ctl.control_hz:.0f}), {overruns} overran the {self.dt * 1000:.0f} ms budget")

    def _print_status(self, t: float) -> None:
        rows = self.log.rows
        n_status = rows["n_status"][-1] if rows.get("n_status") else np.nan
        tq = np.abs([st.torque_nm for st in self.states])
        msg = (f"[t={t:5.1f}s] replies {n_status:.0f}/10  max|tau| {tq.max():5.2f} Nm "
               f"(m{self.states[int(tq.argmax())].motor_id})  "
               f"Tmax {max(st.temp_c for st in self.states):.0f}C")
        if np.isfinite(self.vbus_last).any():
            msg += f"  VBUS {np.nanmin(self.vbus_last):.2f}-{np.nanmax(self.vbus_last):.2f} V"
        if self.active_idx is not None:
            st = self.states[self.active_idx]
            msg += (f"  m{st.motor_id} cmd {math.degrees(st.commanded_joint):+6.2f} "
                    f"pos {math.degrees(st.joint_pos):+6.2f} deg")
        print(msg)

    def drain_tail(self, seconds: float) -> None:
        """Keep recording frames for a moment after the last command, to catch
        late fault frames or a motor announcing a reset."""
        if self.robot.bus is None or self.log is None:
            return
        end = time.monotonic() + seconds
        handler = self.robot.bus.channel_handler
        while time.monotonic() < end:
            msg = handler.recv(timeout=max(0.0, end - time.monotonic()))
            if msg is None:
                break
            self.log.frame_in(msg, self.step_idx, time.time(), time.monotonic())


# ---------------------------------------------------------------- modes


def passive_listen(channel: str, seconds: float) -> dict[str, Any]:
    """Open the CAN socket read-only and count frames. Transmits nothing."""
    import can

    out: dict[str, Any] = {"channel": channel, "seconds": seconds, "frames": 0, "ids": {}}
    try:
        bus = can.Bus(interface="socketcan", channel=channel)
    except Exception as exc:
        out["error"] = str(exc)
        return out
    try:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = bus.recv(timeout=max(0.0, end - time.monotonic()))
            if msg is None:
                break
            out["frames"] += 1
            key = f"0x{msg.arbitration_id:08X}"
            out["ids"][key] = out["ids"].get(key, 0) + 1
    finally:
        bus.shutdown()
    return out


def parse_link(text: str) -> str:
    keys = ("state UP", "state DOWN", "ERROR-ACTIVE", "ERROR-WARNING", "ERROR-PASSIVE", "BUS-OFF")
    found = [k for k in keys if k in text]
    berr = [ln.strip() for ln in text.splitlines() if "berr-counter" in ln or "re-started" in ln]
    return " ".join(found) + ("  " + " | ".join(berr) if berr else "")


def mode_check(cfg: dict[str, Any], args: argparse.Namespace, session_dir: Path) -> None:
    _, trials = load_trials()
    run = SysIdRun(cfg, args, session_dir, "check", None, {})
    run.describe()
    prov = provenance(cfg)
    print("\n[HASH] sha256")
    for path, digest in prov["sha256"].items():
        print(f"  {digest[:16] if digest else 'MISSING         '}  {path}")
    git = prov["git"]
    print(f"[GIT] {git['branch']} @ {git['head'][:10]}, {len(git['dirty_files'])} dirty/untracked entries, "
          f"diff sha256 {git['diff_sha256'][:16]}")

    channel = str(cfg["can_channel"])
    link = link_state(channel)
    print(f"\n[CAN] {channel}: {parse_link(link['out']) or link['err']}")
    listen = passive_listen(channel, 1.0)
    if "error" in listen:
        print(f"[CAN] could not open {channel}: {listen['error']}")
    else:
        print(f"[CAN] listened 1.0 s without transmitting: {listen['frames']} frames"
              + (f" {listen['ids']}" if listen["frames"] else " (expected: motors only reply to commands)"))

    imu_result: dict[str, Any] = {"enabled": bool(cfg["imu"]["enabled"])}
    if cfg["imu"]["enabled"]:
        try:
            run.ctl.imu.start()
            g_samples, w_samples = [], []
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                w, g = run.ctl.imu.get()
                w_samples.append(w)
                g_samples.append(g)
                time.sleep(0.02)
            g_mean, w_std = np.mean(g_samples, axis=0), np.std(w_samples, axis=0)
            imu_result.update(gravity=g_mean.tolist(), gyro_mean=np.mean(w_samples, axis=0).tolist(),
                              gyro_std=w_std.tolist())
            print(f"[IMU] projected gravity ({g_mean[0]:+.3f}, {g_mean[1]:+.3f}, {g_mean[2]:+.3f}) "
                  f"(hanging upright: about (0, 0, -1)); gyro mean "
                  f"{np.degrees(np.mean(w_samples, axis=0)).round(2).tolist()} deg/s")
        except Exception as exc:
            imu_result["error"] = str(exc)
            print(f"[IMU] {exc}")
        finally:
            run.ctl.imu.stop()

    print(f"\n[TRIALS] {len(trials)} defined in {TRIALS_PATH.name} (list them with --list)")
    Manifest(session_dir).add_run({
        "run": "check", "mode": "check", "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        "provenance": prov, "config": run.config_record(), "can_link": link,
        "passive_listen": listen, "imu": imu_result, "stop_reason": "check only, nothing enabled",
    })


def mode_list() -> None:
    spec, trials = load_trials()
    dt = 0.02
    print(f"{'trial':<14} {'joint':<18} {'leg':<18} {'kind':<6} {'tag':<8} {'span deg':>9} {'sec':>5}")
    from robot_hardware import JOINT_NAME_BY_ID
    for name, tr in trials.items():
        off = trial_offsets(tr, dt)
        dur = len(off) * dt + float(spec["ramp_s"]) + float(spec["settle_s"]) + float(spec["tail_s"])
        span = f"{math.degrees(off.min()):+.0f}..{math.degrees(off.max()):+.0f}"
        if tr["one_sided"]:
            span = f"0..{math.degrees(off.max()):.0f} in"
        print(f"{name:<14} {JOINT_NAME_BY_ID[tr['motor_id']]:<18} {physical_leg(tr['motor_id']):<18} "
              f"{tr['kind']:<6} {tr['tag']:<8} {span:>9} {dur:>5.1f}")


def mode_run(cfg: dict[str, Any], args: argparse.Namespace, session_dir: Path) -> None:
    spec, trials = load_trials()
    trial = None
    if args.trial:
        if args.trial not in trials:
            raise SystemExit(f"unknown trial '{args.trial}'. Run with --list.")
        trial = trials[args.trial]
        run_name = args.trial
        if not args.yes:
            raise SystemExit("--trial moves a joint. Re-run with --yes once the robot is on the gantry "
                             "and someone has the E-stop.")
    else:
        run_name = f"hold_{args.seconds:g}s"

    run = SysIdRun(cfg, args, session_dir, run_name, trial, spec)
    run.describe()
    if trial is not None:
        st = run.states[run.idx_by_mid[trial["motor_id"]]]
        print(f"\n[TRIAL] {run_name}: motor {st.motor_id} {st.joint_name} = {physical_leg(st.motor_id)}, "
              f"{trial['kind']} ({trial['tag']}). The other nine joints hold their startup pose.")
    print("\nThis ENABLES all 10 motors and holds their current pose"
          + (" while one joint moves." if trial else "."))
    try:
        input("Robot on the gantry, E-stop in hand? Press Enter to start, Ctrl+C to abort: ")
    except (KeyboardInterrupt, EOFError):
        raise SystemExit("\naborted before anything was enabled")

    prov = provenance(cfg)
    channel = str(cfg["can_channel"])
    link_before = link_state(channel)
    started_epoch = time.time()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run.log = RunLog(session_dir / f"{run_name}_{stamp}.npz", {"run": run_name, "started": stamp,
                                                               **run.config_record()})
    signal.signal(signal.SIGINT, lambda *_: run.request_stop())
    signal.signal(signal.SIGTERM, lambda *_: run.request_stop())
    max_steps = None if trial else int(round(args.seconds * run.ctl.control_hz))
    run.running = True   # cleared by Ctrl+C, even while connect() is enabling motors
    try:
        if not run.ctl.connect():
            run.stop_reason = "connect failed"
            raise SystemExit(1)
        if trial is not None:
            try:
                run.prepare_trial()
            except ValueError as exc:
                run.stop_reason = f"trial refused: {exc}"
                raise SystemExit(f"[TRIAL] refused, nothing moved: {exc}")
        run.run(max_steps)
    except BaseException as exc:
        if not isinstance(exc, SystemExit) and run.stop_reason == "completed":
            run.stop_reason = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            run.drain_tail(0.1)
        except Exception:
            pass
        run.ctl.shutdown()
        run.log.meta.update(stop_reason=run.stop_reason, steps=run.step_idx, trial=trial,
                            trial_info=run.trial_info)
        steps = 0
        try:
            steps = run.log.close()
        except Exception as exc:
            print(f"[LOG] failed to write {run.log.path}: {exc}")
        Manifest(session_dir).add_run({
            "run": run_name, "mode": "trial" if trial else "hold", "started": stamp,
            "file": run.log.path.name, "steps": steps, "stop_reason": run.stop_reason,
            "trial": trial, "trial_info": run.trial_info, "events": run.log.events,
            "provenance": prov, "config": run.config_record(),
            "can_link_before": link_before, "can_link_after": link_state(channel),
            "kernel_log": kernel_log_since(started_epoch),
        })
        print(f"[RESULT] {run_name}: {run.stop_reason}")
        print(f"[NEXT] ./.venv/bin/python utils/sysid_report.py {session_dir}")
    if run.stop_reason != "completed":
        raise SystemExit(2)   # lets a shell loop of trials stop at the first one that did not complete


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true", help="No enable, no transmit: config, hashes, link, IMU.")
    g.add_argument("--hold", action="store_true", help="Enable and hold the startup pose.")
    g.add_argument("--trial", default=None, help="Trial name from sysid_trials.json (see --list).")
    g.add_argument("--list", action="store_true", help="List trials and exit.")
    p.add_argument("--seconds", type=float, default=10.0, help="Hold duration (default 10).")
    p.add_argument("--yes", action="store_true", help="Required for --trial.")
    p.add_argument("--session", default=None,
                   help="Session directory (default logs/system_id/<timestamp>). Reuse it to group runs.")
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    p.add_argument("--no-imu", action="store_true", help="Run without the IMU (gravity/gyro logged as defaults).")
    p.add_argument("--no-vbus", action="store_true", help="Do not poll VBUS (type-17 reads).")
    p.add_argument("--vbus-min", type=float, default=0.0,
                   help="Stop if any motor reports VBUS below this (V). 0 = off. Set it from the "
                        "battery's cell count, e.g. 10S Li-ion: 35.0 (3.5 V/cell).")
    p.add_argument("--vbus-sag", type=float, default=3.0,
                   help="Stop if a motor's VBUS drops this many volts below its own idle reading "
                        "from the first second of the run (default 3.0). 0 = off.")
    p.add_argument("--no-mode-stop", action="store_true",
                   help="Log, but do not stop on, a motor leaving run mode.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if args.list:
        mode_list()
        return
    cfg = load_config(Path(args.config).expanduser().resolve())
    if args.no_imu:
        cfg["imu"]["enabled"] = False
    session_dir = Path(args.session or f"logs/system_id/{time.strftime('%Y%m%d_%H%M%S')}").expanduser()
    if not session_dir.is_absolute():
        session_dir = REPO_ROOT / session_dir
    session_dir.mkdir(parents=True, exist_ok=True)
    print(f"[SESSION] {session_dir}")
    if args.check:
        mode_check(cfg, args, session_dir)
    else:
        mode_run(cfg, args, session_dir)


if __name__ == "__main__":
    main()
