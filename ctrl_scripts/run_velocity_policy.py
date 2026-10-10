#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Run the Triton legs velocity policy (39-D actor) from a gamepad, at 50 Hz.

Uses the same CAN path, torque guard, fault/mode/VBUS checks and read-only
logging as sysid_logger.py. The robot interface (joint order, stand pose,
action scale, target slew, clips, gains, observation layout) comes from the
velocity_contract.json saved next to the policy by train_velocity.py, so the
runner always uses the contract the policy was trained with.

Operator flow (gamepad; see run_velocity_config.json for the buttons):
  start          motors enabled, holding the pose they are in
  deadman + A    home slowly to the stand pose, then hold it
  deadman + Start  policy takes over (command from the sticks)
  B (any time)   stop: hold briefly, then disable every motor
Releasing the deadman, or the gamepad going quiet for stale_zero_s, ramps the
command to zero (the policy keeps balancing, standing still). A gamepad quiet
for stale_stop_s stops the run.

Stops on: torque guard, missed replies, status fault bits, a motor leaving run
mode, type-21 fault frames, VBUS floor or sag, tilt over stop_tilt_deg, a
non-finite action, the IMU reader dying, or --max-seconds.

    ./.venv/bin/python ctrl_scripts/run_velocity_policy.py --policy <run>/<run>.onnx --yes --vbus-min 35
    ./.venv/bin/python ctrl_scripts/run_velocity_policy.py --policy <...>.onnx --check   # no CAN, no motion
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import struct
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

from run_tracking_policy import N_JOINTS, load_config  # noqa: E402
from sysid_logger import (  # noqa: E402
    Manifest,
    RunLog,
    SysIdRun,
    kernel_log_since,
    link_state,
    provenance,
    sha256_file,
)

DEFAULT_CONFIG = Path(__file__).with_name("run_velocity_config.json")
HW_CONFIG = Path(__file__).with_name("run_tracking_config.json")
MODES = {"wait": 0, "homing": 1, "stand": 2, "policy": 3}


def smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


class Gamepad:
    """Linux joystick reader (/dev/input/jsN, the kernel js event API). No dependencies."""

    def __init__(self, cfg: dict[str, Any]):
        self.cfg = cfg
        self.device = cfg["device"]
        self.axes: dict[int, float] = {}
        self.buttons: dict[int, int] = {}
        self.last_event = 0.0          # time.monotonic() of the last event
        self.error: str | None = None
        self._lock = threading.Lock()
        self._running = False

    def start(self) -> None:
        fd = os.open(self.device, os.O_RDONLY)
        self._running = True
        threading.Thread(target=self._loop, args=(fd,), daemon=True, name="gamepad").start()

    def stop(self) -> None:
        self._running = False

    def _loop(self, fd: int) -> None:
        try:
            while self._running:
                data = os.read(fd, 8)
                if len(data) < 8:
                    continue
                _t, value, etype, number = struct.unpack("<IhBB", data)
                with self._lock:
                    if etype & 0x02:
                        self.axes[number] = value / 32767.0
                    elif etype & 0x01:
                        self.buttons[number] = value
                    self.last_event = time.monotonic()
        except Exception as exc:
            self.error = str(exc)
        finally:
            os.close(fd)

    def snapshot(self) -> tuple[dict[int, float], dict[int, int], float]:
        with self._lock:
            return dict(self.axes), dict(self.buttons), self.last_event


def shape_axis(x: float, deadzone: float) -> float:
    """Dead zone, then rescale so the output still reaches +/-1."""
    if abs(x) <= deadzone:
        return 0.0
    return math.copysign((abs(x) - deadzone) / (1.0 - deadzone), x)


class VelocityPolicy:
    def __init__(self, path: Path, contract: dict[str, Any]):
        import onnxruntime as ort

        self.session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        ins, outs = self.session.get_inputs(), self.session.get_outputs()
        if len(ins) != 1 or int(ins[0].shape[-1]) != contract["actor_obs_dim"]:
            raise ValueError(f"policy input {[(i.name, i.shape) for i in ins]} is not "
                             f"[1, {contract['actor_obs_dim']}]")
        if int(outs[0].shape[-1]) != N_JOINTS:
            raise ValueError(f"policy output {outs[0].shape} is not [1, {N_JOINTS}]")
        self.in_name, self.out_name = ins[0].name, outs[0].name
        meta = self.session.get_modelmeta().custom_metadata_map
        self.meta = dict(meta)
        problems = []
        if "joint_names" in meta and meta["joint_names"].split(",") != contract["joint_order"]:
            problems.append("joint_names")
        if "default_joint_pos" in meta:
            dflt = np.array([float(v) for v in meta["default_joint_pos"].split(",")])
            stand = np.array([contract["stand_pose_rad"][j] for j in contract["joint_order"]])
            if not np.allclose(dflt, stand, atol=1e-3):
                problems.append(f"default_joint_pos {dflt.round(3).tolist()} vs stand {stand.round(3).tolist()}")
        if "action_scale" in meta:
            scales = [float(v) for v in meta["action_scale"].split(",")]
            if not np.allclose(scales, contract["action_scale"], atol=1e-6):
                problems.append(f"action_scale {scales} vs {contract['action_scale']}")
        if "observation_names" in meta:
            names = meta["observation_names"].split(",")
            if names != [t["name"] for t in contract["actor_obs"]]:
                problems.append(f"observation_names {names}")
        if problems:
            raise ValueError("ONNX metadata disagrees with the contract: " + "; ".join(problems))

    def act(self, obs: np.ndarray) -> np.ndarray:
        out = self.session.run([self.out_name], {self.in_name: obs.astype(np.float32).reshape(1, -1)})
        return out[0][0].astype(float)


class VelocityRun(SysIdRun):
    """sysid_logger's CAN path and checks, with the velocity policy choosing the targets."""

    def __init__(self, cfg, args, session_dir, contract, vcfg, policy, gamepad):
        super().__init__(cfg, args, session_dir, "velocity", None, {})
        self.contract = contract
        self.vcfg = vcfg
        self.policy = policy
        self.gamepad = gamepad
        names = contract["joint_order"]
        if [st.joint_name for st in self.states] != names:
            raise ValueError("robot joint order differs from the contract")
        kp = [st.kp for st in self.states]
        kd = [st.kd for st in self.states]
        if not (np.allclose(kp, contract["kp_motor"]) and np.allclose(kd, contract["kd_motor"])):
            raise ValueError(f"robot gains kp {kp} kd {kd} differ from the contract "
                             f"kp {contract['kp_motor']} kd {contract['kd_motor']}; fix "
                             f"tracking_kp/kd_by_joint in {HW_CONFIG.name}")
        self.sign = np.array([contract["sim_to_hw_joint_sign"].get(j, 1.0) for j in names])
        self.stand_sim = np.array([contract["stand_pose_rad"][j] for j in names])
        self.clip_lo = np.array(contract["target_clip_lo"])
        self.clip_hi = np.array(contract["target_clip_hi"])
        self.scale = float(contract["action_scale"])
        self.max_step = float(contract["target_slew_rad_s"]) * self.dt
        lim = contract["command_limits"]
        self.cmd_lo = np.array([lim["v_right"][0], lim["v_forward"][0], lim["yaw_rate"][0]])
        self.cmd_hi = np.array([lim["v_right"][1], lim["v_forward"][1], lim["yaw_rate"][1]])
        acc = vcfg["command_accel"]
        self.cmd_rate = np.array([acc["v_right"], acc["v_forward"], acc["yaw_rate"]]) * self.dt
        self.mode = "wait"
        self.cmd = np.zeros(3)
        self.last_action = np.zeros(N_JOINTS)
        self.commanded_sim: np.ndarray | None = None
        self.home_from_hw: np.ndarray | None = None
        self.home_steps = 0
        self.home_k = 0
        self._edge: dict[str, bool] = {}

    # --- operator input ------------------------------------------------------

    def _pressed(self, buttons: dict[int, int], name: str) -> bool:
        """Rising edge of a named button."""
        down = bool(buttons.get(int(self.vcfg["gamepad"][f"button_{name}"]), 0))
        if name not in self._edge:
            # First look: a button already held when the runner starts is not a press.
            self._edge[name] = down
            return False
        was = self._edge[name]
        self._edge[name] = down
        return down and not was

    def _operator(self, t_rel: float) -> tuple[np.ndarray, bool, dict]:
        """Desired command from the sticks, plus whether the deadman is held."""
        g = self.vcfg["gamepad"]
        info = {"stale_s": math.inf, "deadman": False, "sticks": [0.0, 0.0, 0.0]}
        if self.gamepad is None:
            return np.zeros(3), False, info
        axes, buttons, last = self.gamepad.snapshot()
        stale = time.monotonic() - last if last > 0 else math.inf
        info["stale_s"] = stale
        if self.gamepad.error:
            self._stop(f"gamepad reader stopped: {self.gamepad.error}")
        if self.mode == "policy" and stale > float(g["stale_stop_s"]):
            self._stop(f"gamepad silent for {stale:.2f} s (stale_stop_s {g['stale_stop_s']})")
        # Update every edge each step, so a button held before the deadman
        # cannot fire the moment the deadman goes down.
        stop_p, home_p, start_p = (self._pressed(buttons, n) for n in ("stop", "home", "start"))
        if stop_p:
            self._stop("stop button")
        deadman = bool(buttons.get(int(g["button_deadman"]), 0)) and stale <= float(g["stale_zero_s"])
        info["deadman"] = deadman
        if deadman and home_p and self.mode == "wait":
            self._begin_homing(t_rel)
        if deadman and start_p and self.mode == "stand":
            self._begin_policy(t_rel)
        dz = float(g["deadzone"])
        sticks = []
        for key in ("lateral", "forward", "yaw"):
            v = shape_axis(axes.get(int(g[f"axis_{key}"]), 0.0), dz)
            sticks.append(-v if g.get(f"invert_{key}", False) else v)
        info["sticks"] = sticks
        if not deadman:
            return np.zeros(3), False, info
        s = np.array(sticks)
        if not self.vcfg.get("lateral_enabled", False):
            s[0] = 0.0
        desired = np.where(s >= 0, s * self.cmd_hi, -s * self.cmd_lo)
        return desired, True, info

    # --- modes -----------------------------------------------------------------

    def _begin_homing(self, t_rel: float) -> None:
        self.home_from_hw = self.robot.joint_vectors_real()[0].copy()
        stand_hw = self.sign * self.stand_sim
        h = self.vcfg["homing"]
        span = float(np.max(np.abs(stand_hw - self.home_from_hw)))
        duration = max(float(h["duration_s"]), 1.5 * span / float(h["max_speed_rad_s"]))
        self.home_steps = max(1, int(round(duration / self.dt)))
        self.home_k = 0
        self.mode = "homing"
        self.log.event(self.step_idx, t_rel, "mode", f"homing to the stand pose over {duration:.1f} s "
                                                     f"(largest move {math.degrees(span):.1f} deg)")

    def _begin_policy(self, t_rel: float) -> None:
        if not self.ctl.imu.enabled:
            self.log.event(self.step_idx, t_rel, "refused", "policy needs the IMU (run without --no-imu)")
            return
        q_hw = self.robot.joint_vectors_real()[0]
        self.commanded_sim = np.clip(self.sign * q_hw, self.clip_lo, self.clip_hi)
        self.last_action = np.zeros(N_JOINTS)
        self.cmd = np.zeros(3)
        self.mode = "policy"
        self.log.event(self.step_idx, t_rel, "mode", "policy active")

    # --- control step ------------------------------------------------------------

    def step(self, t_loop: float, t0: float) -> None:
        t_rel = t_loop - t0
        ctl = self.ctl
        fb = self._drain(t_loop + ctl.feedback_timeout, t_rel)
        if self.robot.safety_tripped:
            self.log.event(self.step_idx, t_rel, "safety", str(self.robot.safety_reason))
            self._stop(str(self.robot.safety_reason))
        self._check_status(fb, t_rel)
        fresh = fb["got"].astype(float)
        q_hw, qd_hw = self.robot.joint_vectors_real()
        ang_vel, proj_gravity = ctl.imu.get()
        if ctl.imu.enabled and ctl.imu.last_error:
            self._stop(f"IMU reader stopped: {ctl.imu.last_error}")
        reason = self.guard.check(fresh, self.step_idx)
        if reason is not None:
            self.log.event(self.step_idx, t_rel, "torque guard", reason)
            self._stop(f"torque guard: {reason}")
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(proj_gravity[2])))))
        if self.mode == "policy" and ctl.imu.enabled and tilt > float(self.vcfg["stop_tilt_deg"]):
            self._stop(f"tilt {tilt:.1f} deg over stop_tilt_deg {self.vcfg['stop_tilt_deg']}")
        if self.args.max_seconds and t_rel > self.args.max_seconds:
            self._stop(f"--max-seconds {self.args.max_seconds} reached")
        desired, deadman, op = self._operator(t_rel)

        nan10 = np.full(N_JOINTS, np.nan)
        extra = {"mode": MODES[self.mode], "cmd": self.cmd.copy(), "cmd_desired": desired,
                 "deadman": float(deadman), "sticks": op["sticks"],
                 "gamepad_stale_s": min(op["stale_s"], 1e3), "tilt_deg": tilt,
                 "obs": np.full(self.contract["actor_obs_dim"], np.nan), "action": nan10,
                 "target_sim": nan10, "target_clipped_sim": nan10}
        if not self.running:
            self._log_row(t_loop, t_rel, fb, None, q_hw, qd_hw, ang_vel, proj_gravity)
            self.log.add(**extra)
            return

        hold = ctl.hold_target.copy()
        if self.mode == "wait":
            target_hw = hold
        elif self.mode == "homing":
            self.home_k += 1
            a = smoothstep(self.home_k / self.home_steps)
            target_hw = self.home_from_hw + a * (self.sign * self.stand_sim - self.home_from_hw)
            if self.home_k >= self.home_steps:
                self.mode = "stand"
                self.log.event(self.step_idx, t_rel, "mode", "standing; deadman + start runs the policy")
        elif self.mode == "stand":
            target_hw = self.sign * self.stand_sim
        else:   # policy
            # Command: rate-limited toward what the operator asks for (zero without the deadman).
            self.cmd = self.cmd + np.clip(desired - self.cmd, -self.cmd_rate, self.cmd_rate)
            q_sim, qd_sim = self.sign * q_hw, self.sign * qd_hw
            obs = np.concatenate([ang_vel, proj_gravity, q_sim - self.stand_sim, qd_sim,
                                  self.last_action, self.cmd])
            action = self.policy.act(obs)
            if not np.all(np.isfinite(action)):
                self._stop(f"policy returned a non-finite action {action}")
                self._log_row(t_loop, t_rel, fb, None, q_hw, qd_hw, ang_vel, proj_gravity)
                self.log.add(**extra)
                return
            self.last_action = action
            target_sim = self.stand_sim + self.scale * action
            clipped = np.clip(target_sim, self.clip_lo, self.clip_hi)
            delta = np.clip(clipped - self.commanded_sim, -self.max_step, self.max_step)
            self.commanded_sim = np.clip(self.commanded_sim + delta, self.clip_lo, self.clip_hi)
            target_hw = self.sign * self.commanded_sim
            extra.update(obs=obs, action=action, target_sim=target_sim, target_clipped_sim=clipped)

        commanded = target_hw.copy()
        sent, guard_clipped, cmd_torque_est = self.guard.limit(self.robot, target_hw)
        if self.mode == "policy":
            self.commanded_sim = self.sign * sent   # restart the slew from what was sent
        self.robot._assert_safe()
        tx_wall, tx_mono_after, tx_ok = self._write(sent, t_rel)
        vbus_req = self._request_vbus()
        targets = {
            "target_raw": target_hw, "target_clipped": target_hw, "commanded": commanded,
            "sent": sent, "guard_clipped": guard_clipped, "cmd_torque_est": cmd_torque_est,
            "trial_offset": 0.0, "tx_wall": tx_wall, "tx_mono_after": tx_mono_after,
            "tx_ok": tx_ok, "vbus_req_motor": vbus_req,
        }
        self._log_row(t_loop, t_rel, fb, targets, q_hw, qd_hw, ang_vel, proj_gravity)
        extra["mode"] = MODES[self.mode]
        self.log.add(**extra)

    def _print_status(self, t: float) -> None:
        rows = self.log.rows
        n_status = rows["n_status"][-1] if rows.get("n_status") else np.nan
        tq = np.abs([st.torque_nm for st in self.states])
        tilt = rows["tilt_deg"][-1] if rows.get("tilt_deg") else np.nan
        msg = (f"[t={t:5.1f}s] {self.mode:<6} cmd fwd {self.cmd[1]:+.2f} m/s yaw {self.cmd[2]:+.2f} "
               f"| replies {n_status:.0f}/10 max|tau| {tq.max():5.2f} Nm (m{self.states[int(tq.argmax())].motor_id}) "
               f"tilt {float(tilt):4.1f} deg Tmax {max(st.temp_c for st in self.states):.0f}C")
        if np.isfinite(self.vbus_last).any():
            msg += f" VBUS {np.nanmedian(self.vbus_last):.2f} V"
        print(msg)


def load_contract(path: Path) -> dict[str, Any]:
    contract = json.loads(path.read_text(encoding="utf-8"))
    if sum(t["dim"] for t in contract["actor_obs"]) != contract["actor_obs_dim"]:
        raise ValueError(f"{path}: actor_obs dims do not add up")
    return contract


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--policy", required=True, type=Path, help="ONNX exported by train_velocity.py")
    p.add_argument("--contract", type=Path, default=None,
                   help="default: velocity_contract.json next to the policy (the one it was trained with)")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    p.add_argument("--hw-config", type=Path, default=HW_CONFIG)
    p.add_argument("--check", action="store_true", help="validate policy, contract, config and gamepad; no CAN")
    p.add_argument("--yes", action="store_true", help="required to enable motors")
    p.add_argument("--session", default=None)
    p.add_argument("--max-seconds", type=float, default=300.0)
    p.add_argument("--no-imu", action="store_true", help="homing/stand tests only; the policy refuses to start")
    p.add_argument("--no-gamepad", action="store_true", help="holds the startup pose only (no homing, no policy)")
    p.add_argument("--no-vbus", action="store_true")
    p.add_argument("--vbus-min", type=float, default=0.0)
    p.add_argument("--vbus-sag", type=float, default=3.0)
    p.add_argument("--no-mode-stop", action="store_true")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    policy_path = args.policy.expanduser().resolve()
    contract_path = (args.contract or policy_path.parent / "velocity_contract.json").expanduser().resolve()
    if not contract_path.is_file():
        raise SystemExit(f"no contract at {contract_path}; pass --contract")
    contract = load_contract(contract_path)
    vcfg = json.loads(args.config.read_text(encoding="utf-8"))
    policy = VelocityPolicy(policy_path, contract)
    print(f"[POLICY] {policy_path.name}  sha256 {sha256_file(policy_path)[:16]}  "
          f"contract {contract['name']} ({contract_path})")
    print(f"[POLICY] obs {contract['actor_obs_dim']} -> actions 10, stand {contract['stand_pose_rad']}, "
          f"scale {contract['action_scale']}, slew {contract['target_slew_rad_s']} rad/s")

    gamepad = None
    if not args.no_gamepad:
        gamepad = Gamepad(vcfg["gamepad"])
        try:
            gamepad.start()
            print(f"[GAMEPAD] reading {gamepad.device}")
        except OSError as exc:
            raise SystemExit(f"[GAMEPAD] cannot open {vcfg['gamepad']['device']}: {exc} "
                             f"(plug it in, or --no-gamepad to only hold the startup pose)")
    if args.check:
        if gamepad is not None:
            print("[CHECK] move the sticks and press buttons for 5 s ...")
            end = time.monotonic() + 5.0
            while time.monotonic() < end:
                axes, buttons, _ = gamepad.snapshot()
                print(f"\r  axes {{{', '.join(f'{k}: {v:+.2f}' for k, v in sorted(axes.items()))}}}  "
                      f"buttons down {[k for k, v in sorted(buttons.items()) if v]}      ", end="")
                time.sleep(0.1)
            print()
        print("[CHECK] policy, contract and config are consistent. Nothing was enabled.")
        return
    if not args.yes:
        raise SystemExit("this enables the motors; re-run with --yes once the robot is supported and "
                         "someone has the E-stop")

    cfg = load_config(args.hw_config.expanduser().resolve())
    if args.no_imu:
        cfg["imu"]["enabled"] = False
    session_dir = Path(args.session or f"logs/velocity/{time.strftime('%Y%m%d_%H%M%S')}").expanduser()
    if not session_dir.is_absolute():
        session_dir = REPO_ROOT / session_dir
    session_dir.mkdir(parents=True, exist_ok=True)
    run = VelocityRun(cfg, args, session_dir, contract, vcfg, policy, gamepad)
    run.describe()
    try:
        input("\nEnables all 10 motors, holding their current pose. Robot supported, E-stop in hand? "
              "Enter to start, Ctrl+C to abort: ")
    except (KeyboardInterrupt, EOFError):
        raise SystemExit("\naborted before anything was enabled")

    prov = provenance(cfg)
    prov["sha256"][str(policy_path)] = sha256_file(policy_path)
    prov["sha256"][str(contract_path)] = sha256_file(contract_path)
    channel = str(cfg["can_channel"])
    link_before = link_state(channel)
    started = time.time()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run.log = RunLog(session_dir / f"velocity_{stamp}.npz", {
        "run": "velocity", "started": stamp, **run.config_record(),
        "policy": str(policy_path), "contract": contract, "velocity_config": vcfg,
        "policy_metadata": policy.meta, "modes": MODES})
    signal.signal(signal.SIGINT, lambda *_: run.request_stop())
    signal.signal(signal.SIGTERM, lambda *_: run.request_stop())
    run.running = True
    try:
        if not run.ctl.connect():
            run.stop_reason = "connect failed"
            raise SystemExit(1)
        print("[READY] holding the startup pose. deadman + A: home to stand; deadman + Start: policy; B: stop")
        run.run(None)
    except BaseException as exc:
        if not isinstance(exc, SystemExit) and run.stop_reason == "completed":
            run.stop_reason = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if gamepad is not None:
            gamepad.stop()
        try:
            run.drain_tail(0.1)
        except Exception:
            pass
        run.ctl.shutdown()
        run.log.meta.update(stop_reason=run.stop_reason, steps=run.step_idx)
        steps = 0
        try:
            steps = run.log.close()
        except Exception as exc:
            print(f"[LOG] failed to write {run.log.path}: {exc}")
        Manifest(session_dir).add_run({
            "run": "velocity", "mode": "velocity", "started": stamp, "file": run.log.path.name,
            "steps": steps, "stop_reason": run.stop_reason, "events": run.log.events,
            "provenance": prov, "config": run.config_record(), "contract": contract,
            "velocity_config": vcfg, "can_link_before": link_before,
            "can_link_after": link_state(channel), "kernel_log": kernel_log_since(started),
        })
        print(f"[RESULT] velocity: {run.stop_reason}")


if __name__ == "__main__":
    main()
