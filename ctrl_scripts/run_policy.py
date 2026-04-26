#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Run a Torch policy on the real robot using MuJoCo-compatible observation layout.

Observation layout mirrors `mujoco_env_reference.py`:
  [lin_vel(3), ang_vel_scaled(3), up(3), commands(3),
   act_pos_scaled(10), act_vel_scaled(10), last_action(10), (phase_clock(2))]

Key ordering:
- Real robot joint order == MuJoCo actuator order (left block then right block).
- Policy uses interleaved order.
- Script reorders observations/actions exactly between those two.

Control behavior:
- Non-ankle joints: direct position commands (MIT mode), gain_tuner-style.
- Ankle joints: ankle-angle command mapped through 4-bar linkage to motor angle
  (ankle_fv/ankle_gain_tuner_no_plot style).
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
import torch

from robot_interface import RobotInterface
from run_policy_validation import require_key, validate_config


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from utils.imu_read import RK4DeadReckoner, iter_imu_samples

    IMU_AVAILABLE = True
except Exception:
    IMU_AVAILABLE = False
    iter_imu_samples = None
    RK4DeadReckoner = None


class ImuReader:
    def __init__(self, cfg: dict[str, Any]):
        self.enabled = bool(require_key(cfg, "enabled", "imu"))
        self.source = str(require_key(cfg, "source", "imu"))
        self.port = str(require_key(cfg, "port", "imu"))
        self.baud = int(require_key(cfg, "baud", "imu"))
        self.can_interface = str(cfg.get("can_interface", "socketcan"))
        self.can_channel = str(cfg.get("can_channel", self.port))
        self.can_bitrate = int(cfg.get("can_bitrate", self.baud))
        self.rate_hz = float(require_key(cfg, "rate_hz", "imu"))
        self.include_all = bool(require_key(cfg, "include_all", "imu"))
        self.use_quaternion_up = bool(cfg.get("use_quaternion_up", True))
        self.wait_for_first_sample_s = float(require_key(cfg, "wait_for_first_sample_s", "imu"))
        self.gyro_sign = np.asarray(require_key(cfg, "gyro_sign", "imu"), dtype=float)
        self.accel_sign = np.asarray(require_key(cfg, "accel_sign", "imu"), dtype=float)

        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._has_sample = False

        self._ang_vel_rad_s = np.zeros(3, dtype=float)
        self._up_body = np.array([0.0, 0.0, 1.0], dtype=float)
        self._last_error: str | None = None

    def start(self) -> None:
        if not self.enabled:
            return
        if not IMU_AVAILABLE or iter_imu_samples is None:
            raise RuntimeError("IMU enabled in config, but utils/imu_read.py import failed.")
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="imu_reader")
        self._thread.start()
        if self.wait_for_first_sample_s > 0.0:
            t0 = time.time()
            while (time.time() - t0) < self.wait_for_first_sample_s:
                with self._lock:
                    if self._has_sample:
                        return
                time.sleep(0.01)
            print(f"[IMU] No sample received within {self.wait_for_first_sample_s:.2f}s; continuing with fallback zeros.")

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def get(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            return self._ang_vel_rad_s.copy(), self._up_body.copy()

    def _loop(self) -> None:
        try:
            if iter_imu_samples is None:
                raise RuntimeError("iter_imu_samples is unavailable")

            kwargs: dict[str, Any] = dict(
                source=self.source,
                rate_hz=self.rate_hz,
            )
            if self.source == "can":
                kwargs["can_interface"] = self.can_interface
                kwargs["can_channel"] = self.can_channel
                kwargs["can_bitrate"] = self.can_bitrate
            else:
                kwargs["port"] = self.port
                kwargs["baud"] = self.baud

            if self.use_quaternion_up:
                kwargs["include_all"] = True
                if RK4DeadReckoner is not None:
                    kwargs["integrator"] = RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665))
                gen = iter_imu_samples(**kwargs)
            elif self.include_all:
                kwargs["include_all"] = True
                gen = iter_imu_samples(**kwargs)
            else:
                kwargs["keys"] = ("acc_g", "gyro_dps")
                kwargs["include_all"] = False
                gen = iter_imu_samples(**kwargs)
            for sample in gen:
                if not self._running:
                    break
                gyro_dps = sample.get("gyro_dps")
                if gyro_dps is None:
                    continue
                gyro = np.asarray(gyro_dps, dtype=float) * self.gyro_sign

                # Directly use up_body computed by RK4DeadReckoner
                up = self._up_body
                up_body_from_sample = sample.get("up_body")
                if up_body_from_sample is not None:
                    up = np.asarray(up_body_from_sample, dtype=float)
                ang_vel = np.radians(gyro)

                with self._lock:
                    self._ang_vel_rad_s = ang_vel
                    self._up_body = up
                    self._has_sample = True
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)


class ObservationBuilder:
    def __init__(
        self,
        cfg: dict[str, Any],
        real_joint_order: list[str],
        policy_joint_order: list[str],
        joint_limits_real: np.ndarray,
        dt: float,
    ):
        self.real_joint_order = list(real_joint_order)
        self.policy_joint_order = list(policy_joint_order)
        self.dt = float(dt)

        self.policy_to_real = np.array(
            [self.real_joint_order.index(jn) for jn in self.policy_joint_order],
            dtype=int,
        )
        self.real_to_policy = np.argsort(self.policy_to_real)

        if not np.array_equal(np.arange(len(self.policy_to_real)), np.arange(len(self.policy_to_real))[self.real_to_policy][self.policy_to_real]):
            raise ValueError("invalid reorder mapping (policy <-> real)")

        lower_real = joint_limits_real[:, 0].astype(float)
        upper_real = joint_limits_real[:, 1].astype(float)
        lower_policy = lower_real[self.policy_to_real]
        upper_policy = upper_real[self.policy_to_real]

        soft_factor = float(require_key(cfg, "soft_joint_limit_factor"))
        center_policy = 0.5 * (lower_policy + upper_policy)
        halfspan_policy = 0.5 * (upper_policy - lower_policy)
        soft_halfspan_policy = soft_factor * halfspan_policy

        self.joint_soft_lower_policy = center_policy - soft_halfspan_policy
        self.joint_soft_upper_policy = center_policy + soft_halfspan_policy
        self.joint_soft_lower_real = self.joint_soft_lower_policy[self.real_to_policy]
        self.joint_soft_upper_real = self.joint_soft_upper_policy[self.real_to_policy]

        self.commands = np.asarray(require_key(cfg, "command_obs"), dtype=float).reshape(-1)
        if self.commands.shape != (3,):
            raise ValueError("command_obs must have 3 values: [vx, vy, yaw_rate]")

        self.ang_vel_scale = float(require_key(cfg, "ang_vel_scale"))
        self.dof_vel_scale = float(require_key(cfg, "dof_vel_scale"))

        self.use_phase_obs = bool(require_key(cfg, "use_phase_obs"))
        self.gait_period_s = float(require_key(cfg, "gait_period_s"))
        self.freeze_phase_when_standing = bool(require_key(cfg, "freeze_phase_when_standing"))
        self.stand_phase_lin_threshold = float(require_key(cfg, "stand_phase_lin_threshold"))
        self.stand_phase_yaw_threshold = float(require_key(cfg, "stand_phase_yaw_threshold"))
        self.stand_phase_value = float(require_key(cfg, "stand_phase_value"))

        self.frame_stack = max(1, int(require_key(cfg, "frame_stack")))
        self.stack_frame_major = bool(require_key(cfg, "stack_frame_major"))

        self._last_action_policy = np.zeros(len(policy_joint_order), dtype=float)
        self._step_count = 0

        self.single_obs_size = 3 + 3 + 3 + 3 + len(policy_joint_order) + len(policy_joint_order) + len(policy_joint_order)
        if self.use_phase_obs:
            self.single_obs_size += 2
        self.obs_size = self.single_obs_size * self.frame_stack

        self._stack_buf: np.ndarray | None = None
        if self.frame_stack > 1:
            if self.stack_frame_major:
                self._stack_buf = np.zeros((self.frame_stack, self.single_obs_size), dtype=float)
            else:
                self._stack_buf = np.zeros((self.single_obs_size, self.frame_stack), dtype=float)

    def reset(
        self,
        joint_pos_real: np.ndarray,
        joint_vel_real: np.ndarray,
        ang_vel_body: np.ndarray,
        up_body: np.ndarray,
    ) -> np.ndarray:
        self._step_count = 0
        self._last_action_policy[:] = 0.0
        single = self._single(joint_pos_real, joint_vel_real, ang_vel_body, up_body)
        if self._stack_buf is None:
            return single.copy()
        if self.stack_frame_major:
            self._stack_buf[:, :] = single[None, :]
        else:
            self._stack_buf[:, :] = single[:, None]
        return self._stack_buf.reshape(-1).copy()

    def observe(
        self,
        joint_pos_real: np.ndarray,
        joint_vel_real: np.ndarray,
        ang_vel_body: np.ndarray,
        up_body: np.ndarray,
    ) -> np.ndarray:
        single = self._single(joint_pos_real, joint_vel_real, ang_vel_body, up_body)
        if self._stack_buf is None:
            return single.copy()
        if self.stack_frame_major:
            self._stack_buf = np.roll(self._stack_buf, shift=-1, axis=0)
            self._stack_buf[-1, :] = single
        else:
            self._stack_buf = np.roll(self._stack_buf, shift=-1, axis=1)
            self._stack_buf[:, -1] = single
        return self._stack_buf.reshape(-1).copy()

    def note_action(self, action_policy: np.ndarray) -> None:
        self._last_action_policy = np.asarray(action_policy, dtype=float).reshape(-1).copy()
        self._step_count += 1

    def _is_standing_command(self) -> bool:
        lin_norm = float(np.linalg.norm(self.commands[:2]))
        yaw_abs = abs(float(self.commands[2]))
        return (lin_norm < self.stand_phase_lin_threshold) and (yaw_abs < self.stand_phase_yaw_threshold)

    def _phase_for_observation(self) -> float:
        if self.gait_period_s <= 0.0:
            raise ValueError("gait_period_s must be > 0 when use_phase_obs is enabled")
        t = float(self._step_count) * self.dt
        phase = 2.0 * math.pi * (t / self.gait_period_s)
        if self.freeze_phase_when_standing and self._is_standing_command():
            phase = self.stand_phase_value
        return phase

    def _single(
        self,
        joint_pos_real: np.ndarray,
        joint_vel_real: np.ndarray,
        ang_vel_body: np.ndarray,
        up_body: np.ndarray,
    ) -> np.ndarray:
        joint_pos_policy = np.asarray(joint_pos_real, dtype=float)[self.policy_to_real]
        joint_vel_policy = np.asarray(joint_vel_real, dtype=float)[self.policy_to_real]

        lo = self.joint_soft_lower_policy
        hi = self.joint_soft_upper_policy
        act_pos_scaled = 2.0 * (joint_pos_policy - lo) / (hi - lo + 1e-6) - 1.0
        act_vel_scaled = joint_vel_policy * self.dof_vel_scale

        torso_lin_vel_cmd = np.zeros(3, dtype=float)
        obs = np.concatenate(
            [
                torso_lin_vel_cmd,
                np.asarray(ang_vel_body, dtype=float) * self.ang_vel_scale,
                np.asarray(up_body, dtype=float),
                self.commands.copy(),
                act_pos_scaled,
                act_vel_scaled,
                self._last_action_policy.copy(),
            ]
        )

        if self.use_phase_obs:
            phase = self._phase_for_observation()
            obs = np.concatenate([obs, np.array([math.sin(phase), math.cos(phase)], dtype=float)])

        return obs


class PolicyRunner:
    def __init__(self, policy_path: Path, device: str):
        self.policy_path = policy_path
        self.device = torch.device(device)
        self.model = self._load_model()
        self._action_dim: int | None = None

    def _load_model(self) -> Any:
        try:
            model = torch.jit.load(str(self.policy_path), map_location=self.device)
            if hasattr(model, "eval"):
                model.eval()
            return model
        except Exception as jit_err:
            loaded = torch.load(str(self.policy_path), map_location=self.device)
            if isinstance(loaded, torch.nn.Module):
                loaded.to(self.device)
                loaded.eval()
                return loaded
            if isinstance(loaded, dict):
                for key in ("model", "policy", "actor", "actor_critic"):
                    module = loaded.get(key)
                    if isinstance(module, torch.nn.Module):
                        module.to(self.device)
                        module.eval()
                        return module
            if callable(loaded):
                return loaded
            raise RuntimeError(
                f"Could not load policy from {self.policy_path}. "
                f"torch.jit.load error: {jit_err}"
            )

    def _forward(self, obs: torch.Tensor) -> Any:
        if hasattr(self.model, "act_inference"):
            return self.model.act_inference(obs)
        if hasattr(self.model, "act"):
            try:
                return self.model.act(obs)
            except Exception:
                pass
        try:
            return self.model(obs)
        except Exception:
            return self.model({"obs": obs})

    def _extract_action_tensor(self, output: Any) -> torch.Tensor:
        if torch.is_tensor(output):
            return output
        if isinstance(output, dict):
            for key in ("actions", "action", "mu", "mean", "policy"):
                val = output.get(key)
                if torch.is_tensor(val):
                    return val
            for val in output.values():
                if torch.is_tensor(val):
                    return val
        if isinstance(output, (tuple, list)):
            for item in output:
                if torch.is_tensor(item):
                    return item
                if isinstance(item, dict):
                    for val in item.values():
                        if torch.is_tensor(val):
                            return val
        raise RuntimeError(f"Could not parse action tensor from policy output type {type(output)}")

    def act(self, obs: np.ndarray) -> np.ndarray:
        obs_t = torch.as_tensor(obs, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            out = self._forward(obs_t)
        action_t = self._extract_action_tensor(out)
        action = action_t.detach().float().cpu().numpy().reshape(-1)
        if self._action_dim is None:
            self._action_dim = int(action.shape[0])
        return action.astype(np.float32, copy=False)


class RunPolicyController:
    def __init__(self, cfg: dict[str, Any], policy_path: Path, dry_run: bool = False):
        self.cfg = cfg
        self.dry_run = bool(dry_run)

        self.control_hz = float(require_key(cfg, "control_hz"))
        if self.control_hz <= 0.0:
            raise ValueError("control_hz must be > 0")
        self.dt = 1.0 / self.control_hz

        self.action_scale = float(require_key(cfg, "action_scale"))
        self.policy_action_clip = float(require_key(cfg, "policy_action_clip"))
        self.use_soft_joint_limits = bool(require_key(cfg, "use_soft_joint_limits"))
        self.status_print_hz = float(require_key(cfg, "status_print_hz"))
        self.status_interval = (1.0 / self.status_print_hz) if self.status_print_hz > 0 else None

        self.robot = RobotInterface(cfg, dry_run=self.dry_run)
        imu_cfg = dict(require_key(cfg, "imu"))
        if str(imu_cfg.get("source", "")).lower() == "can":
            imu_cfg["can_interface"] = str(imu_cfg.get("can_interface", "socketcan"))
            imu_cfg["can_channel"] = str(require_key(cfg, "can_channel"))
            imu_cfg["can_bitrate"] = int(require_key(cfg, "bitrate"))
        self.imu = ImuReader(imu_cfg)

        self.obs_builder = ObservationBuilder(
            cfg=cfg,
            real_joint_order=self.robot.real_joint_order,
            policy_joint_order=self.robot.policy_joint_order,
            joint_limits_real=self.robot.joint_limits_real,
            dt=self.dt,
        )
        self.policy = PolicyRunner(policy_path, str(require_key(cfg, "torch_device")))

        self._clip_lower = (
            self.obs_builder.joint_soft_lower_real
            if self.use_soft_joint_limits
            else self.robot.joint_limits_real[:, 0]
        )
        self._clip_upper = (
            self.obs_builder.joint_soft_upper_real
            if self.use_soft_joint_limits
            else self.robot.joint_limits_real[:, 1]
        )

        self.commanded_joint = self.robot.default_joint_pos_real.copy()

        self.running = False
        self.step_idx = 0
        self._last_status_print = 0.0

    def connect(self) -> bool:
        if not self.robot.connect():
            return False
        self.running = True
        self.imu.start()

        joint_pos_real, joint_vel_real = self.robot.joint_vectors_real()
        self.commanded_joint = joint_pos_real.copy()
        ang_vel_body, up_body = self.imu.get()
        self.obs_builder.reset(joint_pos_real, joint_vel_real, ang_vel_body, up_body)

        print(
            f"Run-policy controller ready. Control {self.control_hz:.1f} Hz, "
            f"obs_dim={self.obs_builder.obs_size}, dry_run={self.dry_run}"
        )
        return True

    def request_stop(self) -> None:
        self.running = False

    def run(self, max_steps: int | None = None) -> None:
        next_tick = time.perf_counter()
        while self.running and self.robot.connected:
            now_perf = time.perf_counter()
            if now_perf < next_tick:
                time.sleep(next_tick - now_perf)
            else:
                next_tick = now_perf

            self._control_step()
            self.step_idx += 1
            if (max_steps is not None) and (self.step_idx >= max_steps):
                break
            next_tick += self.dt

    def shutdown(self) -> None:
        self.running = False
        self.imu.stop()
        self.robot.shutdown()
        print("Controller shutdown complete.")

    def _control_step(self) -> None:
        self.robot.read_feedback()
        joint_pos_real, joint_vel_real = self.robot.joint_vectors_real()
        ang_vel_body, up_body = self.imu.get()
        obs = self.obs_builder.observe(joint_pos_real, joint_vel_real, ang_vel_body, up_body)

        action_policy = self.policy.act(obs)
        if action_policy.shape[0] != len(self.robot.policy_joint_order):
            raise RuntimeError(
                f"Policy action size mismatch. Expected {len(self.robot.policy_joint_order)}, got {action_policy.shape[0]}"
            )

        clip = abs(self.policy_action_clip)
        if clip > 0.0:
            action_policy = np.clip(action_policy, -clip, clip)
        action_policy = np.clip(action_policy, -1.0, 1.0)

        action_real = action_policy[self.obs_builder.real_to_policy]
        targets_real = (
            self.robot.default_joint_pos_real
            + action_real * self.action_scale * self.robot.joint_action_scales_real
        )
        targets_real = np.clip(targets_real, self._clip_lower, self._clip_upper)

        max_step = self.robot.max_vel_real * self.dt
        delta = np.clip(targets_real - self.commanded_joint, -max_step, max_step)
        self.commanded_joint = np.clip(self.commanded_joint + delta, self._clip_lower, self._clip_upper)

        self.robot.write_joint_targets(self.commanded_joint)

        self.obs_builder.note_action(action_policy)
        self._maybe_print_status()

    def _maybe_print_status(self) -> None:
        if self.status_interval is None:
            return
        now = time.time()
        if (now - self._last_status_print) < self.status_interval:
            return
        self._last_status_print = now

        max_temp = float(np.max(self.robot.temperatures_real()))
        l_ank = self.robot.get_joint_state("left_ankle_joint")
        r_ank = self.robot.get_joint_state("right_ankle_joint")
        msg = f"[step {self.step_idx}] max_temp={max_temp:.1f}C"
        if l_ank is not None:
            msg += f" | L_ank pos={l_ank.joint_pos:+.3f} cmd={l_ank.commanded_joint:+.3f}"
        if r_ank is not None:
            msg += f" | R_ank pos={r_ank.joint_pos:+.3f} cmd={r_ank.commanded_joint:+.3f}"
        print(msg)


def load_config(config_path: Path) -> dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("Config JSON must be an object")
    validate_config(data)
    return data


def parse_args() -> argparse.Namespace:
    default_cfg = Path(__file__).with_name("run_policy_config.json")
    p = argparse.ArgumentParser(description="Run policy .pt on robot with MuJoCo-compatible obs/action ordering.")
    p.add_argument("--config", type=str, default=str(default_cfg), help="Path to JSON config file.")
    p.add_argument("--policy", type=str, default=None, help="Override policy path from config.")
    p.add_argument("--steps", type=int, default=None, help="Optional max steps, else run until Ctrl+C.")
    p.add_argument("--dry-run", action="store_true", help="Compute policy/actions but do not write CAN commands.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg_path = Path(args.config).expanduser().resolve()
    cfg = load_config(cfg_path)

    policy_path_raw = args.policy if args.policy is not None else require_key(cfg, "default_policy_path")
    policy_path = Path(str(policy_path_raw)).expanduser()
    if not policy_path.is_absolute():
        policy_path = (cfg_path.parent / policy_path).resolve()
    if not policy_path.exists():
        raise SystemExit(f"Policy file not found: {policy_path}")

    controller = RunPolicyController(cfg=cfg, policy_path=policy_path, dry_run=args.dry_run)

    def _signal_handler(_signum=None, _frame=None):
        controller.request_stop()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        if not controller.connect():
            raise SystemExit(1)
        controller.run(max_steps=args.steps)
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
