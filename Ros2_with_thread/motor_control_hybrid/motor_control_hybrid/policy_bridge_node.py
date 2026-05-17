#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from motor_control_interfaces.msg import MotorCommand
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


def require_key(cfg: dict[str, Any], key: str, context: str = "config") -> Any:
    if key not in cfg:
        raise KeyError(f"Missing required {context} key: {key}")
    return cfg[key]


def quat_inverse_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by q inverse. q is [w, x, y, z]."""
    w = float(q[0])
    xyz = -np.asarray(q[1:4], dtype=float)
    vec = np.asarray(v, dtype=float)
    t = 2.0 * np.cross(xyz, vec)
    return vec + w * t + np.cross(xyz, t)


class ObservationBuilder:
    def __init__(self, cfg: dict[str, Any], dt: float):
        self.real_joint_order = [str(name) for name in require_key(cfg, "real_joint_order")]
        self.policy_joint_order = [str(name) for name in require_key(cfg, "policy_joint_order")]
        self.dt = float(dt)

        self.policy_to_real = np.asarray(
            [self.real_joint_order.index(name) for name in self.policy_joint_order],
            dtype=int,
        )
        self.real_to_policy = np.argsort(self.policy_to_real)

        self.joint_limits_real = np.asarray(
            [require_key(cfg["joint_limits_rad_by_joint"], name, "joint_limits_rad_by_joint")
             for name in self.real_joint_order],
            dtype=float,
        )
        lower_real = self.joint_limits_real[:, 0]
        upper_real = self.joint_limits_real[:, 1]
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
            raise ValueError("command_obs must be [vx, vy, yaw_rate]")

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

        self._last_action_policy = np.zeros(len(self.policy_joint_order), dtype=float)
        self._step_count = 0
        self.single_obs_size = 3 + 3 + 3 + 3 + 3 * len(self.policy_joint_order)
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

    def _phase_for_observation(self) -> float:
        phase = 2.0 * math.pi * (float(self._step_count) * self.dt / self.gait_period_s)
        lin_norm = float(np.linalg.norm(self.commands[:2]))
        yaw_abs = abs(float(self.commands[2]))
        if (
            self.freeze_phase_when_standing
            and lin_norm < self.stand_phase_lin_threshold
            and yaw_abs < self.stand_phase_yaw_threshold
        ):
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
            obs = np.concatenate([obs, np.asarray([math.sin(phase), math.cos(phase)], dtype=float)])
        return obs


class PolicyRunner:
    def __init__(self, policy_path: Path, device: str):
        try:
            import torch
        except ModuleNotFoundError as exc:
            raise RuntimeError("Policy bridge requires torch in rosenv.") from exc

        self.torch = torch
        self.policy_path = policy_path
        self.device = torch.device(device)
        self.model = self._load_model()

    def _load_model(self) -> Any:
        torch = self.torch
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
                f"Could not load policy from {self.policy_path}. torch.jit.load error: {jit_err}"
            )

    def act(self, obs: np.ndarray) -> np.ndarray:
        torch = self.torch
        obs_t = torch.as_tensor(obs, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            output = self._forward(obs_t)
        action_t = self._extract_action_tensor(output)
        return action_t.detach().float().cpu().numpy().reshape(-1).astype(np.float32, copy=False)

    def _forward(self, obs_t: Any) -> Any:
        if hasattr(self.model, "act_inference"):
            return self.model.act_inference(obs_t)
        if hasattr(self.model, "act"):
            try:
                return self.model.act(obs_t)
            except Exception:
                pass
        try:
            return self.model(obs_t)
        except Exception:
            return self.model({"obs": obs_t})

    def _extract_action_tensor(self, output: Any) -> Any:
        torch = self.torch
        if torch.is_tensor(output):
            return output
        if isinstance(output, dict):
            for key in ("actions", "action", "mu", "mean", "policy"):
                value = output.get(key)
                if torch.is_tensor(value):
                    return value
            for value in output.values():
                if torch.is_tensor(value):
                    return value
        if isinstance(output, (tuple, list)):
            for item in output:
                if torch.is_tensor(item):
                    return item
                if isinstance(item, dict):
                    for value in item.values():
                        if torch.is_tensor(value):
                            return value
        raise RuntimeError(f"Could not parse action tensor from policy output type {type(output)}")


class PolicyBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_bridge_node")

        self.declare_parameter("config_path", "")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("output_topic", "/desired_motor_subset")
        self.declare_parameter("control_rate_hz", 0.0)
        self.declare_parameter("joint_state_timeout_s", 0.25)
        self.declare_parameter("imu_timeout_s", 0.25)
        self.declare_parameter("publish_mode", "motion")

        config_path_param = str(self.get_parameter("config_path").value)
        if not config_path_param:
            raise ValueError("config_path parameter is required")
        config_path = Path(config_path_param).expanduser().resolve()
        self.cfg = self._load_config(config_path)

        policy_param = str(self.get_parameter("policy_path").value)
        policy_path = self._resolve_policy_path(config_path, policy_param)
        if not policy_path.exists():
            raise FileNotFoundError(f"Policy file not found: {policy_path}")

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        if control_rate_hz <= 0.0:
            control_rate_hz = float(require_key(self.cfg, "control_hz"))
        self.dt = 1.0 / control_rate_hz

        self.obs_builder = ObservationBuilder(self.cfg, self.dt)
        self.policy = PolicyRunner(policy_path, str(require_key(self.cfg, "torch_device")))

        self.default_joint_pos_real = np.asarray(
            [require_key(self.cfg["default_joint_pos_real_rad_by_joint"], name, "default_joint_pos_real_rad_by_joint")
             for name in self.obs_builder.real_joint_order],
            dtype=float,
        )
        self.joint_action_scales_real = np.asarray(
            [
                float(self.cfg.get("action_scale_by_joint", {}).get(name, 1.0))
                for name in self.obs_builder.real_joint_order
            ],
            dtype=float,
        )
        self.max_vel_real = np.asarray(
            [
                float(self.cfg.get("max_vel_rad_s_by_joint", {}).get(name, self.cfg["max_vel_rad_s"]))
                for name in self.obs_builder.real_joint_order
            ],
            dtype=float,
        )
        self.kp_real = np.asarray(
            [float(self.cfg.get("kp_by_joint", {}).get(name, self.cfg["kp"]))
             for name in self.obs_builder.real_joint_order],
            dtype=float,
        )
        self.kd_real = np.asarray(
            [float(self.cfg.get("kd_by_joint", {}).get(name, self.cfg["kd"]))
             for name in self.obs_builder.real_joint_order],
            dtype=float,
        )
        self.action_scale = float(require_key(self.cfg, "action_scale"))
        self.policy_action_clip = abs(float(require_key(self.cfg, "policy_action_clip")))
        self.use_soft_joint_limits = bool(require_key(self.cfg, "use_soft_joint_limits"))
        self.clip_lower = (
            self.obs_builder.joint_soft_lower_real
            if self.use_soft_joint_limits
            else self.obs_builder.joint_limits_real[:, 0]
        )
        self.clip_upper = (
            self.obs_builder.joint_soft_upper_real
            if self.use_soft_joint_limits
            else self.obs_builder.joint_limits_real[:, 1]
        )

        self.commanded_joint = self.default_joint_pos_real.copy()
        self.joint_pos_by_name: dict[str, float] = {}
        self.joint_vel_by_name: dict[str, float] = {}
        self.ang_vel_body: np.ndarray | None = None
        self.up_body: np.ndarray | None = None
        self.last_joint_state_time = None
        self.last_imu_time = None
        self.obs_initialized = False
        self.warned_waiting = False

        publish_mode = str(self.get_parameter("publish_mode").value).lower()
        self.output_mode = {
            "position": MotorCommand.MODE_POSITION,
            "motion": MotorCommand.MODE_MOTION,
        }.get(publish_mode)
        if self.output_mode is None:
            raise ValueError("publish_mode must be 'motion' or 'position'")

        self.cmd_pub = self.create_publisher(
            MotorCommand,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_state_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._imu_callback,
            10,
        )
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"Policy bridge ready: policy={policy_path}, obs_dim={self.obs_builder.obs_size}, "
            f"rate={control_rate_hz:.1f} Hz, output={self.get_parameter('output_topic').value}"
        )

    def _load_config(self, config_path: Path) -> dict[str, Any]:
        with config_path.open("r", encoding="utf-8") as file:
            data = json.load(file)
        if not isinstance(data, dict):
            raise ValueError("Policy config must be a JSON object")
        return data

    def _resolve_policy_path(self, config_path: Path, policy_param: str) -> Path:
        raw = policy_param if policy_param else str(require_key(self.cfg, "default_policy_path"))
        path = Path(raw).expanduser()
        if not path.is_absolute():
            path = (config_path.parent / path).resolve()
        return path

    def _joint_state_callback(self, msg: JointState) -> None:
        self.last_joint_state_time = self.get_clock().now()
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.joint_pos_by_name[name] = float(msg.position[index])
            if index < len(msg.velocity):
                self.joint_vel_by_name[name] = float(msg.velocity[index])

    def _imu_callback(self, msg: Imu) -> None:
        self.last_imu_time = self.get_clock().now()
        self.ang_vel_body = np.asarray(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=float,
        )
        q = np.asarray(
            [
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
            ],
            dtype=float,
        )
        if np.linalg.norm(q) < 1e-8:
            self.up_body = np.asarray([0.0, 0.0, 1.0], dtype=float)
        else:
            q = q / np.linalg.norm(q)
            self.up_body = quat_inverse_rotate(q, np.asarray([0.0, 0.0, 1.0], dtype=float))

    def _tick(self) -> None:
        if not self._inputs_ready():
            return

        joint_pos_real = np.asarray(
            [self.joint_pos_by_name[name] for name in self.obs_builder.real_joint_order],
            dtype=float,
        )
        joint_vel_real = np.asarray(
            [self.joint_vel_by_name.get(name, 0.0) for name in self.obs_builder.real_joint_order],
            dtype=float,
        )
        assert self.ang_vel_body is not None
        assert self.up_body is not None

        if not self.obs_initialized:
            self.commanded_joint = joint_pos_real.copy()
            obs = self.obs_builder.reset(joint_pos_real, joint_vel_real, self.ang_vel_body, self.up_body)
            self.obs_initialized = True
        else:
            obs = self.obs_builder.observe(joint_pos_real, joint_vel_real, self.ang_vel_body, self.up_body)

        action_policy = self.policy.act(obs)
        expected = len(self.obs_builder.policy_joint_order)
        if action_policy.shape[0] != expected:
            raise RuntimeError(f"Policy action size mismatch. Expected {expected}, got {action_policy.shape[0]}")

        if self.policy_action_clip > 0.0:
            action_policy = np.clip(action_policy, -self.policy_action_clip, self.policy_action_clip)
        action_policy = np.clip(action_policy, -1.0, 1.0)

        action_real = action_policy[self.obs_builder.real_to_policy]
        targets_real = (
            self.default_joint_pos_real
            + action_real * self.action_scale * self.joint_action_scales_real
        )
        targets_real = np.clip(targets_real, self.clip_lower, self.clip_upper)

        max_step = self.max_vel_real * self.dt
        delta = np.clip(targets_real - self.commanded_joint, -max_step, max_step)
        self.commanded_joint = np.clip(self.commanded_joint + delta, self.clip_lower, self.clip_upper)

        self._publish_targets()
        self.obs_builder.note_action(action_policy)

    def _inputs_ready(self) -> bool:
        now = self.get_clock().now()
        missing = [
            name
            for name in self.obs_builder.real_joint_order
            if name not in self.joint_pos_by_name
        ]
        if missing:
            self._warn_waiting(f"waiting for /joint_states joints: {', '.join(missing)}")
            return False
        if self.ang_vel_body is None or self.up_body is None:
            self._warn_waiting("waiting for /imu")
            return False
        if self.last_joint_state_time is not None:
            timeout = float(self.get_parameter("joint_state_timeout_s").value)
            if timeout > 0.0 and (now - self.last_joint_state_time).nanoseconds * 1e-9 > timeout:
                self._warn_waiting("joint state timeout")
                return False
        if self.last_imu_time is not None:
            timeout = float(self.get_parameter("imu_timeout_s").value)
            if timeout > 0.0 and (now - self.last_imu_time).nanoseconds * 1e-9 > timeout:
                self._warn_waiting("imu timeout")
                return False
        self.warned_waiting = False
        return True

    def _warn_waiting(self, message: str) -> None:
        if not self.warned_waiting:
            self.get_logger().warn(message)
            self.warned_waiting = True

    def _publish_targets(self) -> None:
        msg = MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = list(self.obs_builder.real_joint_order)
        n = len(msg.joint_name)
        msg.mode = [self.output_mode] * n
        msg.position = [float(value) for value in self.commanded_joint]
        msg.velocity = [0.0] * n
        msg.acceleration = [0.0] * n
        msg.torque = [0.0] * n
        msg.kp = [float(value) for value in self.kp_real]
        msg.kd = [float(value) for value in self.kd_real]
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
