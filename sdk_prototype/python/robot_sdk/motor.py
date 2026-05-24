"""Motor wrapper and controller using the SDK motor gRPC client.

Provides `Motor` lightweight proxy and `MotorController` which runs a
control loop (ramp, excitation, temp derate) using `MotorGrpcClient`.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional
import warnings

import yaml

from .grpc_client import MotorGrpcClient


@dataclass(frozen=True)
class MotorConfig:
    """Motor metadata compatible with the humanoid motor registry YAML files."""

    joint_name: str
    can_interface: str | None = None
    master_id: int | None = None
    motor_id: int | None = None
    actuator_type: int | None = None
    model: str | None = None
    direction: int | None = None
    min_position: float | None = None
    max_position: float | None = None
    kp: float | None = None
    kd: float | None = None

    @classmethod
    def from_ros2_yaml(
        cls,
        joint_name: str,
        cfg: dict,
    ):
        known_keys = (
            "can_interface",
            "master_id",
            "motor_id",
            "actuator_type",
            "model",
            "direction",
            "min_position",
            "max_position",
            "kp",
            "kd",
        )
        missing = [key for key in known_keys if key not in cfg]
        if missing:
            warnings.warn(
                f"Motor config for '{joint_name}' is missing field(s): {', '.join(missing)}",
                RuntimeWarning,
                stacklevel=2,
            )

        return cls(
            joint_name=joint_name,
            can_interface=str(cfg["can_interface"]) if "can_interface" in cfg else None,
            master_id=int(cfg["master_id"]) if "master_id" in cfg else None,
            motor_id=int(cfg["motor_id"]) if "motor_id" in cfg else None,
            actuator_type=int(cfg["actuator_type"]) if "actuator_type" in cfg else None,
            model=str(cfg["model"]) if "model" in cfg else None,
            direction=int(cfg["direction"]) if "direction" in cfg else None,
            min_position=float(cfg["min_position"]) if "min_position" in cfg else None,
            max_position=float(cfg["max_position"]) if "max_position" in cfg else None,
            kp=float(cfg["kp"]) if "kp" in cfg else None,
            kd=float(cfg["kd"]) if "kd" in cfg else None,
        )


def load_motor_configs_from_yaml(config_path: str | Path) -> Dict[str, MotorConfig]:
    """Load motor configs from a ROS2-style motor registry YAML file."""

    path = Path(config_path)
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if "motor_control_node" in data:
        params = data["motor_control_node"].get("ros__parameters", {})
    else:
        params = data

    motors_cfg = params.get("motors", {}) or {}

    result: Dict[str, MotorConfig] = {}
    for joint_name, cfg in motors_cfg.items():
        result[str(joint_name)] = MotorConfig.from_ros2_yaml(str(joint_name), cfg or {})
    return result


class Motor:
    """Lightweight proxy around `MotorGrpcClient` for simple calls."""

    def __init__(
        self,
        client: MotorGrpcClient | None = None,
        joint_name: str = "",
        motor_config: MotorConfig | None = None,
    ) -> None:
        self.client = client or MotorGrpcClient()
        self.joint_name = str(joint_name)
        self.motor_config: MotorConfig | None = motor_config

    def get_motor_config(self) -> MotorConfig | None:
        """Return this motor's config when available."""

        return self.motor_config

    def _clamp_position(self, position: float) -> float:
        cfg = self.motor_config
        if cfg is None or cfg.min_position is None or cfg.max_position is None:
            return float(position)
        return max(cfg.min_position, min(cfg.max_position, float(position)))

    def _resolve_kp(self, kp: float | None) -> float:
        if kp is not None:
            return float(kp)
        if self.motor_config is not None and self.motor_config.kp is not None:
            return float(self.motor_config.kp)
        raise ValueError(f"Motor '{self.joint_name}' requires kp, but no config default is set")

    def _resolve_kd(self, kd: float | None) -> float:
        if kd is not None:
            return float(kd)
        if self.motor_config is not None and self.motor_config.kd is not None:
            return float(self.motor_config.kd)
        raise ValueError(f"Motor '{self.joint_name}' requires kd, but no config default is set")

    @classmethod
    def from_sdk(
        cls,
        sdk,
        joint_name: str,
        motor_config: MotorConfig | None = None,
    ):
        client = getattr(sdk, "motor_client", None)
        if client is None:
            client = MotorGrpcClient()
        return cls(client=client, joint_name=joint_name, motor_config=motor_config)

    def enable(self):
        return self.client.enable_motors([self.joint_name])

    def disable(self):
        return self.client.disable_motors([self.joint_name])

    def set_velocity(self, velocity: float, acceleration: float | None = None):
        accelerations = [float(acceleration)] if acceleration is not None else None
        return self.client.set_motor_velocity([self.joint_name], [float(velocity)], accelerations)

    def set_position(self, position: float, velocity: float | None = None, kp: float | None = None, kd: float | None = None):
        return self.client.set_motor_position(
            [self.joint_name],
            [self._clamp_position(position)],
            [float(velocity)] if velocity is not None else None,
            [self._resolve_kp(kp)],
            [self._resolve_kd(kd)],
        )

    def set_mit(self, position: float, velocity: float, torque_nm: float | None = None, kp: float | None = None, kd: float | None = None):
        return self.client.set_motor_mit(
            [self.joint_name],
            [self._clamp_position(position)],
            [float(velocity)],
            [float(torque_nm)] if torque_nm is not None else None,
            [self._resolve_kp(kp)],
            [self._resolve_kd(kd)],
        )

    def get_status(self):
        return self.client.get_motor_status([self.joint_name])
