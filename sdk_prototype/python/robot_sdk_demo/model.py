from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
import threading
import time
from typing import Any


MODE_IDLE = "idle"
MODE_MANUAL = "manual"
MODE_VELOCITY = "velocity"
MODE_POLICY = "policy"
MODE_ESTOP = "estop"


@dataclass
class VelocityCommand:
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    wz_radps: float = 0.0
    timeout_s: float = 0.25


@dataclass
class JointState:
    name: str
    position_rad: float = 0.0
    velocity_radps: float = 0.0
    effort_nm: float = 0.0


@dataclass
class ImuState:
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0


@dataclass
class RobotStatus:
    enabled: bool = False
    mode: str = MODE_IDLE
    health: str = "ok"
    active_policy_id: str = ""
    state_sequence: int = 0
    stamp_unix_s: float = field(default_factory=time.time)


@dataclass
class RobotState:
    status: RobotStatus = field(default_factory=RobotStatus)
    commanded_velocity: VelocityCommand = field(default_factory=VelocityCommand)
    joints: list[JointState] = field(
        default_factory=lambda: [
            JointState("left_hip_pitch"),
            JointState("left_knee"),
            JointState("right_hip_pitch"),
            JointState("right_knee"),
        ]
    )
    imu: ImuState = field(default_factory=ImuState)


class RobotSimulator:
    """In-memory stand-in for the internal ROS2 gateway adapter."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = RobotState()
        self._loaded_policies: dict[str, str] = {}
        self._last_tick = time.monotonic()

    def enable_robot(self) -> dict[str, Any]:
        with self._lock:
            self._state.status.enabled = True
            self._state.status.mode = MODE_IDLE
            return self._reply("robot enabled")

    def disable_robot(self) -> dict[str, Any]:
        with self._lock:
            self._state.status.enabled = False
            self._state.status.mode = MODE_IDLE
            self._state.status.active_policy_id = ""
            self._state.commanded_velocity = VelocityCommand()
            return self._reply("robot disabled")

    def set_mode(self, mode: str) -> dict[str, Any]:
        if mode not in {MODE_IDLE, MODE_MANUAL, MODE_VELOCITY, MODE_POLICY, MODE_ESTOP}:
            return self._reply(f"unsupported mode: {mode}", accepted=False)
        with self._lock:
            if not self._state.status.enabled and mode not in {MODE_IDLE, MODE_ESTOP}:
                return self._reply("robot must be enabled before entering active modes", accepted=False)
            self._state.status.mode = mode
            return self._reply(f"mode set to {mode}")

    def load_policy(self, policy_id: str, uri: str) -> dict[str, Any]:
        if not policy_id or not uri:
            return self._reply("policy_id and uri are required", accepted=False)
        with self._lock:
            self._loaded_policies[policy_id] = uri
            return self._reply(f"loaded policy {policy_id}")

    def start_policy(self, policy_id: str) -> dict[str, Any]:
        with self._lock:
            if policy_id not in self._loaded_policies:
                return self._reply(f"policy is not loaded: {policy_id}", accepted=False)
            if not self._state.status.enabled:
                return self._reply("robot must be enabled before starting policy", accepted=False)
            self._state.status.mode = MODE_POLICY
            self._state.status.active_policy_id = policy_id
            return self._reply(f"started policy {policy_id}")

    def stop_policy(self) -> dict[str, Any]:
        with self._lock:
            self._state.status.active_policy_id = ""
            self._state.status.mode = MODE_IDLE
            return self._reply("stopped policy")

    def set_velocity_command(self, vx_mps: float, vy_mps: float, wz_radps: float, timeout_s: float) -> dict[str, Any]:
        with self._lock:
            if not self._state.status.enabled:
                return self._reply("robot must be enabled before velocity commands", accepted=False)
            if self._state.status.mode not in {MODE_VELOCITY, MODE_MANUAL}:
                return self._reply("robot must be in velocity or manual mode", accepted=False)
            self._state.commanded_velocity = VelocityCommand(vx_mps, vy_mps, wz_radps, timeout_s)
            return self._reply("velocity command accepted")

    def status_dict(self) -> dict[str, Any]:
        with self._lock:
            self._tick_locked()
            return asdict(self._state.status)

    def state_dict(self) -> dict[str, Any]:
        with self._lock:
            self._tick_locked()
            return asdict(self._state)

    def _reply(self, message: str, accepted: bool = True) -> dict[str, Any]:
        self._tick_locked()
        return {"accepted": accepted, "message": message, "status": asdict(self._state.status)}

    def _tick_locked(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now

        self._state.status.state_sequence += 1
        self._state.status.stamp_unix_s = time.time()

        velocity = self._state.commanded_velocity
        for index, joint in enumerate(self._state.joints):
            phase = self._state.status.state_sequence * 0.03 + index
            joint.velocity_radps = velocity.vx_mps * 0.5 + math.sin(phase) * 0.02
            joint.position_rad += joint.velocity_radps * dt
            joint.effort_nm = abs(joint.velocity_radps) * 1.5

        self._state.imu.yaw_rad += velocity.wz_radps * dt
        self._state.imu.roll_rad = math.sin(self._state.status.state_sequence * 0.02) * 0.01
        self._state.imu.pitch_rad = math.cos(self._state.status.state_sequence * 0.018) * 0.01
