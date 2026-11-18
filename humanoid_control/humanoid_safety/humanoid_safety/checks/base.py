"""Shared types for safety checks: observations, results and the check base.

Checks do not depend on ROS, which keeps them deterministic and easy to test.
The node converts each ROS message into one of the observation types below and
hands it to the supervisor; SafetyCheck.observe routes it to the matching
observe_* hook, so a check only overrides the hooks for inputs it uses.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Mapping, Optional, Tuple

Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class ImuObservation:
    received_at: float  # Host monotonic seconds.
    roll_rad: float
    pitch_rad: float
    valid: bool = True
    source_age_s: float = 0.0  # Nonnegative source age at receipt, in seconds.
    angular_velocity: Optional[Vector3] = None  # rad/s; None if not reported.
    linear_acceleration: Optional[Vector3] = None  # m/s^2; None if not reported.


@dataclass(frozen=True)
class JointObservation:
    """One JointState message; joints with stale feedback are left out."""

    received_at: float  # Host monotonic seconds.
    positions: Mapping[str, float] = field(default_factory=dict)  # rad
    velocities: Mapping[str, float] = field(default_factory=dict)  # rad/s
    efforts: Mapping[str, float] = field(default_factory=dict)  # Nm
    source_age_s: float = 0.0


@dataclass(frozen=True)
class MotorObservation:
    """Per-motor telemetry that JointState does not carry."""

    received_at: float  # Host monotonic seconds.
    temperatures_c: Mapping[str, float] = field(default_factory=dict)
    fault_bits: Mapping[str, int] = field(default_factory=dict)


@dataclass(frozen=True)
class CanObservation:
    """One CAN transport status report from the motor node."""

    received_at: float  # Host monotonic seconds.
    transport_fault: bool = False
    tx_errors: int = 0


@dataclass(frozen=True)
class PowerObservation:
    received_at: float  # Host monotonic seconds.
    voltage_v: float = float("nan")
    percentage: Optional[float] = None  # 0..1; None if not reported.


@dataclass(frozen=True)
class CheckResult:
    name: str
    active: bool
    healthy: bool
    clearable: bool
    reason: str


class SafetyCheck(ABC):
    """Interface for a safety condition managed by the supervisor."""

    name: str

    def observe(self, observation: object) -> None:
        """Route an observation to the matching observe_* hook."""
        if isinstance(observation, ImuObservation):
            self.observe_imu(observation)
        elif isinstance(observation, JointObservation):
            self.observe_joints(observation)
        elif isinstance(observation, MotorObservation):
            self.observe_motors(observation)
        elif isinstance(observation, CanObservation):
            self.observe_can(observation)
        elif isinstance(observation, PowerObservation):
            self.observe_power(observation)
        else:
            raise TypeError(f"unknown observation type: {type(observation).__name__}")

    def observe_imu(self, observation: ImuObservation) -> None:
        del observation

    def observe_joints(self, observation: JointObservation) -> None:
        del observation

    def observe_motors(self, observation: MotorObservation) -> None:
        del observation

    def observe_can(self, observation: CanObservation) -> None:
        del observation

    def observe_power(self, observation: PowerObservation) -> None:
        del observation

    @abstractmethod
    def evaluate(self, now: float) -> CheckResult:
        """Return the check's current state."""

    def reset(self) -> None:
        """Clear non-latched internal counters after a supervisor reset."""


class FreshnessCheck(SafetyCheck):
    """Trip when a data stream stops arriving.

    Subclasses call _mark() from their observe_* hook for each good sample.
    Before the first sample a startup grace period applies.
    """

    def __init__(
        self,
        *,
        label: str,
        timeout_s: float,
        startup_grace_s: float,
        started_at: float,
        enabled: bool = True,
    ) -> None:
        self.label = label
        self.timeout_s = max(0.0, float(timeout_s))
        self.startup_grace_s = max(0.0, float(startup_grace_s))
        self.started_at = float(started_at)
        self.enabled = bool(enabled)
        self._last_received_at: Optional[float] = None

    def _mark(self, received_at: float, source_age_s: float = 0.0) -> None:
        # Preserve source delay: receipt of a 200-ms-old sample must not grant
        # another full timeout. Subsequent expiry is independent of ROS time.
        self._last_received_at = received_at - source_age_s

    def evaluate(self, now: float) -> CheckResult:
        if not self.enabled or self.timeout_s <= 0.0:
            return CheckResult(self.name, False, True, True, "disabled")

        if self._last_received_at is None:
            age = now - self.started_at
            in_grace = age <= self.startup_grace_s
            reason = (
                f"waiting for {self.label} ({age:.3f}s/{self.startup_grace_s:.3f}s grace)"
                if in_grace
                else f"no {self.label} received within {self.startup_grace_s:.3f}s"
            )
            return CheckResult(
                self.name,
                not in_grace,
                in_grace,
                False,
                reason,
            )

        age = max(0.0, now - self._last_received_at)
        fresh = age <= self.timeout_s
        return CheckResult(
            self.name,
            not fresh,
            fresh,
            fresh,
            f"{self.label} age={age:.3f}s limit={self.timeout_s:.3f}s",
        )

