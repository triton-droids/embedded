"""Pure safety checks and the latched supervisor state machine.

Checks do not depend on ROS, which keeps them deterministic and easy to test.
New checks implement SafetyCheck and register with SafetySupervisor without
changing the shared latch/reset behavior.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence


@dataclass(frozen=True)
class ImuObservation:
    received_at: float
    roll_rad: float
    pitch_rad: float
    valid: bool = True


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

    def observe_imu(self, observation: ImuObservation) -> None:
        del observation

    @abstractmethod
    def evaluate(self, now: float) -> CheckResult:
        """Return the check's current state."""

    def reset(self) -> None:
        """Clear non-latched internal counters after a supervisor reset."""


class FallDetectionCheck(SafetyCheck):
    name = "fall_detection"

    def __init__(
        self,
        *,
        roll_limit_deg: float = 40.0,
        pitch_limit_deg: float = 40.0,
        reset_roll_deg: float = 30.0,
        reset_pitch_deg: float = 30.0,
        confirm_samples: int = 3,
    ) -> None:
        self.roll_limit_rad = math.radians(float(roll_limit_deg))
        self.pitch_limit_rad = math.radians(float(pitch_limit_deg))
        self.reset_roll_rad = math.radians(float(reset_roll_deg))
        self.reset_pitch_rad = math.radians(float(reset_pitch_deg))
        self.confirm_samples = max(1, int(confirm_samples))
        self._consecutive_over_limit = 0
        self._latest: Optional[ImuObservation] = None
        self._active = False

    def observe_imu(self, observation: ImuObservation) -> None:
        self._latest = observation
        if not observation.valid:
            self._consecutive_over_limit = self.confirm_samples
            self._active = True
            return

        over_limit = (
            abs(observation.roll_rad) > self.roll_limit_rad
            or abs(observation.pitch_rad) > self.pitch_limit_rad
        )
        if over_limit:
            self._consecutive_over_limit += 1
        else:
            self._consecutive_over_limit = 0
        self._active = self._consecutive_over_limit >= self.confirm_samples

    def evaluate(self, now: float) -> CheckResult:
        del now
        if self._latest is None:
            return CheckResult(
                self.name, False, False, False, "waiting for first IMU sample"
            )
        if not self._latest.valid:
            return CheckResult(
                self.name, True, False, False, "invalid IMU orientation quaternion"
            )

        roll_deg = math.degrees(self._latest.roll_rad)
        pitch_deg = math.degrees(self._latest.pitch_rad)
        clearable = (
            abs(self._latest.roll_rad) <= self.reset_roll_rad
            and abs(self._latest.pitch_rad) <= self.reset_pitch_rad
        )
        reason = (
            f"roll={roll_deg:+.1f}deg pitch={pitch_deg:+.1f}deg "
            f"count={self._consecutive_over_limit}/{self.confirm_samples}"
        )
        return CheckResult(self.name, self._active, True, clearable, reason)

    def reset(self) -> None:
        self._consecutive_over_limit = 0
        self._active = False


class ImuFreshnessCheck(SafetyCheck):
    name = "imu_freshness"

    def __init__(
        self,
        *,
        timeout_s: float,
        startup_grace_s: float,
        started_at: float,
        enabled: bool = True,
    ) -> None:
        self.timeout_s = max(0.0, float(timeout_s))
        self.startup_grace_s = max(0.0, float(startup_grace_s))
        self.started_at = float(started_at)
        self.enabled = bool(enabled)
        self._last_received_at: Optional[float] = None

    def observe_imu(self, observation: ImuObservation) -> None:
        self._last_received_at = observation.received_at

    def evaluate(self, now: float) -> CheckResult:
        if not self.enabled or self.timeout_s <= 0.0:
            return CheckResult(self.name, False, True, True, "disabled")

        if self._last_received_at is None:
            age = now - self.started_at
            in_grace = age <= self.startup_grace_s
            reason = (
                f"waiting for IMU ({age:.3f}s/{self.startup_grace_s:.3f}s grace)"
                if in_grace
                else f"no IMU received within {self.startup_grace_s:.3f}s"
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
            f"IMU age={age:.3f}s limit={self.timeout_s:.3f}s",
        )


class SafetySupervisor:
    """Evaluate registered checks and latch any unsafe condition."""

    def __init__(self, checks: Iterable[SafetyCheck]) -> None:
        self._checks = list(checks)
        self.latched = False
        self.reason = ""
        self.tripped_at: Optional[float] = None
        self.results: Sequence[CheckResult] = ()

    @property
    def checks(self) -> Sequence[SafetyCheck]:
        return tuple(self._checks)

    def observe_imu(self, observation: ImuObservation) -> None:
        for check in self._checks:
            check.observe_imu(observation)

    def evaluate(self, now: float) -> bool:
        self.results = tuple(check.evaluate(now) for check in self._checks)
        active = [result for result in self.results if result.active]
        if active and not self.latched:
            self.latched = True
            self.tripped_at = now
            self.reason = "; ".join(
                f"{result.name}: {result.reason}" for result in active
            )
            return True
        return False

    def try_reset(self, now: float) -> tuple[bool, str]:
        self.results = tuple(check.evaluate(now) for check in self._checks)
        blockers = [
            result
            for result in self.results
            if not result.healthy or not result.clearable
        ]
        if blockers:
            reason = "; ".join(
                f"{result.name}: {result.reason}" for result in blockers
            )
            return False, f"reset blocked: {reason}"

        for check in self._checks:
            check.reset()
        self.latched = False
        self.reason = ""
        self.tripped_at = None
        return True, "safety latch reset; motors remain disabled until explicitly enabled"


def quaternion_to_roll_pitch(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, bool]:
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        return 0.0, 0.0, False
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-6:
        return 0.0, 0.0, False

    x, y, z, w = (value / norm for value in values)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    return roll, pitch, True
