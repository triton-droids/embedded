"""Motor checks: temperature, fault flags, torque and stale feedback."""

from __future__ import annotations

import math
from typing import Dict, Iterable, Mapping, Optional

from .base import CheckResult, JointObservation, MotorObservation, SafetyCheck


class MotorTemperatureCheck(SafetyCheck):
    """Trip when any motor is too hot; reset only after it cools down."""

    name = "motor_temperature"

    def __init__(self, *, max_temp_c: float = 80.0, reset_temp_c: float = 70.0) -> None:
        self.max_temp_c = float(max_temp_c)
        self.reset_temp_c = min(float(reset_temp_c), self.max_temp_c)
        self._temperatures: Optional[Mapping[str, float]] = None

    def observe_motors(self, observation: MotorObservation) -> None:
        self._temperatures = dict(observation.temperatures_c)

    def evaluate(self, now: float) -> CheckResult:
        del now
        if self._temperatures is None:
            return CheckResult(self.name, False, False, False, "waiting for motor telemetry")
        # A NaN temperature cannot be trusted, so count it as too hot.
        hot = {joint: temp for joint, temp in self._temperatures.items()
               if not math.isfinite(temp) or temp > self.max_temp_c}
        warm = [joint for joint, temp in self._temperatures.items()
                if not math.isfinite(temp) or temp > self.reset_temp_c]
        hottest = max(self._temperatures.items(), key=lambda item: item[1], default=None)
        if hot:
            reason = ", ".join(f"{joint}={temp:.1f}C" for joint, temp in sorted(hot.items()))
            reason += f" over {self.max_temp_c:.1f}C"
        elif hottest is not None:
            reason = f"hottest {hottest[0]}={hottest[1]:.1f}C limit={self.max_temp_c:.1f}C"
        else:
            reason = "no motors reporting"
        return CheckResult(self.name, bool(hot), True, not warm, reason)


class MotorFaultCheck(SafetyCheck):
    """Trip when any motor reports nonzero fault bits."""

    name = "motor_fault"

    def __init__(self) -> None:
        self._faults: Dict[str, int] = {}

    def observe_motors(self, observation: MotorObservation) -> None:
        self._faults = {joint: int(bits) for joint, bits in observation.fault_bits.items() if bits}

    def evaluate(self, now: float) -> CheckResult:
        del now
        if self._faults:
            reason = ", ".join(f"{joint}=0x{bits:02x}" for joint, bits in sorted(self._faults.items()))
            return CheckResult(self.name, True, True, False, f"fault bits {reason}")
        return CheckResult(self.name, False, True, True, "no motor faults reported")


class MotorTorqueCheck(SafetyCheck):
    """Trip when a joint's measured torque stays over its limit.

    Short spikes are normal on contact, so the limit must be exceeded for
    confirm_samples messages in a row before the check trips.
    """

    name = "motor_torque"

    def __init__(
        self,
        *,
        default_limit_nm: float,
        limits_nm: Optional[Mapping[str, float]] = None,
        confirm_samples: int = 5,
    ) -> None:
        self.default_limit_nm = abs(float(default_limit_nm))
        self.limits_nm = {joint: abs(float(limit)) for joint, limit in (limits_nm or {}).items()}
        self.confirm_samples = max(1, int(confirm_samples))
        self._counts: Dict[str, int] = {}
        self._latest: Dict[str, float] = {}

    def _limit(self, joint: str) -> float:
        return self.limits_nm.get(joint, self.default_limit_nm)

    def observe_joints(self, observation: JointObservation) -> None:
        self._latest = dict(observation.efforts)
        for joint, effort in observation.efforts.items():
            over = not math.isfinite(effort) or abs(effort) > self._limit(joint)
            self._counts[joint] = self._counts.get(joint, 0) + 1 if over else 0

    def evaluate(self, now: float) -> CheckResult:
        del now
        over = sorted(joint for joint, count in self._counts.items()
                      if count >= self.confirm_samples)
        if over:
            reason = ", ".join(f"{joint}={self._latest.get(joint, float('nan')):+.1f}Nm"
                               f"/{self._limit(joint):.1f}" for joint in over)
            return CheckResult(self.name, True, True, False, f"torque over limit: {reason}")
        clearable = all(math.isfinite(effort) and abs(effort) <= self._limit(joint)
                        for joint, effort in self._latest.items())
        return CheckResult(self.name, False, True, clearable, "torque within limits")

    def reset(self) -> None:
        self._counts.clear()


class MotorFeedbackCheck(SafetyCheck):
    """Trip when any expected joint stops reporting feedback."""

    name = "motor_feedback"

    def __init__(
        self,
        *,
        expected_joints: Iterable[str],
        timeout_s: float,
        startup_grace_s: float,
        started_at: float,
    ) -> None:
        self.expected_joints = tuple(expected_joints)
        self.timeout_s = max(0.0, float(timeout_s))
        self.startup_grace_s = max(0.0, float(startup_grace_s))
        self.started_at = float(started_at)
        self._last_seen: Dict[str, float] = {}

    def observe_joints(self, observation: JointObservation) -> None:
        seen_at = observation.received_at - observation.source_age_s
        for joint in observation.positions:
            self._last_seen[joint] = seen_at

    def evaluate(self, now: float) -> CheckResult:
        in_grace = now - self.started_at <= self.startup_grace_s
        stale = []
        for joint in self.expected_joints:
            seen_at = self._last_seen.get(joint)
            if seen_at is None or now - seen_at > self.timeout_s:
                stale.append(joint)
        if not stale:
            return CheckResult(self.name, False, True, True,
                               f"{len(self.expected_joints)} joints fresh")
        reason = f"no feedback within {self.timeout_s:.3f}s: {', '.join(stale)}"
        if in_grace and all(joint not in self._last_seen for joint in stale):
            return CheckResult(self.name, False, True, False, f"startup grace; {reason}")
        return CheckResult(self.name, True, False, False, reason)

