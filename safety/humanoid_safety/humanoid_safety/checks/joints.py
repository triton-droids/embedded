"""Joint checks: position and speed limits from measured feedback."""

from __future__ import annotations

import math
from typing import Dict, Mapping, Tuple

from .base import CheckResult, JointObservation, SafetyCheck


class JointPositionLimitCheck(SafetyCheck):
    """Trip when a joint goes past its position limit plus a small margin.

    The margin allows for overshoot at the limit; reset needs every joint back
    inside its real limits.
    """

    name = "joint_position"

    def __init__(self, *, limits_rad: Mapping[str, Tuple[float, float]], margin_rad: float = 0.05) -> None:
        self.limits_rad = {joint: (float(low), float(high)) for joint, (low, high) in limits_rad.items()}
        self.margin_rad = max(0.0, float(margin_rad))
        self._positions: Dict[str, float] = {}

    def observe_joints(self, observation: JointObservation) -> None:
        self._positions.update(observation.positions)

    def _outside(self, joint: str, position: float, margin: float) -> bool:
        low, high = self.limits_rad[joint]
        return not math.isfinite(position) or position < low - margin or position > high + margin

    def evaluate(self, now: float) -> CheckResult:
        del now
        checked = {joint: pos for joint, pos in self._positions.items() if joint in self.limits_rad}
        over = sorted(joint for joint, pos in checked.items()
                      if self._outside(joint, pos, self.margin_rad))
        clearable = not any(self._outside(joint, pos, 0.0) for joint, pos in checked.items())
        if over:
            reason = ", ".join(
                f"{joint}={checked[joint]:+.3f} not in "
                f"[{self.limits_rad[joint][0]:+.3f}, {self.limits_rad[joint][1]:+.3f}]"
                for joint in over)
            return CheckResult(self.name, True, True, clearable, f"past limit: {reason}")
        return CheckResult(self.name, False, True, clearable,
                           f"{len(checked)} joints within limits")


class JointVelocityLimitCheck(SafetyCheck):
    """Trip when a joint moves faster than allowed for several samples in a row."""

    name = "joint_velocity"

    def __init__(self, *, max_velocity_rad_s: float, confirm_samples: int = 3) -> None:
        self.max_velocity_rad_s = abs(float(max_velocity_rad_s))
        self.confirm_samples = max(1, int(confirm_samples))
        self._counts: Dict[str, int] = {}
        self._latest: Dict[str, float] = {}

    def observe_joints(self, observation: JointObservation) -> None:
        self._latest = dict(observation.velocities)
        for joint, velocity in observation.velocities.items():
            over = not math.isfinite(velocity) or abs(velocity) > self.max_velocity_rad_s
            self._counts[joint] = self._counts.get(joint, 0) + 1 if over else 0

    def evaluate(self, now: float) -> CheckResult:
        del now
        over = sorted(joint for joint, count in self._counts.items()
                      if count >= self.confirm_samples)
        if over:
            reason = ", ".join(f"{joint}={self._latest.get(joint, float('nan')):+.2f}rad/s"
                               for joint in over)
            return CheckResult(self.name, True, True, False,
                               f"over {self.max_velocity_rad_s:.2f}rad/s: {reason}")
        return CheckResult(self.name, False, True, True, "joint speeds within limit")

    def reset(self) -> None:
        self._counts.clear()

