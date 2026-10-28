"""Power checks: battery voltage."""

from __future__ import annotations

import math
from typing import Optional

from .base import CheckResult, PowerObservation, SafetyCheck


class BatteryVoltageCheck(SafetyCheck):
    """Trip when battery voltage stays below the minimum.

    Voltage sags briefly under heavy load, so it must be low for
    confirm_samples messages in a row. Reset needs it back above reset_voltage_v.
    """

    name = "battery_voltage"

    def __init__(
        self,
        *,
        min_voltage_v: float,
        reset_voltage_v: float,
        confirm_samples: int = 10,
    ) -> None:
        self.min_voltage_v = float(min_voltage_v)
        self.reset_voltage_v = max(float(reset_voltage_v), self.min_voltage_v)
        self.confirm_samples = max(1, int(confirm_samples))
        self._latest: Optional[float] = None
        self._low_count = 0

    def observe_power(self, observation: PowerObservation) -> None:
        voltage = observation.voltage_v
        self._latest = voltage
        low = not math.isfinite(voltage) or voltage < self.min_voltage_v
        self._low_count = self._low_count + 1 if low else 0

    def evaluate(self, now: float) -> CheckResult:
        del now
        if self._latest is None:
            return CheckResult(self.name, False, False, False, "waiting for battery state")
        active = self._low_count >= self.confirm_samples
        clearable = math.isfinite(self._latest) and self._latest >= self.reset_voltage_v
        reason = (f"voltage={self._latest:.2f}V min={self.min_voltage_v:.2f}V "
                  f"count={self._low_count}/{self.confirm_samples}")
        return CheckResult(self.name, active, True, clearable, reason)

    def reset(self) -> None:
        self._low_count = 0

