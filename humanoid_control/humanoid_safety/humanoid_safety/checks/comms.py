"""Communication checks: CAN transport faults and a silent motor node."""

from __future__ import annotations

from .base import CanObservation, CheckResult, FreshnessCheck


class CanLinkCheck(FreshnessCheck):
    """Trip when the motor node reports a CAN fault or stops reporting at all."""

    name = "can_link"

    def __init__(self, *, timeout_s: float, startup_grace_s: float, started_at: float) -> None:
        super().__init__(
            label="CAN status",
            timeout_s=timeout_s,
            startup_grace_s=startup_grace_s,
            started_at=started_at,
        )
        self._transport_fault = False
        self._tx_errors = 0

    def observe_can(self, observation: CanObservation) -> None:
        self._mark(observation.received_at)
        self._transport_fault = bool(observation.transport_fault)
        self._tx_errors = int(observation.tx_errors)

    def evaluate(self, now: float) -> CheckResult:
        result = super().evaluate(now)
        if self._transport_fault:
            return CheckResult(self.name, True, False, False,
                               f"CAN transport fault reported; {result.reason}")
        if self._tx_errors:
            return CheckResult(result.name, result.active, result.healthy, result.clearable,
                               f"{result.reason} tx_errors={self._tx_errors}")
        return result

