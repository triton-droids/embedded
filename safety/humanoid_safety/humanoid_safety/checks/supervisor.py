"""The latched supervisor state machine. It knows nothing about sensors."""

from __future__ import annotations

from typing import Iterable, Optional, Sequence

from .base import CheckResult, ImuObservation, SafetyCheck


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

    def observe(self, observation: object) -> None:
        for check in self._checks:
            check.observe(observation)

    def observe_imu(self, observation: ImuObservation) -> None:
        self.observe(observation)

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

