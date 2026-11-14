"""Frame pacing for the rendered cameras, in wall time or in simulated time.

Wall mode keeps the original behaviour: one frame per period of host time,
restarting the schedule after a late frame. Simulation-clock mode (use_sim_time)
renders when the physics stamp has advanced one period, so a slowed simulation
still yields ``rate_hz`` frames per simulated second; between frames the loop
polls briefly instead of sleeping a host-time period.
"""
from __future__ import annotations

import threading
import time

SIM_POLL_S = 0.002


class FramePacer:
    def __init__(self, period_s: float, sim_clock: bool, poll_s: float = SIM_POLL_S) -> None:
        if not period_s > 0.0:
            raise ValueError('frame period must be positive')
        self.sim_clock = bool(sim_clock)
        self._poll_s = poll_s
        self._period_ns = int(round(period_s * 1e9))
        self._period_s = period_s
        self._next_ns = None
        self._next_wall = time.monotonic()

    def due(self, stamp_ns) -> bool:
        """Whether to render the state stamped ``stamp_ns`` (always, in wall mode)."""
        if not self.sim_clock:
            return True
        if stamp_ns is None:
            return False
        if self._next_ns is not None and stamp_ns < self._next_ns:
            return False
        # Next frame one period later; after a gap (reset, pause) restart from here.
        nxt = stamp_ns + self._period_ns if self._next_ns is None else self._next_ns + self._period_ns
        self._next_ns = nxt if nxt > stamp_ns else stamp_ns + self._period_ns
        return True

    def wait(self, stop: threading.Event) -> None:
        """Sleep until the next frame may be due."""
        if self.sim_clock:
            stop.wait(self._poll_s)
            return
        self._next_wall += self._period_s
        delay = self._next_wall - time.monotonic()
        if delay > 0.0:
            stop.wait(delay)
        else:
            self._next_wall = time.monotonic()
