#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Shared safety monitor for real-motor actuation scripts.

Safety trips:
1) Joint position outside configured limits.
2) Per-sample position jump greater than max_step_deg.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Dict, Iterable, Tuple


class ActuationSafetyMonitor:
    def __init__(
        self,
        *,
        name: str,
        motor_ids: Iterable[int],
        joint_limits_by_id: Dict[int, Tuple[float, float]],
        read_logical_pos_fn: Callable[[int], float],
        halt_fn: Callable[[str], None],
        control_hz: float,
        read_hz: float | None = None,
        max_step_deg: float = 90.0,
        limit_margin_rad: float = 0.0,
    ):
        self.name = str(name)
        self.motor_ids = list(sorted(int(m) for m in motor_ids))
        self.joint_limits_by_id = dict(joint_limits_by_id)
        self.read_logical_pos_fn = read_logical_pos_fn
        self.halt_fn = halt_fn

        control_hz = max(1e-6, float(control_hz))
        if read_hz is None:
            # Keep safety reads above control rate without overloading CAN.
            read_hz = max(60.0, control_hz * 1.25)
        self.read_hz = max(float(read_hz), control_hz * 1.05)
        self.dt = 1.0 / self.read_hz

        self.max_step_rad = math.radians(float(max_step_deg))
        self.limit_margin_rad = float(limit_margin_rad)

        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()
        self._tripped_evt = threading.Event()
        self._last_pos: Dict[int, float] = {}
        self._reason: str | None = None

    @property
    def tripped(self) -> bool:
        return self._tripped_evt.is_set()

    @property
    def reason(self) -> str | None:
        return self._reason

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._tripped_evt.clear()
        self._reason = None
        self._last_pos.clear()
        self._thread = threading.Thread(target=self._loop, name=f"{self.name}_safety", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)

    def _trip(self, reason: str) -> None:
        if self._tripped_evt.is_set():
            return
        self._reason = reason
        self._tripped_evt.set()
        try:
            self.halt_fn(reason)
        except Exception:
            pass
        self._stop_evt.set()

    def _check_one(self, mid: int, pos: float) -> None:
        lo, hi = self.joint_limits_by_id[mid]
        if pos < (lo - self.limit_margin_rad) or pos > (hi + self.limit_margin_rad):
            self._trip(
                f"[SAFETY] motor {mid} out of joint limits: pos={pos:.4f} rad, "
                f"limits=[{lo:.4f},{hi:.4f}]"
            )
            return

        prev = self._last_pos.get(mid, None)
        if prev is not None:
            jump = abs(pos - prev)
            if jump > self.max_step_rad:
                self._trip(
                    f"[SAFETY] motor {mid} jump too large: |dpos|={math.degrees(jump):.2f} deg "
                    f"(threshold={math.degrees(self.max_step_rad):.2f} deg)"
                )
                return

        self._last_pos[mid] = pos

    def _loop(self) -> None:
        next_t = time.perf_counter()
        idx = 0
        while not self._stop_evt.is_set():
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)
            else:
                next_t = now

            if not self.motor_ids:
                next_t += self.dt
                continue

            # Round-robin one motor per tick to avoid saturating bus/locks.
            mid = self.motor_ids[idx % len(self.motor_ids)]
            idx += 1
            try:
                pos = float(self.read_logical_pos_fn(mid))
                self._check_one(mid, pos)
            except Exception:
                # Ignore transient comm errors; safety is about observed state violations.
                pass

            next_t += self.dt
            # Prevent unbounded drift accumulation if loop falls behind.
            if (time.perf_counter() - next_t) > (5.0 * self.dt):
                next_t = time.perf_counter()
