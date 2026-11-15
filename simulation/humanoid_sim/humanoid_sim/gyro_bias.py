"""Start-up gyro bias calibration, as an IMU board does at power-on.

The bias is the mean measured rate over a sliding window of ``duration_s`` in
which the board is still. Still means both:

* every sample has ``|gyro| < still_rad_s`` (a larger rate empties the window);
* the window is stationary: the mean rates of its first and second halves differ
  by at most ``drift_rad_s``. A per-sample threshold alone accepts slow motion:
  in simulation the head still pitched at 0.01-0.03 rad/s after spawning, which
  passed a 0.05 rad/s threshold and biased the estimate by about as much as the
  bias itself. White noise alone exceeds 0.001 rad/s here about 1 % of the time
  (MPU9250 noise, 1 s window).

Samples are withheld until then, like an MCU that publishes only after
calibrating. If no still window is found within ``timeout_s`` it gives up and
passes samples through uncorrected (``failed``). Times are sample stamps, so a
slowed simulation needs the same simulated time as hardware needs wall time.
"""
from __future__ import annotations

from collections import deque

import numpy as np

CHECK_EVERY = 10  # Samples between stationarity checks once the window is full.


class GyroBiasCalibrator:
    def __init__(self, duration_s: float, still_rad_s: float = 0.05, timeout_s: float = 30.0,
                 drift_rad_s: float = 0.001) -> None:
        if (duration_s < 0.0 or not still_rad_s > 0.0 or not timeout_s > duration_s
                or not drift_rad_s > 0.0):
            raise ValueError('need duration_s >= 0, still_rad_s > 0, drift_rad_s > 0 '
                             'and timeout_s > duration_s')
        self._duration_ns = int(duration_s * 1e9)
        self._still = float(still_rad_s)
        self._drift = float(drift_rad_s)
        self._timeout_ns = int(timeout_s * 1e9)
        self.bias = np.zeros(3)
        self.ready = duration_s == 0.0  # 0: calibration off.
        self.failed = False
        self.samples = 0
        self.rejected_drift = 0  # Full windows rejected as not stationary.
        self._first_ns = None
        self._window = deque()  # (stamp_ns, gyro)
        self._since_check = 0

    def update(self, stamp_ns: int, gyro) -> bool:
        """Feed one measured sample; True once samples may be published (then: correct())."""
        if self.ready:
            return True
        gyro = np.asarray(gyro, dtype=float)
        if self._first_ns is None:
            self._first_ns = stamp_ns
        if np.linalg.norm(gyro) >= self._still:
            self._window.clear()
        else:
            self._window.append((stamp_ns, gyro))
            while stamp_ns - self._window[0][0] > self._duration_ns:
                self._window.popleft()
            self._since_check += 1
            if (stamp_ns - self._window[0][0] >= self._duration_ns and len(self._window) >= 20
                    and self._since_check >= CHECK_EVERY):
                self._since_check = 0
                rates = np.array([g for _, g in self._window])
                half = len(rates) // 2
                if np.linalg.norm(rates[:half].mean(0) - rates[half:].mean(0)) <= self._drift:
                    self.bias, self.samples, self.ready = rates.mean(0), len(rates), True
                    return True
                self.rejected_drift += 1
        if stamp_ns - self._first_ns >= self._timeout_ns:
            self.ready = self.failed = True
            return True
        return False

    def correct(self, gyro) -> np.ndarray:
        return np.asarray(gyro, dtype=float) - self.bias
