"""Start-up gyro bias calibration, as an IMU board does at power-on.

The board is assumed still for a while after start (the robot waits on the
gantry). The calibrator averages the measured rate over ``duration_s`` of
contiguous still samples (``|gyro| < still_rad_s``; any motion restarts the
window), then subtracts that mean from every later sample. Samples are withheld
until then, like an MCU that publishes only after calibrating. If no still
window is found within ``timeout_s`` it gives up and passes samples through
uncorrected (``failed``). Times are sample stamps, so a slowed simulation needs
the same simulated time as real hardware would need wall time.
"""
from __future__ import annotations

import numpy as np


class GyroBiasCalibrator:
    def __init__(self, duration_s: float, still_rad_s: float = 0.05, timeout_s: float = 30.0) -> None:
        if duration_s < 0.0 or not still_rad_s > 0.0 or not timeout_s > duration_s:
            raise ValueError('need duration_s >= 0, still_rad_s > 0 and timeout_s > duration_s')
        self._duration_ns = int(duration_s * 1e9)
        self._still = float(still_rad_s)
        self._timeout_ns = int(timeout_s * 1e9)
        self.bias = np.zeros(3)
        self.ready = duration_s == 0.0  # 0: calibration off.
        self.failed = False
        self.samples = 0
        self._first_ns = None
        self._start_ns = None
        self._sum = np.zeros(3)
        self._count = 0

    def update(self, stamp_ns: int, gyro) -> bool:
        """Feed one measured sample; True once samples may be published (then: correct())."""
        if self.ready:
            return True
        gyro = np.asarray(gyro, dtype=float)
        if self._first_ns is None:
            self._first_ns = stamp_ns
        if np.linalg.norm(gyro) < self._still:
            if self._start_ns is None:
                self._start_ns, self._sum, self._count = stamp_ns, np.zeros(3), 0
            self._sum += gyro
            self._count += 1
            if stamp_ns - self._start_ns >= self._duration_ns and self._count >= 10:
                self.bias, self.samples, self.ready = self._sum / self._count, self._count, True
                return True
        else:
            self._start_ns = None  # Moved: restart the still window.
        if stamp_ns - self._first_ns >= self._timeout_ns:
            self.ready = self.failed = True
            return True
        return False

    def correct(self, gyro) -> np.ndarray:
        return np.asarray(gyro, dtype=float) - self.bias
