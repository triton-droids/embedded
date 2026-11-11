# Vendored from triton-droids td_perception_camera, td_ball_tracker/td_ball_tracker/tracker.py at cae8fb1
# ("P-33: ball tracker"). Keep in sync; local changes: none.
"""Single-ball track manager on top of BallKF (P-33). No ROS.

Lifecycle (status values match td_msgs/BallState):
    no track ──detection──▶ TENTATIVE ──confirm_hits within confirm_window──▶ TRACKING
    TENTATIVE ──no hit for tentative_timeout──▶ dropped (one-off false positives never surface)
    TRACKING ──frame without an in-gate detection──▶ COASTING ──coast_timeout or σ > max_pos_std──▶ LOST
    COASTING ──in-gate detection──▶ TRACKING
    LOST ──any detection──▶ new TENTATIVE track (position/velocity keep last values until then)

Noise switching: the filter runs a quiet process model (a still ball stays still, no fake velocity).
When a detection lands outside the 99% innovation region, the step is redone with a high process
noise so a kicked ball is followed instead of rejected.

Detections drive the filter (stamps = image stamps). Output at other times uses peek(t), which
predicts without mutating, so a late timer tick never pushes the filter past the next image stamp.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .kf import BallKF

TENTATIVE, TRACKING, COASTING, LOST = 0, 1, 2, 3
NO_TRACK = -1


@dataclass
class TrackerConfig:
    friction: float = 0.4  # 1/s, rolling decay rate on turf
    # Tuned on synthetic data (5-15 cm noise @60 Hz): still ball ~0.1 m/s rms velocity, kick followed.
    # Retune on real bags once measurement noise is known (P-41).
    accel_noise: float = 0.1  # q, m²/s³: quiet model, keeps a still ball still
    maneuver_accel_noise: float = 30.0  # q used when the quiet model is surprised (kick, bounce)
    maneuver_chi2: float = 13.8  # 2 dof, 99.9%: innovation above this → refilter with maneuver q
    gate_chi2: float = 25.0  # final gate, applied after the maneuver refilter
    confirm_hits: int = 3
    confirm_window: float = 0.5  # s
    tentative_timeout: float = 0.25  # s without a hit → drop tentative track
    coast_timeout: float = 1.5  # s without a hit → LOST
    max_pos_std: float = 2.0  # m, coasting σ above this → LOST
    init_vel_std: float = 3.0  # m/s


@dataclass
class Measurement:
    z: np.ndarray  # xy in odom, m
    R: np.ndarray  # 2x2 covariance, m²
    score: float


@dataclass
class TrackOutput:
    status: int
    x: np.ndarray  # [px, py, vx, vy]
    P: np.ndarray
    last_seen: float
    confidence: float
    visible: bool


class BallTracker:
    def __init__(self, cfg: TrackerConfig | None = None):
        self.cfg = cfg or TrackerConfig()
        self.kf: BallKF | None = None
        self.status = NO_TRACK
        self.hits = 0
        self.first_seen = self.last_seen = 0.0
        self.visible = False
        self.last_x = np.zeros(4)  # for LOST output
        self.last_P = np.eye(4)

    # --- filter driving (image stamps) ---
    def step(self, t: float, meas: list[Measurement]) -> None:
        c = self.cfg
        if self.kf is not None and t < self.kf.t:
            return  # out-of-order frame: ignore
        self.visible = False

        if self.kf is None or self.status == LOST:
            if meas:
                self._start(t, max(meas, key=lambda m: m.score))
            return

        x0, P0, t0 = self.kf.x.copy(), self.kf.P.copy(), self.kf.t
        self.kf.predict(t, c.friction, c.accel_noise)
        best, best_d2 = self._associate(meas)
        if meas and (best is None or best_d2 > c.maneuver_chi2):
            # Quiet model surprised: maybe a kick. Redo this step with high process noise.
            self.kf.x, self.kf.P, self.kf.t = x0, P0, t0
            self.kf.predict(t, c.friction, c.maneuver_accel_noise)
            best, best_d2 = self._associate(meas)

        if best is not None:
            self.kf.update(best.z, best.R)
            self.hits += 1
            self.last_seen = t
            self.visible = True
            if self.status == TENTATIVE:
                if self.hits >= c.confirm_hits and t - self.first_seen <= c.confirm_window:
                    self.status = TRACKING
            else:
                self.status = TRACKING
        elif self.status == TENTATIVE and meas and t - self.last_seen > c.tentative_timeout:
            self._start(t, max(meas, key=lambda m: m.score))  # stale tentative: restart on new candidate
            return

        self._age(t)

    def _associate(self, meas: list[Measurement]) -> tuple[Measurement | None, float]:
        """Closest in-gate measurement by Mahalanobis distance (not by detector score)."""
        best, best_d2 = None, self.cfg.gate_chi2
        for m in meas:
            d2 = self.kf.mahalanobis2(m.z, m.R)
            if d2 < best_d2:
                best, best_d2 = m, d2
        return best, best_d2

    def _start(self, t: float, m: Measurement) -> None:
        self.kf = BallKF(m.z, m.R, t, self.cfg.init_vel_std)
        self.status = TENTATIVE
        self.hits = 1
        self.first_seen = self.last_seen = t
        self.visible = True

    def _age(self, t: float) -> None:
        c = self.cfg
        since = t - self.last_seen
        if self.status == TENTATIVE:
            if since > c.tentative_timeout or (
                t - self.first_seen > c.confirm_window and self.hits < c.confirm_hits
            ):
                self.kf, self.status = None, NO_TRACK
            return
        if self.status in (TRACKING, COASTING) and since > 0:
            self.status = COASTING
        if self.status == COASTING:
            pos_std = math.sqrt(max(self.kf.P[0, 0], self.kf.P[1, 1]))
            if since > c.coast_timeout or pos_std > c.max_pos_std:
                self.last_x, self.last_P = self.kf.x.copy(), self.kf.P.copy()
                self.status = LOST

    # --- output ---
    def peek(self, t: float) -> TrackOutput | None:
        """Estimate at time t (≥ last filter time). None if there has never been a track."""
        c = self.cfg
        if self.status == NO_TRACK:
            return None
        if self.status == LOST:
            return TrackOutput(LOST, self.last_x, self.last_P, self.last_seen, 0.0, False)
        x, P = self.kf.predicted(t, c.friction, c.accel_noise)
        status = self.status
        since = t - self.last_seen
        if status == TRACKING and since > 1e-6:
            status = COASTING
        if status == COASTING and (
            since > c.coast_timeout or math.sqrt(max(P[0, 0], P[1, 1])) > c.max_pos_std
        ):
            return TrackOutput(LOST, x, P, self.last_seen, 0.0, False)
        conf = {
            TENTATIVE: 0.3 * min(self.hits / c.confirm_hits, 1.0),
            TRACKING: min(0.5 + 0.1 * self.hits, 1.0),
            COASTING: min(0.5 + 0.1 * self.hits, 1.0) * math.exp(-since / c.coast_timeout),
        }[status]
        return TrackOutput(status, x, P, self.last_seen, conf, self.visible and since <= 1e-6)
