"""Ball world model for the chase: odom-frame tracking and what to chase. No ROS.

The tracker (``tracking``, vendored td_ball_tracker) runs in ``odom`` (world
fixed), so a ball keeps its position and velocity while the robot walks, turns
or loses sight of it. This module adds what the chase needs on top:

* ``ball_center``: depth gives the ball's front surface; move it to the center.
* ``measurement``: one detection as an odom-frame tracker measurement.
* ``plan``: from a tracker output to a point to chase in ``base_link`` and a
  speed scale (tracking, coasting, or nothing to chase).
* ``look_pixel``: a predicted ball as the pixel the head tracker should centre.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .tracking.tracker import COASTING, LOST, NO_TRACK, TENTATIVE, TRACKING, Measurement, TrackerConfig

STATUS_NAMES = {NO_TRACK: 'none', TENTATIVE: 'tentative', TRACKING: 'tracking', COASTING: 'coasting',
                LOST: 'lost'}


def tracker_config(**overrides) -> TrackerConfig:
    """td_ball_tracker defaults retuned for the 6 Hz simulated camera.

    The upstream values assume 60 Hz: 3 hits within 0.5 s and a 0.25 s
    tentative timeout drop a candidate on a single missed 6 Hz frame.
    """
    cfg = TrackerConfig(
        friction=0.4,          # 1/s; calibrated value is set by the node parameter
        confirm_hits=2,
        confirm_window=0.6,
        tentative_timeout=0.4,  # > two frame periods
        coast_timeout=2.0,
        max_pos_std=1.5,
    )
    for key, value in overrides.items():
        if not hasattr(cfg, key):
            raise ValueError(f'unknown tracker parameter {key}')
        setattr(cfg, key, type(getattr(cfg, key))(value))
    return cfg


def ball_center(point_optical, radius: float) -> np.ndarray:
    """Median depth at the box center hits the front surface: push it back by one radius."""
    p = np.asarray(point_optical, dtype=float)
    n = np.linalg.norm(p)
    return p if n <= 1e-9 else p * (1.0 + radius / n)


def measurement(point_odom, distance: float, score: float, sigma0: float, sigma_per_m: float) -> Measurement:
    """Isotropic floor-plane measurement; noise grows with range."""
    sigma = sigma0 + sigma_per_m * max(distance, 0.0)
    return Measurement(np.asarray(point_odom, float)[:2].copy(), np.eye(2) * sigma**2, float(score))


@dataclass(frozen=True)
class PlanConfig:
    # A moving ball is chased slightly ahead: where the motion model puts it lead_s from now.
    # Short on purpose: a far target (e.g. the rest point v / friction) turns the robot away from a
    # ball that does not roll freely (a dribbled ball, low turf friction) and loses it from view.
    lead_s: float = 1.0
    lead_speed: float = 0.15       # m/s: slower balls (or velocity noise) get no lead
    lead_vel_std: float = 0.25     # m/s; steady state at 6 Hz with the quiet model is ~0.16
    max_lead_m: float = 1.0
    coast_speed_scale: float = 0.5
    coast_turn_only_std: float = 0.5  # m: coasting σ above this → turn toward it, do not walk
    # peek() between frames always reports COASTING; a detection this recent (frame period plus
    # inference latency at 6 Hz) still counts as tracking.
    fresh_s: float = 0.5


@dataclass(frozen=True)
class ChasePlan:
    mode: str                        # 'tracking' | 'coasting' | 'none'
    point_odom: Optional[np.ndarray]  # xy to chase (position, or lead point)
    speed_scale: float
    lead: bool
    pos_std: float
    speed: float


NONE = ChasePlan('none', None, 0.0, False, math.inf, 0.0)


def lead_offset(velocity, friction: float, horizon: float, limit: float) -> np.ndarray:
    """Distance travelled in ``horizon`` s under v·exp(-c·t): v·(1 - e^(-c·T)) / c, capped."""
    v = np.asarray(velocity, float)
    gain = (1.0 - math.exp(-friction * horizon)) / friction if friction > 1e-9 else horizon
    offset = v * gain
    n = np.linalg.norm(offset)
    return offset * (limit / n) if n > limit else offset


def plan(out, friction: float, cfg: PlanConfig = PlanConfig(), now: Optional[float] = None) -> ChasePlan:
    """What to chase for one tracker output (``TrackOutput`` or None) predicted to ``now``."""
    if out is None or out.status not in (TRACKING, COASTING):
        return NONE
    x, P = np.asarray(out.x, float), np.asarray(out.P, float)
    if not (np.isfinite(x).all() and np.isfinite(P).all()):
        return NONE
    pos_std = math.sqrt(max(P[0, 0], P[1, 1], 0.0))
    vel_std = math.sqrt(max(P[2, 2], P[3, 3], 0.0))
    speed = float(np.hypot(x[2], x[3]))
    point = x[:2].copy()
    lead = cfg.lead_s > 0.0 and speed > cfg.lead_speed and vel_std < cfg.lead_vel_std
    if lead:
        point = point + lead_offset(x[2:], friction, cfg.lead_s, cfg.max_lead_m)
    fresh = out.status == TRACKING or (now is not None and now - out.last_seen <= cfg.fresh_s)
    if fresh:
        return ChasePlan('tracking', point, 1.0, lead, pos_std, speed)
    scale = 0.0 if pos_std > cfg.coast_turn_only_std else cfg.coast_speed_scale
    return ChasePlan('coasting', point, scale, lead, pos_std, speed)


def look_pixel(point_optical, k: np.ndarray, far: float = 10.0) -> tuple[float, float]:
    """Pixel of a camera-optical point, also for points behind the camera.

    The head tracker turns by atan((pixel - c) / f), so an off-image pixel is
    still the correct angle. Points behind the camera (or at z ≈ 0) are placed
    ``far`` focal lengths out on their side, about 84 degrees: turn that way.
    """
    x, y, z = (float(v) for v in point_optical)
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    if z > 0.05:
        ax, ay = x / z, y / z
        ax, ay = max(-far, min(far, ax)), max(-far, min(far, ay))
    else:
        ax = math.copysign(far, x) if abs(x) > 1e-6 else far
        ay = max(-far, min(far, y / max(abs(x), 1e-6) * far)) if abs(x) > 1e-6 else 0.0
    return cx + fx * ax, cy + fy * ay
