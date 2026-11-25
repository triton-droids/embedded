"""Velocity command (m/s, rad/s) -> step length and turn per step. No ROS."""
from __future__ import annotations

import numpy as np


def step_command(v, w, period_s, max_step_m, max_turn_rad, turn_free_rad=0.0, turn_stop_rad=0.0):
    """Return (step_length_m, turn_rad) for one step of `period_s`.

    Step = v * period in [0, max_step_m] (no backward steps); turn = w * period
    clipped to +-max_turn_rad. Turn first: with turn_stop_rad > turn_free_rad
    the step is scaled by how much of the turn is left, full at |turn| <=
    turn_free_rad and 0 at |turn| >= turn_stop_rad, so a large heading error
    is turned out in place before walking (0, 0: no coupling).
    """
    step = float(np.clip(v * period_s, 0.0, max_step_m))
    turn = float(np.clip(w * period_s, -max_turn_rad, max_turn_rad))
    if turn_stop_rad > turn_free_rad:
        scale = (turn_stop_rad - abs(turn)) / (turn_stop_rad - turn_free_rad)
        step *= float(np.clip(scale, 0.0, 1.0))
    return step, turn
