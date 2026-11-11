"""Kick approach for the ball chase: line a foot up behind a still ball, stand, kick. No ROS.

The walking controller kicks with ``/walk/kick`` (a fast step whose swing
strikes forward; see walking_controller.stepping). Offline it kicks a ball
whose centre is 0.12-0.26 m ahead of the kicking foot's sole centre and within
about +-0.06 m sideways of it, best from standing. The gait only walks forward
and turns, so the lateral alignment comes from the heading: the robot turns
until the ball appears where the kicking foot will be at its current
distance, then walks straight at it.

States: ``chase`` (the normal chase drives) -> ``approach`` (this module
drives, slowly) -> ``settle`` (zero command until the walking controller
stands) -> ``kick`` (request sent, waiting for it) -> ``cooldown`` (zero
command, then back to ``chase``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

FEET = ('left_foot', 'right_foot')


def other_foot(foot: str) -> str:
    return FEET[1] if foot == FEET[0] else FEET[0]


@dataclass(frozen=True)
class KickConfig:
    # Measured offline (kick_eval.py): sole centres at x 0.03 m and these y in base_link when
    # standing. The model's 'left_foot' is on the robot's RIGHT (negative y): names as the
    # walking controller's /walk/kick expects them. The kick drives a ball centred 0.18-0.29 m
    # ahead of base_link (0.32: missed). The node freezes the ball position before it gets this
    # close (partial views next to the feet read up to 7 cm short).
    left_foot_y: float = -0.15
    right_foot_y: float = 0.16
    # On the estimate. In the sim the frozen estimate still read 4-7 cm short of the true ball
    # (5 kicks, 2026-11-11), so 0.19 puts the true ball near 0.24, mid-window.
    target_x: float = 0.19
    window_x: float = 0.04
    window_y: float = 0.035
    approach_m: float = 0.9        # the approach takes over within this distance (still ball)
    max_speed: float = 0.08        # m/s while approaching
    gain_x: float = 0.6            # 1/s
    gain_yaw: float = 1.5          # rad/s per rad
    max_yaw_rate: float = 0.3
    turn_first_rad: float = 0.12   # bearing error above which the approach only turns
    max_ball_speed: float = 0.12   # m/s: start only on a (nearly) still ball
    max_pos_std: float = 0.12      # m: estimate good enough to line up on
    # Once approaching, looser limits: velocity noise near the feet reached 0.13 m/s in the sim and
    # handing back to the chase there spun the robot in place next to the ball.
    keep_ball_speed: float = 0.3
    keep_pos_std: float = 0.25
    unusable_grace_s: float = 1.0  # hold still this long on a bad estimate before giving up
    settle_exit_scale: float = 1.6  # the window widens by this once settling (estimate jitter)
    foot_switch_y: float = 0.08    # keep the chosen foot unless the ball is this far to the other side
    settle_timeout_s: float = 4.0  # waiting to stand with the ball in the window
    kick_timeout_s: float = 3.0
    cooldown_s: float = 2.0


@dataclass
class WalkState:
    """What the walking controller reports on /walk/status."""
    standing: bool = False
    kick: Optional[str] = None     # None | requested | armed | kicking
    kicks: int = 0
    stamp: float = -math.inf


@dataclass
class KickDecision:
    active: bool                   # True: use vx, wz below instead of the chase command
    vx: float = 0.0
    wz: float = 0.0
    kick_foot: Optional[str] = None  # send this on /walk/kick now
    state: str = 'chase'


@dataclass
class KickApproach:
    cfg: KickConfig = field(default_factory=KickConfig)
    state: str = 'chase'
    foot: Optional[str] = None
    since: float = 0.0
    kicks_at_request: int = 0
    attempts: int = 0
    kicked: int = 0
    last_errors: tuple = (math.nan, math.nan)
    unusable_since: Optional[float] = None

    def _enter(self, state: str, now: float) -> None:
        self.state, self.since = state, now

    def errors(self, bx: float, by: float) -> tuple[float, float, float]:
        """(forward error, lateral error, bearing error) of the ball vs the kick spot of self.foot."""
        fy = self.cfg.left_foot_y if self.foot == 'left_foot' else self.cfg.right_foot_y
        ex, ey = bx - self.cfg.target_x, by - fy
        dist = math.hypot(bx, by)
        # Bearing at which the ball sits beside the foot at its current distance.
        want = math.asin(max(-1.0, min(1.0, fy / max(dist, 1e-6))))
        return ex, ey, math.remainder(math.atan2(by, bx) - want, 2.0 * math.pi)

    def _choose_foot(self, by: float) -> None:
        """The foot on the ball's side (by > 0: robot's left), with hysteresis."""
        s = self.cfg.foot_switch_y
        on_left = 'left_foot' if self.cfg.left_foot_y > 0 else 'right_foot'   # foot on the robot's left
        on_right = other_foot(on_left)
        if self.foot is None:
            self.foot = on_left if by >= 0.0 else on_right
        elif self.foot == on_left and by < -s:
            self.foot = on_right
        elif self.foot == on_right and by > s:
            self.foot = on_left

    def update(self, now: float, ball_base: Optional[tuple], ball_speed: float, pos_std: float,
               walk: WalkState) -> KickDecision:
        """ball_base: (x, y) ball centre in base_link, or None without a usable estimate."""
        c = self.cfg
        hold = KickDecision(True, state=self.state)
        if self.state == 'cooldown':
            if now - self.since >= c.cooldown_s:
                self._enter('chase', now)
            else:
                return hold
        if self.state == 'kick':
            if walk.kicks > self.kicks_at_request or walk.kick == 'kicking':
                self.kicked += walk.kicks > self.kicks_at_request
                if walk.kick != 'kicking':
                    self._enter('cooldown', now)
                return KickDecision(True, state=self.state)
            if now - self.since > c.kick_timeout_s:
                self._enter('cooldown', now)
            return KickDecision(True, state=self.state)

        engaged = self.state in ('approach', 'settle')
        speed_limit, std_limit = ((c.keep_ball_speed, c.keep_pos_std) if engaged
                                  else (c.max_ball_speed, c.max_pos_std))
        usable = ball_base is not None and pos_std <= std_limit and ball_speed <= speed_limit
        if not usable:
            if not engaged:
                return KickDecision(False, state=self.state)
            self.unusable_since = now if self.unusable_since is None else self.unusable_since
            if now - self.unusable_since > c.unusable_grace_s:
                self._enter('cooldown', now)
            return KickDecision(True, state=self.state)
        self.unusable_since = None
        bx, by = ball_base
        if self.state == 'chase':
            if math.hypot(bx, by) > c.approach_m or bx <= 0.0:
                return KickDecision(False, state=self.state)
            self.foot = None
            self.attempts += 1
            self._enter('approach', now)
        if self.state == 'approach':
            self._choose_foot(by)
        ex, ey, eb = self.errors(bx, by)
        self.last_errors = (ex, ey)
        scale = c.settle_exit_scale if self.state == 'settle' else 1.0
        in_window = abs(ex) <= scale * c.window_x and abs(ey) <= scale * c.window_y
        if bx < c.target_x - c.window_x - 0.05:
            # Overran the ball (the gait cannot step back): leave it to the chase for a while.
            self._enter('cooldown', now)
            return hold
        if self.state == 'approach':
            if in_window:
                self._enter('settle', now)
            else:
                wz = max(-c.max_yaw_rate, min(c.max_yaw_rate, c.gain_yaw * eb))
                vx = 0.0 if abs(eb) > c.turn_first_rad else max(0.0, min(c.max_speed, c.gain_x * ex))
                return KickDecision(True, vx, wz, state=self.state)
        # settle: zero command until the walking controller stands, the ball still in the window.
        if not in_window:
            self._enter('approach', now)
            return KickDecision(True, state=self.state)
        if walk.standing and walk.kick is None:
            self.kicks_at_request = walk.kicks
            self._enter('kick', now)
            return KickDecision(True, kick_foot=self.foot, state=self.state)
        if now - self.since > c.settle_timeout_s:
            self._enter('cooldown', now)
        return KickDecision(True, state=self.state)

    def status(self) -> dict:
        ex, ey = self.last_errors
        return {'state': self.state, 'foot': self.foot, 'attempts': self.attempts, 'kicks': self.kicked,
                'error_x_m': None if math.isnan(ex) else round(ex, 3),
                'error_y_m': None if math.isnan(ey) else round(ey, 3)}
