"""Goal-directed supervisor over the existing forward/turn and kick interfaces.

All xy coordinates passed to update are in the current forward-x, left-y base
frame. The caller transforms a known fixed goal and a visual ball estimate.
No motor commands, enable operation, watchdog or protection changes live here.
"""
from dataclasses import dataclass, field, replace
import math
from .kick import KickApproach, KickConfig, KickDecision, WalkState

def wrap(a):
    return math.remainder(a, 2 * math.pi)

def norm(v):
    return math.hypot(*v)

def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]

def add(a, b):
    return (a[0] + b[0], a[1] + b[1])

def scale(a, s):
    return (a[0] * s, a[1] * s)

@dataclass
class GoalShot:
    approach: KickApproach = field(default_factory=lambda: KickApproach(KickConfig(target_x=0.23, window_x=0.025, max_speed=0.05)))
    state: str = 'stage'
    side: float = 0.0
    aligned_since: float | None = None
    fault: str = ''
    started_at: float | None = None
    heading_error: float = 0.0
    target: tuple | None = None
    pose_settled_since: float | None = None
    foot_reference_xy: tuple | None = None
    restages: int = 0
    stage_distance: float = 0.65
    foot_y: float = 0.16
    max_heading_error: float = 0.18

    def hold(self):
        return KickDecision(True, state=self.state)

    def update(self, now, ball, goal, speed, std, walk, foot_xy=None):
        if self.state in ('done', 'fault'):
            return self.hold()
        if self.state == 'kick':
            # Once admitted, ball loss is expected. Only the existing walking
            # controller acknowledgement can complete this one-shot action.
            self.approach.update(now, None, 0.0, math.inf, walk)
            self.state = self.approach.state
            if self.state == 'cooldown':
                self.state = 'done' if self.approach.kicked else 'fault'
                if self.state == 'fault':
                    self.fault = 'kick completion not acknowledged'
            return self.hold()
        self.started_at = now if self.started_at is None else self.started_at
        if now - self.started_at > 120:
            self.state = 'fault'
            self.fault = 'shot attempt timed out'
            return self.hold()
        if ball is None or goal is None or (not all((math.isfinite(x) for x in (*ball, *goal, speed, std)))):
            return self.hold()
        if std > 0.25 or speed > 0.3:
            return self.hold()
        delta = add(goal, scale(ball, -1))
        distance = norm(delta)
        if distance < 0.3:
            self.state = 'fault'
            self.fault = 'goal too close to ball'
            return self.hold()
        direction = scale(delta, 1 / distance)
        left = (-direction[1], direction[0])
        self.heading_error = math.atan2(direction[1], direction[0])
        target = add(add(ball, scale(direction, -self.stage_distance)), scale(left, -self.foot_y))
        self.target = target
        if self.state == 'stage':
            length2 = dot(target, target)
            along = max(0, min(1, dot(ball, target) / max(length2, 1e-09)))
            closest = add(scale(target, along), scale(ball, -1))
            if self.side == 0 and norm(closest) < 0.38 and (norm(ball) < norm(target)):
                self.side = 1.0 if dot(scale(ball, -1), left) >= 0 else -1.0
                self.state = 'around'
        if self.state == 'around':
            target = add(add(ball, scale(direction, -0.35)), scale(left, self.side * 0.7))
            self.target = target
            if norm(target) < 0.09:
                self.state = 'stage'
        if self.state in ('stage', 'around'):
            if norm(target) < 0.12 and self.state == 'stage':
                self.state = 'align'
                self.aligned_since = None
            else:
                bearing = math.atan2(target[1], target[0])
                w = max(-0.22, min(0.22, 1.0 * bearing))
                v = 0.0 if abs(bearing) > 0.16 else min(0.1, 0.6 * norm(target))
                return KickDecision(True, v, w, state=self.state)
        if self.state == 'align':
            if abs(self.heading_error) > 0.055:
                self.aligned_since = None
                return KickDecision(True, 0, max(-0.18, min(0.18, self.heading_error)), state='align')
            if not walk.standing or walk.kick is not None:
                self.aligned_since = None
                return self.hold()
            self.aligned_since = now if self.aligned_since is None else self.aligned_since
            if now - self.aligned_since < 0.5:
                return self.hold()
            # Turning while stepping moves the base. Check the staging point
            # again only after standing, rather than correcting it beside the ball.
            if norm(target) > 0.18:
                self.restages += 1
                self.state = 'stage'
                self.aligned_since = None
                return self.hold()
            self.state = 'approach'
        if self.state in ('approach', 'settle'):
            if foot_xy is not None:
                if not all(math.isfinite(v) for v in foot_xy):
                    return self.hold()
                self.foot_reference_xy = tuple(foot_xy)
                self.approach.cfg = replace(self.approach.cfg,
                    target_x=foot_xy[0]+0.20, right_foot_y=foot_xy[1])
            c = self.approach.cfg
            foot_y = c.right_foot_y if foot_xy is not None else self.foot_y
            # Follow a pose behind the ball: both its translation and goal heading.
            # Do not let the generic foot-bearing controller turn away from the goal.
            spot = add(add(ball, scale(direction, -c.target_x)), scale(left, -foot_y))
            window_scale = c.settle_exit_scale if self.state == 'settle' else 1.0
            in_window = abs(ball[0] - c.target_x) <= window_scale*c.window_x and abs(ball[1] - foot_y) <= window_scale*c.window_y
            if not in_window or abs(self.heading_error) > 0.08:
                self.state = 'approach'
                self.pose_settled_since = None
                if self.approach.state == 'settle':
                    self.approach._enter('approach', now)
                if ball[0] < c.target_x - c.window_x - 0.05:
                    self.state = 'fault'
                    self.fault = 'ball behind kick window; reposition required'
                    return self.hold()
                if norm(ball) < 0.40:
                    # The terminal polar bearing becomes ill-conditioned at
                    # the feet. Keep the goal heading and make small forward
                    # corrections instead of chasing a near-zero vector.
                    forward_error = ball[0] - c.target_x
                    lateral_error = ball[1] - foot_y
                    vx = max(0, min(0.03, 0.6*forward_error))
                    wz = max(-0.12, min(0.12, 2.0*self.heading_error + 4.0*lateral_error))
                    return KickDecision(True, vx, wz, state=self.state)
                if norm(spot) < 0.04 and abs(self.heading_error) > 0.08:
                    return KickDecision(True, 0, max(-0.12, min(0.12, 1.5*self.heading_error)), state=self.state)
                alpha = math.atan2(spot[1], spot[0])
                beta = wrap(self.heading_error - alpha)
                vx = max(0, min(c.max_speed, 0.5 * norm(spot) * math.cos(alpha)))
                wz = max(-0.18, min(0.18, alpha - 0.3*beta))
                return KickDecision(True, vx, wz, state=self.state)
            if self.approach.state in ('chase', 'approach'):
                # Enter the existing settle window without admitting a kick;
                # its hysteresis absorbs the final support/body shift.
                self.approach.update(now, ball, speed, std,
                                     WalkState(False, walk.kick, walk.kicks, walk.stamp))
            self.state = 'settle'
            if not walk.standing or walk.kick is not None:
                self.pose_settled_since = None
                return self.hold()
            if foot_xy is not None and (abs(ball[0]-c.target_x)>c.window_x or abs(ball[1]-foot_y)>c.window_y):
                self.state = 'approach'
                self.pose_settled_since = None
                self.approach._enter('approach', now)
                return self.hold()
            self.pose_settled_since = now if self.pose_settled_since is None else self.pose_settled_since
            if now - self.pose_settled_since < 0.3:
                return self.hold()
        decision = self.approach.update(now, ball, speed, std, walk)
        self.state = self.approach.state
        if self.state == 'chase':
            self.state = 'stage'
            return self.hold()
        if decision.kick_foot and abs(self.heading_error) > self.max_heading_error:
            self.state = 'fault'
            self.fault = 'heading outside shot tolerance at kick window; reposition required'
            return self.hold()
        if self.state == 'cooldown':
            self.state = 'done'
        decision.state = self.state
        return decision

    def status(self):
        return {**self.approach.status(), 'state': self.state, 'goal_heading_error_rad': self.heading_error, 'staging_point_base': self.target, 'fault': self.fault, 'restages': self.restages, 'foot_reference_xy': self.foot_reference_xy}
