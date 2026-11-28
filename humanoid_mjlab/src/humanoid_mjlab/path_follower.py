"""Convert a world-XY path into mjlab body-frame forward and yaw commands."""

import math

from humanoid_mjlab.plan_path import PlanningMap, Point

Twist = tuple[float, float, float]


def angle_error(target: float, current: float) -> float:
    return math.remainder(target - current, math.tau)


def _segment_projection(point: Point, start: Point, end: Point) -> tuple[Point, float]:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return start, 0.0
    projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    fraction = max(0.0, min(1.0, projection))
    return (start[0] + fraction * dx, start[1] + fraction * dy), projection


def segment_distance(point: Point, start: Point, end: Point) -> float:
    projected, _ = _segment_projection(point, start, end)
    return math.dist(point, projected)


class PathFollower:
    def __init__(
        self, map_: PlanningMap, path: list[Point], *, speed: float,
        yaw_rate: float, tolerance: float, max_tracking_error: float,
    ) -> None:
        self.map = map_
        self.path = path
        self.speed = speed
        self.yaw_rate = yaw_rate
        self.tolerance = tolerance
        self.max_tracking_error = max_tracking_error
        self.next_index = min(1, len(path) - 1)
        self.previous = map_.start
        self.peak_tracking_error = 0.0
        self.status = "tracking"

    def _observe(self, point: Point, heading: float) -> bool:
        if self.status != "tracking":
            return False
        if not all(map(math.isfinite, (*point, heading))):
            self.status = "invalid_state"
            return False
        if not self.map.edge_clear(self.previous, point):
            self.status = "clearance_violation"
            return False

        self.previous = point
        return True

    def _check_tracking_error(self, error: float) -> bool:
        self.peak_tracking_error = max(self.peak_tracking_error, error)
        if error > self.max_tracking_error:
            self.status = "tracking_error"
            return False
        return True

    def update(self, point: Point, heading: float) -> Twist:
        if not self._observe(point, heading):
            return 0.0, 0.0, 0.0
        start = self.path[max(0, self.next_index - 1)]
        target = self.path[self.next_index]
        if not self._check_tracking_error(segment_distance(point, start, target)):
            return 0.0, 0.0, 0.0

        # Only advance when the next target is directly visible from the measured pose.
        while math.dist(point, target) <= self.tolerance:
            if self.next_index == len(self.path) - 1:
                self.status = "reached_goal"
                return 0.0, 0.0, 0.0
            following = self.path[self.next_index + 1]
            if not self.map.edge_clear(point, following):
                break
            self.next_index += 1
            target = following

        if not self.map.edge_clear(point, target):
            self.status = "blocked_tracking_target"
            return 0.0, 0.0, 0.0

        dx, dy = target[0] - point[0], target[1] - point[1]
        heading_error = angle_error(math.atan2(dy, dx), heading)
        forward = min(self.speed, 2.0 * math.hypot(dx, dy)) * max(0.0, math.cos(heading_error))
        turn = max(-self.yaw_rate, min(self.yaw_rate, 2.0 * heading_error))
        return forward, 0.0, turn


class LookaheadFollower(PathFollower):
    """Track a visible point ahead on the path with yaw damping and goal braking."""

    lookahead = 0.28  # m
    heading_kp = 2.6
    heading_kd = 0.55
    acceleration = 0.45  # m/s², limit when increasing the forward command
    deceleration = 0.18  # m/s², assumed by the braking speed limit
    settle_speed = 0.05  # m/s
    settle_yaw_rate = 0.08  # rad/s
    settle_time = 0.30  # s

    def __init__(
        self, map_: PlanningMap, path: list[Point], *, speed: float, yaw_rate: float,
        tolerance: float, max_tracking_error: float, step_dt: float,
    ) -> None:
        super().__init__(
            map_, path, speed=speed, yaw_rate=yaw_rate,
            tolerance=tolerance, max_tracking_error=max_tracking_error,
        )
        self.step_dt = step_dt
        self.forward_command = 0.0
        self.settled_seconds = 0.0

        self.remaining = [0.0] * len(path)
        for i in range(len(path) - 2, -1, -1):
            self.remaining[i] = math.dist(path[i], path[i + 1]) + self.remaining[i + 1]

    def _lookahead_point(self, point: Point, projected: Point) -> Point | None:
        """Stop extending the target at the first blocked line of sight."""
        budget = self.lookahead
        start = projected
        visible = None
        for end in self.path[self.next_index:]:
            length = math.dist(start, end)
            fraction = min(1.0, budget / length) if length else 1.0
            target = (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            )
            if not self.map.edge_clear(point, target):
                break
            visible = target
            if fraction < 1.0:
                break
            budget -= length
            start = end
        if visible is None and self.map.edge_clear(point, projected):
            return projected
        return visible

    def update(
        self, point: Point, heading: float, *, planar_speed: float = 0.0, measured_yaw_rate: float = 0.0,
    ) -> Twist:
        if not self._observe(point, heading):
            return 0.0, 0.0, 0.0
        if not all(map(math.isfinite, (planar_speed, measured_yaw_rate))):
            self.status = "invalid_state"
            return 0.0, 0.0, 0.0
        goal_distance = math.dist(point, self.path[-1])
        if goal_distance <= self.tolerance:
            if planar_speed <= self.settle_speed and abs(measured_yaw_rate) <= self.settle_yaw_rate:
                self.settled_seconds += self.step_dt
                if self.settled_seconds >= self.settle_time:
                    self.status = "reached_goal"
            else:
                self.settled_seconds = 0.0
            self.forward_command = 0.0
            return 0.0, 0.0, 0.0
        self.settled_seconds = 0.0

        while True:
            end = self.path[self.next_index]
            start = self.path[max(0, self.next_index - 1)]
            projected, along = _segment_projection(point, start, end)
            if self.next_index == len(self.path) - 1:
                break
            if along < 1.0 and math.dist(point, end) > self.tolerance:
                break
            if not self.map.edge_clear(point, self.path[self.next_index + 1]):
                break
            self.next_index += 1

        if not self._check_tracking_error(math.dist(point, projected)):
            return 0.0, 0.0, 0.0
        target = self._lookahead_point(point, projected)
        if target is None or target == point:
            self.status = "blocked_tracking_target"
            return 0.0, 0.0, 0.0

        dx, dy = target[0] - point[0], target[1] - point[1]
        error = angle_error(math.atan2(dy, dx), heading)
        turn = self.heading_kp * error - self.heading_kd * measured_yaw_rate
        turn = max(-self.yaw_rate, min(self.yaw_rate, turn))
        remaining = max(goal_distance, math.dist(projected, end) + self.remaining[self.next_index])
        braking_speed = math.sqrt(2.0 * self.deceleration * max(0.0, remaining - self.tolerance / 2.0))
        desired_speed = min(self.speed, braking_speed) * max(0.0, math.cos(error)) ** 2
        self.forward_command = min(desired_speed, self.forward_command + self.acceleration * self.step_dt)
        return self.forward_command, 0.0, turn
