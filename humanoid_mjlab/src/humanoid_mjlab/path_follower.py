"""Convert a world-XY path into mjlab body-frame forward and yaw commands."""

import math

from humanoid_mjlab.plan_path import PlanningMap, Point

Twist = tuple[float, float, float]


def angle_error(target: float, current: float) -> float:
    return math.remainder(target - current, math.tau)


def segment_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.dist(point, start)
    projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    fraction = max(0.0, min(1.0, projection))
    return math.dist(point, (start[0] + fraction * dx, start[1] + fraction * dy))


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

    def update(self, point: Point, heading: float) -> Twist:
        if self.status != "tracking":
            return 0.0, 0.0, 0.0
        if not all(map(math.isfinite, (*point, heading))):
            self.status = "invalid_state"
            return 0.0, 0.0, 0.0
        if not self.map.edge_clear(self.previous, point):
            self.status = "clearance_violation"
            return 0.0, 0.0, 0.0

        self.previous = point
        start = self.path[max(0, self.next_index - 1)]
        target = self.path[self.next_index]
        error = segment_distance(point, start, target)
        self.peak_tracking_error = max(self.peak_tracking_error, error)
        if error > self.max_tracking_error:
            self.status = "tracking_error"
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
