"""Plan a clearance-checked XY route on a static rectangle map with RRT*."""

import argparse
import json
import math
import random
from dataclasses import dataclass, field, replace
from pathlib import Path

Point = tuple[float, float]
Rectangle = tuple[float, float, float, float]
DEFAULT_TRACKING_BUFFER = 0.25  # meters beyond the execution clearance


def _offset(box: Rectangle, margin: float) -> Rectangle:
    xmin, ymin, xmax, ymax = box
    return xmin - margin, ymin - margin, xmax + margin, ymax + margin


def _intersects(box: Rectangle, start: Point, end: Point) -> bool:
    """Clip the segment to the box, counting boundary contact."""
    enter, leave = 0.0, 1.0
    for a, b, low, high in zip(start, end, box[:2], box[2:], strict=True):
        if a == b:
            if not low <= a <= high:
                return False
            continue
        first, second = sorted(((low - a) / (b - a), (high - a) / (b - a)))
        enter, leave = max(enter, first), min(leave, second)
        if enter > leave:
            return False
    return True


def _coordinates(value, size: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size or not all(
        type(item) in (int, float) and math.isfinite(item) for item in value
    ):
        raise ValueError(f"{name} must have {size} finite coordinates")
    return tuple(value)


def _rectangle(value, name: str) -> Rectangle:
    xmin, ymin, xmax, ymax = _coordinates(value, 4, name)
    if xmin >= xmax or ymin >= ymax:
        raise ValueError(f"{name} must have positive width and height")
    return xmin, ymin, xmax, ymax


@dataclass(frozen=True)
class PlanningMap:
    bounds: Rectangle
    obstacles: tuple[Rectangle, ...]
    start: Point
    goal: Point
    clearance: float

    @classmethod
    def load(cls, path: Path, clearance: float) -> "PlanningMap":
        data = json.loads(path.read_text())
        if not isinstance(data, dict) or not isinstance(data.get("obstacles", []), list):
            raise ValueError("expected a map object with a list of obstacles")
        sx, sy = _coordinates(data.get("start"), 2, "start")
        gx, gy = _coordinates(data.get("goal"), 2, "goal")
        return cls(
            _rectangle(data.get("bounds"), "bounds"),
            tuple(_rectangle(box, f"obstacles[{i}]") for i, box in enumerate(data.get("obstacles", []))),
            (sx, sy), (gx, gy), clearance,
        )

    def __post_init__(self) -> None:
        if not math.isfinite(self.clearance) or self.clearance < 0:
            raise ValueError("clearance must be finite and nonnegative")
        xmin, ymin, xmax, ymax = self.free_bounds
        if xmin >= xmax or ymin >= ymax:
            raise ValueError("clearance leaves no interior in the map")
        for name, point in (("start", self.start), ("goal", self.goal)):
            if not self.edge_clear(point, point):
                raise ValueError(f"{name} violates map clearance")

    @property
    def free_bounds(self) -> Rectangle:
        return _offset(self.bounds, -self.clearance)

    def edge_clear(self, start: Point, end: Point) -> bool:
        xmin, ymin, xmax, ymax = self.free_bounds
        return (
            all(xmin <= x <= xmax and ymin <= y <= ymax for x, y in (start, end))
            and not any(_intersects(_offset(box, self.clearance), start, end) for box in self.obstacles)
        )


@dataclass(eq=False)
class _Node:
    point: Point
    cost: float = 0.0
    parent: "_Node | None" = None
    children: set["_Node"] = field(default_factory=set)

    def attach(self, parent: "_Node", cost: float) -> None:
        if self.parent is not None:
            self.parent.children.remove(self)
        self.parent = parent
        parent.children.add(self)
        change = cost - self.cost
        self.cost = cost
        pending = list(self.children)
        while pending:
            child = pending.pop()
            child.cost += change
            pending.extend(child.children)


def plan_path(
    map_: PlanningMap, *, step_size: float = 0.5, iterations: int = 2000, seed: int = 42,
    tracking_buffer: float = DEFAULT_TRACKING_BUFFER,
) -> list[Point] | None:
    """Choose lower-cost parents, rewire neighbors, and return the best goal connection."""
    if not math.isfinite(step_size) or step_size <= 0 or iterations < 1:
        raise ValueError("step size and iteration budget must be positive")
    if not math.isfinite(tracking_buffer) or tracking_buffer < 0:
        raise ValueError("tracking buffer must be finite and nonnegative")
    try:
        map_ = replace(map_, clearance=map_.clearance + tracking_buffer)
    except ValueError as exc:
        raise ValueError(f"with the {tracking_buffer:g} m tracking buffer, {exc}") from exc
    start, goal = map_.start, map_.goal
    if start == goal:
        return [start]
    if map_.edge_clear(start, goal):
        return [start, goal]

    xmin, ymin, xmax, ymax = map_.free_bounds
    gamma = 2.0 * math.sqrt((xmax - xmin) * (ymax - ymin) / math.pi)
    rng = random.Random(seed)
    nodes = [_Node(start)]
    points = {start}
    goal_connections: list[tuple[_Node, float]] = []
    for _ in range(iterations):
        sample = goal if rng.random() < 0.1 else (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))
        nearest = min(nodes, key=lambda node: math.dist(node.point, sample))
        distance = math.dist(nearest.point, sample)
        point = sample
        if distance > step_size:
            fraction = step_size / distance
            x, y = nearest.point
            point = (x + fraction * (sample[0] - x), y + fraction * (sample[1] - y))
        if point in points or not map_.edge_clear(nearest.point, point):
            continue

        count = len(nodes) + 1
        radius = min(step_size, gamma * math.sqrt(math.log(count) / count))
        neighbors = []
        for node in nodes:
            length = math.dist(node.point, point)
            if length <= radius and map_.edge_clear(node.point, point):
                neighbors.append((node, length))

        parent, length = min(
            [(nearest, math.dist(nearest.point, point)), *neighbors],
            key=lambda candidate: candidate[0].cost + candidate[1],
        )
        new = _Node(point)
        new.attach(parent, parent.cost + length)
        nodes.append(new)
        points.add(point)
        for neighbor, length in neighbors:
            cost = new.cost + length
            if cost + 1e-12 < neighbor.cost:
                neighbor.attach(new, cost)

        goal_distance = math.dist(point, goal)
        if goal_distance <= step_size and map_.edge_clear(point, goal):
            goal_connections.append((new, goal_distance))

    if not goal_connections:
        return None
    last, distance = min(goal_connections, key=lambda candidate: candidate[0].cost + candidate[1])
    path = [goal] if distance else []
    node: _Node | None = last
    while node is not None:
        path.append(node.point)
        node = node.parent
    return path[::-1]


def add_planning_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("map", type=Path, help="JSON map with bounds, obstacles, start and goal in meters")
    parser.add_argument("--clearance", type=float, required=True, help="Footprint radius plus margin, in meters")
    parser.add_argument(
        "--tracking-buffer", type=float, default=DEFAULT_TRACKING_BUFFER,
        help="Extra planning clearance from obstacles and map edges in meters (default: %(default)s)",
    )
    parser.add_argument("--step-size", type=float, default=0.5)
    parser.add_argument("--iterations", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=42)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_planning_args(parser)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        map_ = PlanningMap.load(args.map, args.clearance)
        path = plan_path(
            map_, step_size=args.step_size, iterations=args.iterations, seed=args.seed,
            tracking_buffer=args.tracking_buffer,
        )
        encoded = json.dumps({
            "status": "planned" if path is not None else "search_exhausted",
            "waypoints_xy_m": path or [],
            "length_m": math.fsum(math.dist(a, b) for a, b in zip(path, path[1:])) if path else None,
            "clearance_m": map_.clearance,
            "tracking_buffer_m": args.tracking_buffer,
            "planning_clearance_m": map_.clearance + args.tracking_buffer,
        }, indent=2)
        if args.output:
            args.output.write_text(encoded + "\n")
        else:
            print(encoded)
    except (OSError, ValueError, OverflowError) as exc:
        parser.error(str(exc))
    return 0 if path is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
