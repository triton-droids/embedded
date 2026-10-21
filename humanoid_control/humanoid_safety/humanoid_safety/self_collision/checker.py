"""Advisory self-collision and minimum-distance checks for leg joint poses.

This module is backend-independent and does not depend on ROS. A
CollisionBackend computes link-pair distances for one joint pose; this module
validates inputs, samples paths between poses, checks input freshness, and
classifies results.

Results are warnings only: they never change commands. A sampled path check
only covers the poses it sampled, so a clear result does not prove the
continuous motion between them is collision-free. Ground and environment
contact are out of scope; this only checks the robot against itself.

Units: joint positions in radians, distances in metres, times in seconds.
"""

from __future__ import annotations

import importlib
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Iterable, Mapping, Optional, Protocol, Sequence

import numpy as np


LinkPair = tuple[str, str]


def link_pair(first: str, second: str) -> LinkPair:
    """Order-independent key for a pair of links."""
    return (first, second) if first <= second else (second, first)


class CollisionBackend(Protocol):
    """Computes distances between the checked link pairs for one pose.

    joint_names: the order of the positions passed to distances().
    link_pairs: the pairs that are checked, aligned with distances().
    distances(): one value per link pair in metres. Negative values are
        penetration depth; zero or negative means the pair is in collision.
    """

    name: str
    joint_names: tuple[str, ...]
    link_pairs: tuple[LinkPair, ...]

    def distances(self, positions: np.ndarray) -> np.ndarray:
        ...


# Backends are imported lazily so that selecting one never requires the
# others' libraries. To add a backend for benchmarking, implement
# CollisionBackend and register it here.
BACKENDS = {
    # Coal GJK on convex shapes. Exact for the current *_ch.stl collision
    # meshes, which are already convex hulls; an approximation otherwise.
    "pinocchio": ("humanoid_safety.self_collision.pinocchio_backend", "PinocchioBackend",
                  {"use_convex": True}),
    # Coal BVH mesh-to-mesh distance. Slower; useful as a reference.
    "pinocchio_mesh": ("humanoid_safety.self_collision.pinocchio_backend", "PinocchioBackend",
                       {"use_convex": False}),
    # MoveIt PlanningScene (FCL) through moveit_py, one masked query per
    # pair. Slow: Humble's Python bindings expose no per-pair distances.
    "moveit": ("humanoid_safety.self_collision.moveit_backend", "MoveItBackend", {}),
}


def create_backend(
    name: str,
    urdf_path: str,
    package_dirs: Sequence[str],
    allowed_pairs: Iterable[LinkPair],
) -> CollisionBackend:
    """Build a registered backend. allowed_pairs are never checked."""
    if name not in BACKENDS:
        raise ValueError(f"Unknown collision backend {name!r}; choose from {sorted(BACKENDS)}")
    module_name, class_name, options = BACKENDS[name]
    backend_class = getattr(importlib.import_module(module_name), class_name)
    return backend_class(urdf_path, package_dirs, set(allowed_pairs), **options)


def urdf_adjacent_link_pairs(urdf_path: str) -> set[LinkPair]:
    """Parent/child link pairs of every URDF joint, including fixed joints.

    Adjacent links touch at their joint by construction, so they are exempt.
    This matches the "Adjacent" entries MoveIt writes into an SRDF.
    """
    return {
        link_pair(joint.find("parent").get("link"), joint.find("child").get("link"))
        for joint in _kinematic_joints(urdf_path)
    }


def _kinematic_joints(urdf_path: str) -> list[ET.Element]:
    # Only direct <robot> children: <ros2_control> blocks also contain <joint>.
    return ET.parse(urdf_path).getroot().findall("joint")


def urdf_link_names(urdf_path: str) -> set[str]:
    return {link.get("name") for link in ET.parse(urdf_path).getroot().findall("link")}


def urdf_collision_link_names(urdf_path: str) -> list[str]:
    """Links with at least one <collision> element, in URDF order."""
    return [link.get("name") for link in ET.parse(urdf_path).getroot().findall("link")
            if link.find("collision") is not None]


def urdf_movable_joint_names(urdf_path: str) -> tuple[str, ...]:
    """Non-fixed joints, in URDF order."""
    return tuple(joint.get("name") for joint in _kinematic_joints(urdf_path)
                 if joint.get("type") != "fixed")


def urdf_joint_limits(urdf_path: str, joint_names: Sequence[str]) -> tuple[np.ndarray, np.ndarray]:
    """Lower and upper position limits (radians) for the named URDF joints."""
    joints = {joint.get("name"): joint for joint in _kinematic_joints(urdf_path)}
    lower, upper = [], []
    for name in joint_names:
        limit = joints[name].find("limit")
        lower.append(float(limit.get("lower")))
        upper.append(float(limit.get("upper")))
    return np.asarray(lower), np.asarray(upper)


def srdf_disabled_link_pairs(srdf_path: str) -> set[LinkPair]:
    """Pairs listed as <disable_collisions> in a MoveIt SRDF."""
    root = ET.parse(srdf_path).getroot()
    return {
        link_pair(entry.get("link1"), entry.get("link2"))
        for entry in root.iter("disable_collisions")
    }


def parse_link_pairs(text: str) -> set[LinkPair]:
    """Parse "link_a:link_b, link_c:link_d" into link pairs."""
    pairs = set()
    for entry in text.split(","):
        entry = entry.strip()
        if not entry:
            continue
        names = [name.strip() for name in entry.split(":")]
        if len(names) != 2 or not all(names):
            raise ValueError(f"Invalid link pair {entry!r}; expected link_a:link_b")
        pairs.add(link_pair(*names))
    return pairs


@dataclass(frozen=True)
class ClearanceResult:
    min_distance_m: float
    closest_pair: LinkPair
    colliding_pairs: tuple[LinkPair, ...]  # distance <= 0
    close_pairs: tuple[LinkPair, ...]  # 0 < distance < clearance margin
    poses_checked: int
    compute_s: float

    @property
    def level(self) -> str:
        if self.colliding_pairs:
            return "collision"
        if self.close_pairs:
            return "close"
        return "clear"


class SelfCollisionChecker:
    """Classify backend distances against a clearance margin."""

    def __init__(self, backend: CollisionBackend, margin_m: float) -> None:
        if not math.isfinite(margin_m) or margin_m < 0.0:
            raise ValueError("Clearance margin must be finite and nonnegative")
        if not backend.link_pairs:
            raise ValueError("Collision backend has no link pairs to check")
        self.backend = backend
        self.margin_m = margin_m

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.backend.joint_names

    def positions_from(self, positions_by_name: Mapping[str, float]) -> np.ndarray:
        """Joint positions in backend order; every joint must be present and finite."""
        missing = [name for name in self.joint_names if name not in positions_by_name]
        if missing:
            raise ValueError(f"Missing joints: {', '.join(missing)}")
        positions = np.asarray([positions_by_name[name] for name in self.joint_names], dtype=float)
        if not np.isfinite(positions).all():
            raise ValueError("Joint positions must be finite")
        return positions

    def check_pose(self, positions_by_name: Mapping[str, float]) -> ClearanceResult:
        started = time.perf_counter()
        distances = self.backend.distances(self.positions_from(positions_by_name))
        return self._classify(distances, 1, started)

    def check_path(
        self,
        start_by_name: Mapping[str, float],
        end_by_name: Mapping[str, float],
        max_joint_step_rad: float,
        max_poses: int,
    ) -> ClearanceResult:
        """Worst distance per pair over poses sampled from start to end.

        Samples are evenly spaced in joint space, including both endpoints,
        so that no joint moves more than max_joint_step_rad between samples.
        If that needs more than max_poses samples, max_poses are used and the
        step is larger; poses_checked reports what was actually checked.
        The straight joint-space line is an approximation of the real motion.
        """
        if not math.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0.0:
            raise ValueError("max_joint_step_rad must be finite and positive")
        if max_poses < 2:
            raise ValueError("max_poses must be at least 2")
        started = time.perf_counter()
        start = self.positions_from(start_by_name)
        end = self.positions_from(end_by_name)
        largest_step = float(np.max(np.abs(end - start)))
        intervals = max(1, math.ceil(largest_step / max_joint_step_rad))
        intervals = min(intervals, max_poses - 1)
        worst = np.full(len(self.backend.link_pairs), math.inf)
        for fraction in np.linspace(0.0, 1.0, intervals + 1):
            pose = start + fraction * (end - start)
            worst = np.minimum(worst, self.backend.distances(pose))
        return self._classify(worst, intervals + 1, started)

    def _classify(self, distances: np.ndarray, poses: int, started: float) -> ClearanceResult:
        pairs = self.backend.link_pairs
        if distances.shape != (len(pairs),) or np.isnan(distances).any():
            raise ValueError("Collision backend returned invalid distances")
        closest = int(np.argmin(distances))
        return ClearanceResult(
            min_distance_m=float(distances[closest]),
            closest_pair=pairs[closest],
            colliding_pairs=tuple(p for p, d in zip(pairs, distances) if d <= 0.0),
            close_pairs=tuple(p for p, d in zip(pairs, distances) if 0.0 < d < self.margin_m),
            poses_checked=poses,
            compute_s=time.perf_counter() - started,
        )


@dataclass(frozen=True)
class PoseSample:
    """The latest pose from one topic, with host receipt and source times."""

    positions: Mapping[str, float]
    received_at: float  # host monotonic seconds
    stamp: float  # source ROS time in seconds


@dataclass(frozen=True)
class Report:
    check: str  # "measured", "target" or "path"
    result: Optional[ClearanceResult]
    input_age_s: Optional[float]
    problem: str = ""  # why there is no result


def sample_problem(
    sample: Optional[PoseSample],
    joint_names: Sequence[str],
    now_monotonic: float,
    now_ros: float,
    timeout_s: float,
) -> str:
    """Empty if the sample is usable; otherwise why it is not."""
    if sample is None:
        return "no input received"
    receipt_age = now_monotonic - sample.received_at
    source_age = now_ros - sample.stamp
    if sample.stamp <= 0.0 or not -0.01 <= source_age <= timeout_s:
        return f"stale or unstamped input (source age {source_age:.3f} s)"
    if not 0.0 <= receipt_age <= timeout_s:
        return f"stale input (received {receipt_age:.3f} s ago)"
    missing = [name for name in joint_names if name not in sample.positions]
    if missing:
        return f"incomplete input, missing: {', '.join(missing)}"
    return ""


def evaluate(
    checker: SelfCollisionChecker,
    measured: Optional[PoseSample],
    target: Optional[PoseSample],
    now_monotonic: float,
    now_ros: float,
    timeout_s: float,
    max_joint_step_rad: float,
    max_path_poses: int,
) -> list[Report]:
    """Check the measured pose, the target pose and the path between them.

    Stale or incomplete inputs produce a report with a problem and no
    result, never a clearance figure computed from old data.
    """
    def run(check, samples, compute):
        problems = [sample_problem(s, checker.joint_names, now_monotonic, now_ros, timeout_s)
                    for s in samples]
        problem = "; ".join(p for p in problems if p)
        if problem:
            return Report(check, None, None, problem)
        age = max(now_ros - s.stamp for s in samples)
        try:
            return Report(check, compute(), age)
        except Exception as error:  # A monitor failure must be visible, not fatal.
            return Report(check, None, age, f"check failed: {error}")

    return [
        run("measured", [measured], lambda: checker.check_pose(measured.positions)),
        run("target", [target], lambda: checker.check_pose(target.positions)),
        run("path", [measured, target], lambda: checker.check_path(
            measured.positions, target.positions, max_joint_step_rad, max_path_poses)),
    ]
