"""MoveIt PlanningScene implementation of the self-collision CollisionBackend.

Uses moveit_py (ros-humble-moveit-py) and MoveIt's default FCL collision
environment. Humble's Python bindings only return the minimum distance over
all allowed pairs, so each pair is measured with its own query: every other
pair is marked allowed in the collision matrix. That is one query per pair,
which makes this backend much slower than one MoveIt query; the benchmark
reports both through min_distance().

Differences from the Pinocchio backends:
- A colliding pair reports 0.0, not a penetration depth: the bindings cannot
  return MoveIt's contact details.
- Meshes are loaded by MoveIt's resource retriever, so package:// URIs need
  their packages installed and sourced; package_dirs is not used.
- MoveIt requires an SRDF. A minimal one with no exemptions is generated, and
  the allowed pairs are applied to the collision matrix here, so exemptions
  match the other backends instead of coming from a MoveIt config package.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from typing import Sequence

import numpy as np
from moveit.core.collision_detection import AllowedCollisionMatrix, CollisionRequest, CollisionResult
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState

from .checker import (
    LinkPair,
    link_pair,
    urdf_collision_link_names,
    urdf_movable_joint_names,
)


class MoveItBackend:
    def __init__(self, urdf_path: str, package_dirs: Sequence[str], allowed_pairs: set[LinkPair]) -> None:
        del package_dirs  # MoveIt resolves package:// URIs itself.
        self.name = "moveit"
        self.joint_names = urdf_movable_joint_names(urdf_path)
        links = urdf_collision_link_names(urdf_path)
        self.link_pairs = tuple(sorted(
            link_pair(first, second)
            for index, first in enumerate(links) for second in links[index + 1:]
            if link_pair(first, second) not in allowed_pairs))

        robot_name = ET.parse(urdf_path).getroot().get("name")
        with tempfile.NamedTemporaryFile("w", suffix=".srdf", delete=False) as srdf:
            srdf.write(f'<?xml version="1.0"?>\n<robot name="{robot_name}"/>\n')
        try:
            self._model = RobotModel(urdf_xml_path=urdf_path, srdf_xml_path=srdf.name)
        finally:
            os.unlink(srdf.name)
        self._scene = PlanningScene(self._model)
        self._state = RobotState(self._model)
        # Every pair allowed; distances() opens one pair at a time.
        self._pair_acm = AllowedCollisionMatrix(links, True)
        # Fixed matrix for min_distance(): only the exempt pairs are allowed.
        self._global_acm = AllowedCollisionMatrix(links, False)
        for first, second in allowed_pairs:
            if first in links and second in links:
                self._global_acm.set_entry(first, second, True)

    def _set_positions(self, positions: np.ndarray) -> None:
        self._state.joint_positions = dict(zip(self.joint_names, map(float, positions)))
        self._state.update()

    def _query(self, acm: AllowedCollisionMatrix) -> float:
        request = CollisionRequest()
        request.distance = True
        result = CollisionResult()
        self._scene.check_self_collision(request, result, self._state, acm)
        # MoveIt reports 0 in collision; no penetration depth is available.
        return 0.0 if result.collision else float(result.distance)

    def distances(self, positions: np.ndarray) -> np.ndarray:
        self._set_positions(positions)
        distances = np.empty(len(self.link_pairs))
        for index, (first, second) in enumerate(self.link_pairs):
            self._pair_acm.set_entry(first, second, False)
            try:
                distances[index] = self._query(self._pair_acm)
            finally:
                self._pair_acm.set_entry(first, second, True)
        return distances

    def min_distance(self, positions: np.ndarray) -> float:
        """One MoveIt query over all checked pairs: the cost MoveIt itself has.

        Not part of CollisionBackend; the benchmark uses it to show what a
        per-pair C++ binding could cost at best.
        """
        self._set_positions(positions)
        return self._query(self._global_acm)
