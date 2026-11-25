"""Pinocchio + Coal implementation of the self-collision CollisionBackend.

The model has a fixed base: self-collision only depends on the joint angles,
not on where the robot is in the world.
"""

from __future__ import annotations

from typing import Sequence

import coal
import numpy as np
import pinocchio as pin

from .checker import LinkPair, link_pair


class PinocchioBackend:
    def __init__(
        self,
        urdf_path: str,
        package_dirs: Sequence[str],
        allowed_pairs: set[LinkPair],
        use_convex: bool = True,
    ) -> None:
        self.name = "pinocchio" if use_convex else "pinocchio_mesh"
        model = pin.buildModelFromUrdf(urdf_path)
        geometry = pin.buildGeomFromUrdf(
            model, urdf_path, pin.GeometryType.COLLISION, package_dirs=list(package_dirs))

        joints = [model.joints[index] for index in range(1, model.njoints)]
        if any(joint.nq != 1 for joint in joints):
            raise ValueError("Only single-axis revolute/prismatic URDF joints are supported")
        self.joint_names = tuple(model.names[index] for index in range(1, model.njoints))
        self._q_index = np.asarray([joint.idx_q for joint in joints])

        if use_convex:
            # Replace each mesh with its convex hull. This is exact only for
            # meshes that are already convex, such as the *_ch.stl files.
            for item in geometry.geometryObjects:
                if isinstance(item.geometry, coal.BVHModelBase):
                    item.geometry.buildConvexRepresentation(False)
                    item.geometry = item.geometry.convex

        # One link can own several geometries; a link pair's distance is the
        # minimum over all of its geometry pairs.
        links = [model.frames[item.parentFrame].name for item in geometry.geometryObjects]
        geometry_pairs = []
        for first in range(len(links)):
            for second in range(first + 1, len(links)):
                pair = link_pair(links[first], links[second])
                if links[first] != links[second] and pair not in allowed_pairs:
                    geometry.addCollisionPair(pin.CollisionPair(first, second))
                    geometry_pairs.append(pair)
        self.link_pairs = tuple(sorted(set(geometry_pairs)))
        position = {pair: index for index, pair in enumerate(self.link_pairs)}
        self._pair_index = np.asarray([position[pair] for pair in geometry_pairs])

        self._model = model
        self._data = model.createData()
        self._geometry = geometry
        self._geometry_data = pin.GeometryData(geometry)

    def distances(self, positions: np.ndarray) -> np.ndarray:
        q = pin.neutral(self._model)
        q[self._q_index] = positions
        pin.computeDistances(self._model, self._data, self._geometry, self._geometry_data, q)
        # distanceResults is aligned with the collision pairs added above.
        geometry_distances = np.fromiter(
            (result.min_distance for result in self._geometry_data.distanceResults),
            dtype=float, count=len(self._pair_index))
        distances = np.full(len(self.link_pairs), np.inf)
        np.minimum.at(distances, self._pair_index, geometry_distances)
        return distances
