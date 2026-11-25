"""RViz markers that highlight the links involved in a self-collision result.

Markers are drawn from the URDF <collision> elements in each link's own TF
frame, so robot_state_publisher must publish TF for the same joint states the
result was computed from. They show what the checker saw; they are not
computed by the backend and work with any backend.

Highlighted shapes are opaque: RViz sorts transparent objects as a whole,
not per pixel, so overlapping transparent links flicker as the camera moves.
They are drawn slightly inflated about their own centre so they enclose the
link's visual mesh; many hull faces lie exactly on it, and coincident opaque
surfaces would z-fight. Inflation only changes the drawing.
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .checker import ClearanceResult


@dataclass(frozen=True)
class CollisionShape:
    link: str
    kind: str  # "mesh", "box", "cylinder" or "sphere"
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    # mesh: scale; box: size; cylinder: (radius, length); sphere: (radius,)
    dimensions: tuple[float, ...]
    mesh_uri: str = ""
    # Mean mesh vertex in unscaled mesh units; inflation is about this point.
    mesh_center: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _floats(text: Optional[str], default: str) -> tuple[float, ...]:
    return tuple(float(value) for value in (text or default).split())


def resolve_mesh_path(uri: str, package_dir: str) -> str:
    """package://pkg/file under package_dir (the directory holding pkg), or file://."""
    if uri.startswith("package://"):
        return os.path.join(package_dir, uri[len("package://"):])
    if uri.startswith("file://"):
        return uri[len("file://"):]
    return uri


def stl_vertices(path: str) -> np.ndarray:
    """Unique vertices of a binary or ASCII STL file, in file units."""
    with open(path, "rb") as stream:
        data = stream.read()
    if data[:5] == b"solid" and b"facet" in data[:512]:
        rows = [line.split()[1:4] for line in data.decode(errors="replace").splitlines()
                if line.strip().startswith("vertex")]
        vertices = np.asarray(rows, dtype=float)
    else:
        count = int.from_bytes(data[80:84], "little")
        triangle = np.dtype([("normal", "<3f4"), ("vertices", "<9f4"), ("attribute", "<u2")])
        triangles = np.frombuffer(data, dtype=triangle, count=count, offset=84)
        vertices = triangles["vertices"].reshape(-1, 3).astype(float)
    if vertices.size == 0:
        raise ValueError(f"No vertices in STL file {path}")
    return np.unique(vertices, axis=0)


def urdf_collision_shapes(urdf_path: str, package_dir: str) -> dict[str, list[CollisionShape]]:
    """Collision geometry per link, in metres and radians as the URDF states it.

    Mesh files are read once to find their centres, resolving package:// URIs
    under package_dir.
    """
    shapes: dict[str, list[CollisionShape]] = {}
    for link in ET.parse(urdf_path).getroot().findall("link"):
        for collision in link.findall("collision"):
            origin = collision.find("origin")
            xyz = _floats(None if origin is None else origin.get("xyz"), "0 0 0")
            rpy = _floats(None if origin is None else origin.get("rpy"), "0 0 0")
            geometry = collision.find("geometry")
            mesh, box = geometry.find("mesh"), geometry.find("box")
            cylinder, sphere = geometry.find("cylinder"), geometry.find("sphere")
            if mesh is not None:
                uri = mesh.get("filename")
                center = stl_vertices(resolve_mesh_path(uri, package_dir)).mean(axis=0)
                shape = CollisionShape(link.get("name"), "mesh", xyz, rpy,
                                       _floats(mesh.get("scale"), "1 1 1"), uri, tuple(center))
            elif box is not None:
                shape = CollisionShape(link.get("name"), "box", xyz, rpy, _floats(box.get("size"), ""))
            elif cylinder is not None:
                shape = CollisionShape(link.get("name"), "cylinder", xyz, rpy,
                                       (float(cylinder.get("radius")), float(cylinder.get("length"))))
            elif sphere is not None:
                shape = CollisionShape(link.get("name"), "sphere", xyz, rpy, (float(sphere.get("radius")),))
            else:
                raise ValueError(f"Unsupported collision geometry on link {link.get('name')}")
            shapes.setdefault(shape.link, []).append(shape)
    return shapes


def _quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """URDF rpy: fixed-axis X, then Y, then Z."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(x=sr * cp * cy - cr * sp * sy, y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy, w=cr * cp * cy + sr * sp * sy)


COLLISION_COLOR = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
CLOSE_COLOR = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)


def _rotation(q: Quaternion) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _shape_marker(shape: CollisionShape, color: ColorRGBA, marker_id: int, inflation: float) -> Marker:
    marker = Marker(ns="self_collision/links", id=marker_id, action=Marker.ADD, color=color)
    marker.header.frame_id = shape.link
    marker.frame_locked = True  # Follow the link's TF as it moves.
    orientation = _quaternion(*shape.rpy)
    position = np.asarray(shape.xyz, dtype=float)
    if shape.kind == "mesh":
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = shape.mesh_uri
        # A mesh is drawn as origin + R * (scale * vertex). Scaling it by
        # `inflation` about its centre c keeps origin + R * (scale * c) fixed.
        center = np.asarray(shape.dimensions) * np.asarray(shape.mesh_center)
        position = position + _rotation(orientation) @ ((1.0 - inflation) * center)
        dims = shape.dimensions
    elif shape.kind == "box":
        marker.type = Marker.CUBE
        dims = shape.dimensions
    elif shape.kind == "cylinder":
        marker.type = Marker.CYLINDER
        radius, length = shape.dimensions
        dims = (2 * radius, 2 * radius, length)
    else:
        marker.type = Marker.SPHERE
        dims = (2 * shape.dimensions[0],) * 3
    # Primitive markers are centred on their pose, so they inflate in place.
    marker.pose = Pose(position=Point(x=position[0], y=position[1], z=position[2]),
                       orientation=orientation)
    marker.scale.x, marker.scale.y, marker.scale.z = (inflation * value for value in dims)
    return marker


def build_markers(
    shapes: dict[str, list[CollisionShape]],
    result: Optional[ClearanceResult],
    inflation: float = 1.02,
) -> MarkerArray:
    """Red links touch, yellow links are within the margin; others are not drawn.

    inflation enlarges each drawn shape about its centre (1.02 = 2%) so it
    does not coincide with the visual mesh; it does not affect any check.

    The array starts with DELETEALL so links that are no longer flagged
    disappear, including when there is no result.
    """
    markers = [Marker(action=Marker.DELETEALL)]
    if result is not None:
        flagged = {}
        for pair in result.close_pairs:
            flagged.update(dict.fromkeys(pair, CLOSE_COLOR))
        for pair in result.colliding_pairs:  # Collision wins over close.
            flagged.update(dict.fromkeys(pair, COLLISION_COLOR))
        for link, color in sorted(flagged.items()):
            for shape in shapes.get(link, []):
                markers.append(_shape_marker(shape, color, len(markers), inflation))
    return MarkerArray(markers=markers)
