"""RViz marker geometry from the URDF, and highlighting from a result."""
import math
from pathlib import Path

import numpy as np
import pytest
from visualization_msgs.msg import Marker

from humanoid_safety.self_collision.checker import ClearanceResult
from humanoid_safety.self_collision.markers import (
    CLOSE_COLOR,
    COLLISION_COLOR,
    CollisionShape,
    _quaternion,
    _rotation,
    _shape_marker,
    build_markers,
    urdf_collision_shapes,
)

REPO = Path(__file__).resolve().parents[4]
URDF = str(REPO / 'humanoid_leg_description/urdf/human_offset_corrected.urdf')


@pytest.fixture(scope='module')
def shapes():
    return urdf_collision_shapes(URDF, str(REPO))


def result(colliding=(), close=()):
    return ClearanceResult(-0.01 if colliding else 0.005, (colliding or close)[0],
                           tuple(colliding), tuple(close), 1, 0.0)


def link_markers(shapes, inflation, **pairs):
    first, *links = build_markers(shapes, result(**pairs), inflation).markers
    assert first.action == Marker.DELETEALL
    return {marker.header.frame_id: marker for marker in links}


def test_urdf_collision_shapes(shapes):
    assert len(shapes) == 12
    (torso,) = shapes['torso']
    assert torso.kind == 'box' and torso.dimensions == (0.28, 0.12, 0.096)
    (leg,) = shapes['left_leg1']
    assert leg.kind == 'mesh'
    assert leg.mesh_uri.endswith('robot_meshes/left_leg1_ch.stl')
    assert leg.dimensions == (0.001, 0.001, 0.001)
    assert leg.xyz == (0.068372, 0.029495, -0.577401)
    # Mesh vertices are in millimetres, far from the mesh frame origin.
    assert np.linalg.norm(leg.mesh_center) > 100


def test_colliding_links_are_red_and_close_links_yellow(shapes):
    markers = link_markers(shapes, 1.02, colliding=[('left_leg4', 'right_leg4')],
                           close=[('left_leg4', 'left_foot')])
    # left_leg4 is in both lists; collision wins.
    assert {link: m.color for link, m in markers.items()} == {
        'left_leg4': COLLISION_COLOR, 'right_leg4': COLLISION_COLOR, 'left_foot': CLOSE_COLOR}
    assert all(m.frame_locked and m.action == Marker.ADD for m in markers.values())
    # Opaque: RViz sorts transparent objects as a whole, which flickers.
    assert all(m.color.a == 1.0 for m in markers.values())
    assert len({m.id for m in markers.values()}) == len(markers)
    assert markers['left_foot'].type == Marker.CUBE
    assert markers['left_leg4'].type == Marker.MESH_RESOURCE


def test_no_result_only_clears_markers(shapes):
    array = build_markers(shapes, None)
    assert [m.action for m in array.markers] == [Marker.DELETEALL]


def test_no_inflation_draws_the_urdf_geometry(shapes):
    markers = link_markers(shapes, 1.0, colliding=[('left_leg4', 'left_foot')])
    (leg,) = shapes['left_leg4']
    p = markers['left_leg4'].pose.position
    assert (p.x, p.y, p.z) == pytest.approx(leg.xyz)
    assert markers['left_leg4'].scale.x == pytest.approx(0.001)


def test_inflated_box_grows_in_place(shapes):
    markers = link_markers(shapes, 1.02, colliding=[('left_foot', 'right_foot')])
    (foot,) = shapes['left_foot']
    box = markers['left_foot']
    assert (box.pose.position.x, box.pose.position.y, box.pose.position.z) == pytest.approx(foot.xyz)
    assert (box.scale.x, box.scale.y, box.scale.z) == pytest.approx(
        tuple(1.02 * value for value in foot.dimensions))


def test_inflated_mesh_keeps_its_centre_fixed(shapes):
    """Scaling about the link origin would move the mesh; about its centre it stays put."""
    markers = link_markers(shapes, 1.02, colliding=[('left_leg4', 'right_leg4')])
    for link in ('left_leg4', 'right_leg4'):
        (shape,) = shapes[link]
        marker = markers[link]
        scale = np.array([marker.scale.x, marker.scale.y, marker.scale.z])
        position = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
        drawn_center = position + scale * np.asarray(shape.mesh_center)  # rpy is zero here
        urdf_center = np.asarray(shape.xyz) + np.asarray(shape.dimensions) * np.asarray(shape.mesh_center)
        assert scale == pytest.approx(1.02 * np.asarray(shape.dimensions))
        assert drawn_center == pytest.approx(urdf_center, abs=1e-9)


def test_inflated_mesh_centre_is_fixed_with_rotated_origin():
    shape = CollisionShape('link', 'mesh', (0.1, -0.2, 0.3), (0.3, -0.2, 0.5),
                           (0.001, 0.002, 0.001), 'package://x/y.stl', (150.0, -40.0, 600.0))
    marker = _shape_marker(shape, COLLISION_COLOR, 1, 1.05)
    rotation = _rotation(marker.pose.orientation)
    scale = np.array([marker.scale.x, marker.scale.y, marker.scale.z])
    position = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
    drawn = position + rotation @ (scale * np.asarray(shape.mesh_center))
    expected = np.asarray(shape.xyz) + rotation @ (np.asarray(shape.dimensions) * np.asarray(shape.mesh_center))
    assert drawn == pytest.approx(expected, abs=1e-9)


def test_urdf_rpy_yaw_quaternion():
    q = _quaternion(0.0, 0.0, math.pi / 2)
    assert (q.x, q.y) == pytest.approx((0.0, 0.0))
    assert (q.z, q.w) == pytest.approx((math.sin(math.pi / 4), math.cos(math.pi / 4)))
