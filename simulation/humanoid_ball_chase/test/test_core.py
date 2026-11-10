"""Unit tests for the ROS-independent ball-chase core.

Run from the repository root with::

    python3 -m pytest simulation/humanoid_ball_chase/test/test_core.py -q
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest

from humanoid_ball_chase import core


IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640
CAMERA_K = np.array(
    [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
    dtype=np.float64,
)
ORANGE = (255, 140, 0)  # RGB


def _blank_scene(depth: float = 2.0) -> tuple[np.ndarray, np.ndarray]:
    """Return a black RGB image and a constant valid depth image."""
    rgb = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
    depth = np.full((IMAGE_HEIGHT, IMAGE_WIDTH), depth, dtype=np.float64)
    return rgb, depth


def _draw_orange_ball(rgb: np.ndarray, u: int, v: int, radius: int = 30) -> None:
    cv2.circle(rgb, (u, v), radius, ORANGE, thickness=cv2.FILLED)


# ---------------------------------------------------------------------------
# detect_ball
# ---------------------------------------------------------------------------


def test_detect_centered_ball_returns_expected_3d_point() -> None:
    rgb, depth = _blank_scene(depth=2.0)
    _draw_orange_ball(rgb, 320, 240, radius=40)

    result = core.detect_ball(rgb, depth, CAMERA_K)

    assert result is not None
    u, v, x, y, z = result
    assert u == pytest.approx(320.0, abs=1.0)
    assert v == pytest.approx(240.0, abs=1.0)
    assert z == pytest.approx(2.0, abs=1e-6)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)


def test_detect_off_axis_ball_deprojects_with_intrinsics() -> None:
    rgb, depth = _blank_scene(depth=2.2)
    u0, v0, radius = 400, 300, 35
    _draw_orange_ball(rgb, u0, v0, radius=radius)

    result = core.detect_ball(rgb, depth, CAMERA_K)

    assert result is not None
    u, v, x, y, z = result
    expected_x = (u - CAMERA_K[2]) * z / CAMERA_K[0]
    expected_y = (v - CAMERA_K[5]) * z / CAMERA_K[4]
    assert u == pytest.approx(u0, abs=1.0)
    assert v == pytest.approx(v0, abs=1.0)
    assert z == pytest.approx(2.2)
    assert x == pytest.approx(expected_x)
    assert y == pytest.approx(expected_y)


def test_depth_median_is_robust_to_invalid_holes_and_spikes() -> None:
    rgb, depth = _blank_scene(depth=2.0)
    _draw_orange_ball(rgb, 320, 240, radius=45)

    # Corrupt a small part of the ball interior with zeros, NaN, and outliers.
    depth[230:242, 310:322] = 0.0
    depth[248:254, 315:320] = np.nan
    depth[225:230, 330:340] = 50.0

    result = core.detect_ball(rgb, depth, CAMERA_K)

    assert result is not None
    assert result[4] == pytest.approx(2.0, abs=1e-6)


def test_largest_circular_blob_wins_over_larger_rectangular_board() -> None:
    rgb, depth = _blank_scene(depth=1.8)
    # A large orange rectangular "face board" plus a smaller orange ball.
    cv2.rectangle(rgb, (70, 90), (330, 230), ORANGE, thickness=cv2.FILLED)
    _draw_orange_ball(rgb, 500, 300, radius=28)

    result = core.detect_ball(rgb, depth, CAMERA_K)

    assert result is not None
    assert result[0] == pytest.approx(500.0, abs=1.0)
    assert result[1] == pytest.approx(300.0, abs=1.0)
    assert result[4] == pytest.approx(1.8)


def test_depth_is_sampled_from_ball_not_from_nearby_face_board() -> None:
    rgb, depth = _blank_scene(depth=1.2)
    # The board is closer to the camera than the ball.  Only the ball's
    # eroded contour may contribute to the median depth.
    cv2.rectangle(rgb, (50, 50), (400, 250), ORANGE, thickness=cv2.FILLED)
    _draw_orange_ball(rgb, 500, 320, radius=30)
    depth[50:250, 50:400] = 5.0

    result = core.detect_ball(rgb, depth, CAMERA_K)

    assert result is not None
    assert result[0] == pytest.approx(500.0, abs=1.0)
    assert result[4] == pytest.approx(1.2)


@pytest.mark.parametrize(
    "draw",
    [
        lambda rgb: cv2.rectangle(rgb, (250, 190), (390, 290), ORANGE, -1),
        lambda rgb: cv2.ellipse(rgb, (320, 240), (90, 25), 0, 0, 360, ORANGE, -1),
    ],
)
def test_rectangular_or_elongated_orange_blobs_are_rejected(draw) -> None:
    rgb, depth = _blank_scene(depth=1.5)
    draw(rgb)

    assert core.detect_ball(rgb, depth, CAMERA_K) is None


def test_non_orange_ball_is_not_detected() -> None:
    rgb, depth = _blank_scene(depth=1.5)
    cv2.circle(rgb, (320, 240), 40, (0, 80, 255), thickness=cv2.FILLED)  # blue

    assert core.detect_ball(rgb, depth, CAMERA_K) is None


@pytest.mark.parametrize("bad_depth", [0.05, 20.0, 0.0, -1.0])
def test_too_close_or_out_of_range_depth_is_rejected(bad_depth: float) -> None:
    rgb, depth = _blank_scene(depth=bad_depth)
    _draw_orange_ball(rgb, 320, 240, radius=40)

    assert core.detect_ball(rgb, depth, CAMERA_K) is None


def test_all_invalid_depth_is_rejected_not_zero_distance() -> None:
    rgb, depth = _blank_scene(depth=0.0)
    depth[:] = np.nan
    _draw_orange_ball(rgb, 320, 240, radius=40)

    assert core.detect_ball(rgb, depth, CAMERA_K) is None


@pytest.mark.parametrize(
    "rgb,depth,k",
    [
        (np.zeros((10, 10), dtype=np.uint8), np.zeros((10, 10)), CAMERA_K),
        (np.zeros((10, 10, 4), dtype=np.uint8), np.zeros((10, 10)), CAMERA_K),
        (np.zeros((10, 10, 3), dtype=np.uint16), np.zeros((10, 10)), CAMERA_K),
        (np.zeros((10, 10, 3), dtype=np.uint8), np.zeros((9, 10)), CAMERA_K),
        (np.zeros((10, 10, 3), dtype=np.uint8), np.zeros((10, 10), dtype=np.int32), CAMERA_K),
        (np.zeros((10, 10, 3), dtype=np.uint8), np.zeros((10, 10)), np.zeros(8)),
        (np.zeros((10, 10, 3), dtype=np.uint8), np.zeros((10, 10)), np.zeros(9)),
        (
            np.zeros((10, 10, 3), dtype=np.uint8),
            np.zeros((10, 10)),
            np.array([500.0, 0.0, 5.0, 0.0, 500.0, 5.0, 0.0, 0.0, np.nan]),
        ),
    ],
)
def test_invalid_image_or_intrinsics_returns_none(rgb, depth, k) -> None:
    assert core.detect_ball(rgb, depth, k) is None


def test_accepts_3x3_intrinsics_and_float32_depth() -> None:
    rgb, depth = _blank_scene(depth=1.25)
    _draw_orange_ball(rgb, 320, 240, radius=30)

    result = core.detect_ball(rgb, depth.astype(np.float32), CAMERA_K.reshape(3, 3))

    assert result is not None
    assert result[4] == pytest.approx(1.25, abs=1e-5)


# ---------------------------------------------------------------------------
# chase_command
# ---------------------------------------------------------------------------

# Standard forward-looking camera orientation expressed in base_link:
# optical +x (right) -> base -y, optical +y (down) -> base -z,
# optical +z (forward) -> base +x.
R_OPTICAL_TO_BASE = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)
CAMERA_POSITION_BASE = np.array([0.1, 0.0, 1.0], dtype=np.float64)


def test_aligned_ball_drives_forward_within_limit() -> None:
    forward, yaw = core.chase_command(
        (0.0, 0.0, 2.0),
        R_OPTICAL_TO_BASE,
        CAMERA_POSITION_BASE,
        stop_distance=0.5,
        max_speed=0.2,
        max_yaw_rate=0.5,
    )

    assert forward == pytest.approx(0.2)
    assert yaw == pytest.approx(0.0)


def test_ball_to_left_turns_left_in_place() -> None:
    # Optical (-1, 0, 2) -> base (2.1, +1.0, 1.0): heading > gate.
    forward, yaw = core.chase_command(
        (-1.0, 0.0, 2.0),
        R_OPTICAL_TO_BASE,
        CAMERA_POSITION_BASE,
    )

    assert forward == 0.0
    assert yaw == pytest.approx(0.5)


def test_ball_to_right_turns_right_in_place() -> None:
    # Optical (+1, 0, 2) -> base (2.1, -1.0, 1.0): heading < -gate.
    forward, yaw = core.chase_command(
        (1.0, 0.0, 2.0),
        R_OPTICAL_TO_BASE,
        CAMERA_POSITION_BASE,
    )

    assert forward == 0.0
    assert yaw == pytest.approx(-0.5)


def test_small_heading_error_allows_forward_motion() -> None:
    # Optical (-0.2, 0, 2) -> base (2.1, +0.2, 1.0): heading ~0.095 rad.
    forward, yaw = core.chase_command(
        (-0.2, 0.0, 2.0),
        R_OPTICAL_TO_BASE,
        CAMERA_POSITION_BASE,
    )

    assert forward == pytest.approx(0.2)
    assert 0.0 < yaw < 0.5


def test_near_target_stops_linear_motion() -> None:
    identity = np.eye(3)
    origin = np.zeros(3)

    forward, yaw = core.chase_command(
        (0.3, 0.0, 0.0), identity, origin, stop_distance=0.5
    )

    assert forward == 0.0
    assert yaw == pytest.approx(0.0)


def test_close_off_axis_target_turns_but_does_not_drive() -> None:
    # Planar distance is ~0.424 m, inside the 0.5 m stop distance.
    forward, yaw = core.chase_command(
        (0.3, 0.3, 0.0), np.eye(3), np.zeros(3), stop_distance=0.5
    )

    assert forward == 0.0
    assert yaw > 0.0


def test_heading_gate_is_strictly_forty_degrees() -> None:
    identity = np.eye(3)
    origin = np.zeros(3)
    below_gate = 0.39
    above_gate = 0.41

    forward_below, _ = core.chase_command(
        (math.cos(below_gate), math.sin(below_gate), 0.0), identity, origin
    )
    forward_above, yaw_above = core.chase_command(
        (math.cos(above_gate), math.sin(above_gate), 0.0), identity, origin
    )

    assert forward_below > 0.0
    assert forward_above == 0.0
    assert yaw_above > 0.0


def test_ball_behind_turns_in_place() -> None:
    forward, yaw = core.chase_command(
        (-2.0, 0.0, 0.0), np.eye(3), np.zeros(3)
    )

    assert forward == 0.0
    assert yaw == pytest.approx(0.5)


def test_commands_respect_custom_speed_and_yaw_limits() -> None:
    forward, yaw = core.chase_command(
        (10.0, 0.0, 0.0),
        np.eye(3),
        np.zeros(3),
        max_speed=0.05,
        max_yaw_rate=0.1,
    )
    assert forward == pytest.approx(0.05)
    assert yaw == pytest.approx(0.0)

    forward, yaw = core.chase_command(
        (0.0, 10.0, 0.0),
        np.eye(3),
        np.zeros(3),
        max_speed=0.05,
        max_yaw_rate=0.1,
    )
    assert forward == 0.0
    assert yaw == pytest.approx(0.1)


@pytest.mark.parametrize(
    "point,rotation,position,kwargs",
    [
        ((np.nan, 0.0, 1.0), np.eye(3), np.zeros(3), {}),
        ((0.0, 0.0), np.eye(3), np.zeros(3), {}),
        ((0.0, 0.0, 1.0), np.eye(2), np.zeros(3), {}),
        ((0.0, 0.0, 1.0), np.full((3, 3), np.nan), np.zeros(3), {}),
        ((0.0, 0.0, 1.0), 2.0 * np.eye(3), np.zeros(3), {}),
        ((0.0, 0.0, 1.0), np.diag([1.0, 1.0, -1.0]), np.zeros(3), {}),
        ((0.0, 0.0, 1.0), np.eye(3), (0.0, np.inf, 0.0), {}),
        ((0.0, 0.0, 1.0), np.eye(3), np.zeros(3), {"stop_distance": -0.1}),
        ((0.0, 0.0, 1.0), np.eye(3), np.zeros(3), {"max_speed": -0.1}),
        ((0.0, 0.0, 1.0), np.eye(3), np.zeros(3), {"max_yaw_rate": -0.1}),
        ((0.0, 0.0, 1.0), np.eye(3), np.zeros(3), {"max_speed": np.inf}),
    ],
)
def test_invalid_chase_inputs_return_zero(point, rotation, position, kwargs) -> None:
    assert core.chase_command(point, rotation, position, **kwargs) == (0.0, 0.0)


def test_yaw_deadband_applies_only_when_stopped() -> None:
    # Base (0.4, 0.02): inside 0.5 m, heading 0.05 rad.
    point = (0.4, 0.02, 0.0)
    _, yaw = core.chase_command(point, np.eye(3), np.zeros(3), stop_distance=0.5)
    assert yaw == pytest.approx(1.5 * math.atan2(0.02, 0.4))
    forward, yaw = core.chase_command(point, np.eye(3), np.zeros(3), stop_distance=0.5,
                                      yaw_deadband_rad=0.06)
    assert forward == 0.0 and yaw == 0.0
    # Same heading beyond the stop distance: still steers while walking.
    forward, yaw = core.chase_command((2.0, 0.1, 0.0), np.eye(3), np.zeros(3), stop_distance=0.5,
                                      yaw_deadband_rad=0.06)
    assert forward > 0.0 and yaw > 0.0
    # A larger heading error inside the stop distance still turns.
    _, yaw = core.chase_command((0.4, 0.1, 0.0), np.eye(3), np.zeros(3), stop_distance=0.5,
                                yaw_deadband_rad=0.06)
    assert yaw > 0.0
