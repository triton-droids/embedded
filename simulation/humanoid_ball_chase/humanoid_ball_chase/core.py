"""ROS-independent perception and ball-chase primitives.

This module deliberately contains no ROS types or imports.  It operates on
NumPy arrays and OpenCV so it can be unit tested without a simulator or robot.

Coordinate conventions
----------------------
* Image coordinates ``u`` (column) and ``v`` (row) have their origin at the
  top-left pixel.  Intrinsics use the standard pinhole model.
* Camera optical coordinates follow the ROS optical convention:
  ``+x`` right, ``+y`` down, ``+z`` forward (metres).
* Robot base coordinates follow ROS ``base_link``:
  ``+x`` forward, ``+y`` left, ``+z`` up (metres).

The caller supplies ``camera_rotation_base`` as ``R_base_camera`` (the camera
orientation expressed in the base frame).  A point is transformed as::

    p_base = R_base_camera @ p_optical + p_camera_base

All lengths are metres and all angles are radians.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import cv2
import numpy as np

# Orange segmentation thresholds in OpenCV's 8-bit HSV convention:
# H in [0, 179], S and V in [0, 255].  These bounds include red-orange while
# excluding most red, yellow, and unsaturated highlights.
ORANGE_HUE_MIN = 5
ORANGE_HUE_MAX = 30
ORANGE_SATURATION_MIN = 90
ORANGE_VALUE_MIN = 50

# A blob must be both large enough to be a ball and round enough to reject the
# rectangular orange face boards used elsewhere in the project.
MIN_BLOB_AREA_PX = 100.0
MIN_CIRCULARITY = 0.75      # 4*pi*area / perimeter^2 (a circle is 1.0)
MIN_ENCLOSING_FILL = 0.70   # area / area(min enclosing circle); circle ~0.84+
EROSION_KERNEL_SIZE = 3

# Valid depth range for this task.  Limits prevent a bogus zero/negative depth
# or a far background return from becoming a target.
MIN_DEPTH_M = 0.10
MAX_DEPTH_M = 10.0
MIN_VALID_DEPTH_PIXELS = 5

# Chase controller tuning.  The heading gate is part of the public behaviour:
# outside it the robot turns in place instead of walking while misaligned.
HEADING_GATE_RAD = 0.40
HEADING_GAIN = 1.50    # yaw rate command per radian of heading error
DISTANCE_GAIN = 1.00   # forward speed command per metre beyond stop distance


def _valid_rotation(rotation: np.ndarray) -> bool:
    """Return True for a finite, right-handed, orthonormal 3x3 matrix."""
    identity = np.eye(3)
    with np.errstate(over="ignore", invalid="ignore"):
        gram = rotation @ rotation.T
        determinant = float(np.linalg.det(rotation))
    if not np.allclose(gram, identity, rtol=0.0, atol=1e-3):
        return False
    return math.isclose(determinant, 1.0, abs_tol=1e-3)


def _finite_scalar(value: object) -> Optional[float]:
    """Convert *value* to a finite float, or return None."""
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _finite_vector(value: object, size: int) -> Optional[np.ndarray]:
    """Return *value* as a finite float vector of exactly *size* elements."""
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return None
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        return None
    return result


def _finite_matrix(value: object, shape: Tuple[int, int]) -> Optional[np.ndarray]:
    """Return *value* as a finite float matrix of exactly *shape*."""
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return None
    if result.shape != shape or not np.all(np.isfinite(result)):
        return None
    return result


def detect_ball(
    rgb: np.ndarray,
    depth_m: np.ndarray,
    k: np.ndarray,
) -> Optional[Tuple[float, float, float, float, float]]:
    """Detect the largest orange, circular ball and return its 3D position.

    Parameters
    ----------
    rgb:
        ``H x W x 3`` ``uint8`` RGB image.
    depth_m:
        ``H x W`` floating-point depth image in metres, aligned to *rgb*.
        Non-finite, zero, negative, and out-of-range samples are ignored.
    k:
        Flat length-9 camera intrinsic matrix in row-major order
        ``[fx, 0, cx, 0, fy, cy, 0, 0, 1]``.  A ``3 x 3`` array is also
        accepted for convenience.

    Returns
    -------
    ``(u, v, x, y, z)`` or ``None``
        ``u, v`` are the blob centroid in image pixels.  ``x, y, z`` are in
        the camera optical frame in metres.  ``None`` is returned for invalid
        inputs, no sufficiently large/circular blob, no valid depth, or a
        depth outside the configured range.
    """
    # --- Validate image and depth dimensions/types before touching OpenCV. ---
    if not isinstance(rgb, np.ndarray) or rgb.ndim != 3 or rgb.shape[2] != 3:
        return None
    if rgb.dtype != np.uint8 or rgb.shape[0] == 0 or rgb.shape[1] == 0:
        return None

    height, width = rgb.shape[:2]
    if not isinstance(depth_m, np.ndarray) or depth_m.ndim != 2:
        return None
    if depth_m.shape != (height, width):
        return None
    if not np.issubdtype(depth_m.dtype, np.floating):
        return None

    # --- Validate intrinsics.  Only fx/fy are required for deprojection, but
    # all entries must be finite to catch a corrupt calibration message. ---
    try:
        intrinsics = np.asarray(k, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return None
    if intrinsics.size != 9 or not np.all(np.isfinite(intrinsics)):
        return None
    intrinsics = intrinsics.reshape(3, 3)
    fx = float(intrinsics[0, 0])
    fy = float(intrinsics[1, 1])
    cx = float(intrinsics[0, 2])
    cy = float(intrinsics[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        return None

    # --- Segment orange, then keep only the largest circular contour.  Walk
    # contours by descending area so a larger rectangular board cannot hide a
    # smaller valid ball. ---
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(
        hsv,
        (ORANGE_HUE_MIN, ORANGE_SATURATION_MIN, ORANGE_VALUE_MIN),
        (ORANGE_HUE_MAX, 255, 255),
    )
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    ball_contour: Optional[np.ndarray] = None
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < MIN_BLOB_AREA_PX:
            # Contours are sorted, so every remaining contour is smaller too.
            break

        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < MIN_CIRCULARITY:
            continue

        (_, _), enclosing_radius = cv2.minEnclosingCircle(contour)
        if enclosing_radius <= 0.0:
            continue
        enclosing_area = math.pi * enclosing_radius * enclosing_radius
        if area / enclosing_area < MIN_ENCLOSING_FILL:
            continue

        ball_contour = contour
        break

    if ball_contour is None:
        return None

    moments = cv2.moments(ball_contour)
    if moments["m00"] <= 0.0:
        return None
    u = float(moments["m10"] / moments["m00"])
    v = float(moments["m01"] / moments["m00"])
    if not (0.0 <= u < width and 0.0 <= v < height):
        return None

    # Limit the depth region to the selected ball and erode its border.  Pixel
    # edges mix ball and background/background depth and are therefore unsafe.
    ball_mask = np.zeros_like(mask)
    cv2.drawContours(ball_mask, [ball_contour], -1, 255, thickness=cv2.FILLED)
    kernel = np.ones((EROSION_KERNEL_SIZE, EROSION_KERNEL_SIZE), np.uint8)
    eroded = cv2.erode(ball_mask, kernel, iterations=1)

    samples = depth_m[eroded > 0]
    if samples.size < MIN_VALID_DEPTH_PIXELS:
        return None
    valid = np.isfinite(samples) & (samples > 0.0)
    if int(np.count_nonzero(valid)) < MIN_VALID_DEPTH_PIXELS:
        return None

    # Median is insensitive to invalid holes, edges, and isolated depth spikes.
    z = float(np.median(samples[valid]))
    if not math.isfinite(z) or z < MIN_DEPTH_M or z > MAX_DEPTH_M:
        return None

    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return None

    return (u, v, x, y, z)


def chase_command(
    point_optical: object,
    camera_rotation_base: object,
    camera_position_base: object,
    stop_distance: float = 0.5,
    max_speed: float = 0.2,
    max_yaw_rate: float = 0.5,
) -> Tuple[float, float]:
    """Compute a bounded ``(forward_m_s, yaw_rad_s)`` chase command.

    ``point_optical`` is ``(x, y, z)`` in the camera optical frame (metres).
    ``camera_rotation_base`` is the ``3 x 3`` rotation ``R_base_camera``;
    ``camera_position_base`` is the camera origin in ``base_link`` (metres).
    The point is transformed into ``base_link`` (+x forward, +y left, +z up).

    Behaviour:

    * The robot turns in place (zero forward speed) when ``|heading|`` is
      greater than ``HEADING_GATE_RAD`` (0.4 rad).
    * Forward speed is proportional to the planar distance beyond
      ``stop_distance`` and is zero at or inside that distance.  Yaw keeps
      pointing at the ball while stopped, so a close but off-axis ball can
      still be faced before a kick.
    * ``forward_m_s`` is clamped to ``[0, max_speed]`` and ``yaw_rad_s`` to
      ``[-max_yaw_rate, max_yaw_rate]``.
    * Any malformed or non-finite input returns ``(0.0, 0.0)``.

    Only ``x`` and ``y`` affect the command; height is ignored for planar
    locomotion.
    """
    point = _finite_vector(point_optical, 3)
    rotation = _finite_matrix(camera_rotation_base, (3, 3))
    position = _finite_vector(camera_position_base, 3)
    stop = _finite_scalar(stop_distance)
    speed_limit = _finite_scalar(max_speed)
    yaw_limit = _finite_scalar(max_yaw_rate)

    if (
        point is None
        or rotation is None
        or position is None
        or stop is None
        or speed_limit is None
        or yaw_limit is None
        or not _valid_rotation(rotation)
        or stop < 0.0
        or speed_limit < 0.0
        or yaw_limit < 0.0
    ):
        return (0.0, 0.0)

    with np.errstate(over="ignore", invalid="ignore"):
        point_base = rotation @ point + position
    if not np.all(np.isfinite(point_base)):
        return (0.0, 0.0)

    x = float(point_base[0])
    y = float(point_base[1])
    distance = math.hypot(x, y)
    heading = math.atan2(y, x)

    yaw_rate = max(-yaw_limit, min(yaw_limit, HEADING_GAIN * heading))

    if distance <= stop:
        forward = 0.0
    elif abs(heading) > HEADING_GATE_RAD:
        forward = 0.0
    else:
        forward = max(0.0, min(speed_limit, DISTANCE_GAIN * (distance - stop)))

    # Avoid returning a signed zero for negative headings.
    if yaw_rate == 0.0:
        yaw_rate = 0.0
    return (float(forward), float(yaw_rate))
