#!/usr/bin/env python3
"""Synchronized RGB-D ball perception and conservative chase commands.

The node keeps three responsibilities deliberately separate:

* decode one exactly time-synchronized color/depth/CameraInfo frame;
* publish the detected pixel center and camera-optical 3D point in the same
  convention as ``humanoid_vision``;
* transform that point at the image timestamp into ``base_frame`` and publish a
  bounded ``TwistStamped`` request.

The command is only a request.  This node assumes no locomotion consumer, does
not send servo commands, and does not claim that the bundled tracking policy can
consume ``cmd_vel``.  Every loss, stale frame, frame mismatch, invalid sample, or
TF failure removes the target immediately.  A transition out of tracking also
publishes a zero command immediately, and the 20 Hz timer continues publishing
zero while the target stays invalid.
"""

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TwistStamped
from message_filters import Subscriber, TimeSynchronizer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from humanoid_ball_chase import core


class _FrameRejected(Exception):
    """Expected malformed input or TF failure; it must invalidate the target."""

    def __init__(self, reason: str, perception_ok: bool) -> None:
        super().__init__(reason)
        self.reason = reason
        self.perception_ok = perception_ok


@dataclass(frozen=True)
class _Target:
    """One accepted ball observation and the command derived at its stamp."""

    stamp_sec: float
    frame_id: str
    receipt_monotonic: float
    u: float
    v: float
    optical: Tuple[float, float, float]
    base: Tuple[float, float, float]
    vx: float
    wz: float
    detection_confidence: Optional[float] = None


class BallChaseVision(Node):
    """Detect one ball in synchronized RGB-D frames and publish bounded chase.

    RGB is decoded as ``rgb8``.  Aligned depth accepts only ``16UC1``
    (millimetres) or ``32FC1`` (metres).  The color, depth, and CameraInfo
    headers must have identical stamps and identical frame IDs.  A CameraInfo
    frame is required for every detection; intrinsics are not cached across
    frames because a cached calibration would hide a stream mismatch.
    """

    TARGET_TIMEOUT_S = 0.35
    FUTURE_TOLERANCE_S = 0.01
    CONTROL_RATE_HZ = 20.0
    TF_TIMEOUT_S = 0.10

    RGB_TOPIC = "/camera/camera/color/image_raw"
    DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
    CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
    TARGET_CENTER_TOPIC = "/vision/target_center"
    BALL_3D_TOPIC = "/vision/ball_3d_position"
    CMD_VEL_TOPIC = "/ball_chase/cmd_vel"
    STATUS_TOPIC = "/ball_chase/status"

    def __init__(self) -> None:
        super().__init__("ball_chase_vision")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("stop_distance", 0.5)
        self.declare_parameter("max_speed", 0.2)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("detector", "football_yolo")
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cpu")  # cpu: ONNX Runtime; gpu: TensorRT engine
        self.declare_parameter('target_timeout_s', 0.35)
        self.TARGET_TIMEOUT_S = float(self.get_parameter('target_timeout_s').value)
        if not math.isfinite(self.TARGET_TIMEOUT_S) or not 0 < self.TARGET_TIMEOUT_S <= 2.0:
            raise ValueError('target_timeout_s must be in (0, 2] seconds')
        self._detector = None
        detector = self.get_parameter('detector').value
        if detector == 'football_yolo':
            from .football_detector import FootballDetector
            self._detector = FootballDetector(self.get_parameter('model_path').value or None,
                                              device=str(self.get_parameter('device').value))
            self.get_logger().info(
                f'Football detector on {self._detector.device}'
                + (f' ({self._detector.engine_path.name})' if self._detector.engine_path else ''))
        elif detector != 'orange_test':
            raise ValueError('detector must be football_yolo or orange_test')

        self.base_frame = str(self.get_parameter("base_frame").value).strip()
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        limits = {
            "stop_distance": self.stop_distance,
            "max_speed": self.max_speed,
            "max_yaw_rate": self.max_yaw_rate,
        }
        for name, value in limits.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not self.base_frame:
            raise ValueError("base_frame must be a non-empty TF frame")

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        # All state below is guarded by _lock.  The timer and the synchronized
        # image callback run in different callback groups so a slow detector
        # cannot stop the 20 Hz zero-command watchdog.
        self._target: Optional[_Target] = None
        self._valid = False
        self._command_publish_fault = False
        self._command_publish_failures = 0
        self._perception_ok = False
        self._reason = "startup"
        self._last_log_monotonic = 0.0
        self._last_logged_reason = ""

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer_group = MutuallyExclusiveCallbackGroup()

        # A reliable publisher remains compatible with best-effort subscribers
        # such as humanoid_vision's track_node.  Depth one keeps commands fresh.
        point_qos = QoSProfile(depth=2)
        command_qos = QoSProfile(depth=2)
        self._target_publisher = self.create_publisher(
            PointStamped, self.TARGET_CENTER_TOPIC, point_qos)
        self._ball_publisher = self.create_publisher(
            PointStamped, self.BALL_3D_TOPIC, point_qos)
        self._command_publisher = self.create_publisher(
            TwistStamped, self.CMD_VEL_TOPIC, command_qos)
        self._status_publisher = self.create_publisher(
            String, self.STATUS_TOPIC, command_qos)

        # Sensor-data QoS (best effort, depth 5) matches the camera streams.
        self._color_subscriber = Subscriber(
            self, Image, self.RGB_TOPIC, qos_profile=QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._depth_subscriber = Subscriber(
            self, Image, self.DEPTH_TOPIC, qos_profile=QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._info_subscriber = Subscriber(
            self, CameraInfo, self.CAMERA_INFO_TOPIC,
            qos_profile=QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._synchronizer = TimeSynchronizer(
            [self._color_subscriber, self._depth_subscriber,
             self._info_subscriber],
            queue_size=10)
        self._synchronizer.registerCallback(self._on_frame)

        self._timer = self.create_timer(
            1.0 / self.CONTROL_RATE_HZ, self._control_tick,
            callback_group=self._timer_group)
        # Inference must not leave old images queued in the ROS callbacks.
        # One worker consumes only the newest synchronized RGB-D sample.
        self._pending_lock = threading.Lock()
        self._pending = None
        self._frame_ready = threading.Event()
        self._worker_stop = threading.Event()
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "BallChaseVision ready: exact RGB/depth/CameraInfo sync; "
            f"base_frame={self.base_frame}, stop_distance={self.stop_distance:.2f} m, "
            f"max_speed={self.max_speed:.2f} m/s, "
            f"max_yaw_rate={self.max_yaw_rate:.2f} rad/s; "
            "cmd_vel is a request and no locomotion consumer is assumed.")

    # ------------------------------------------------------------------ input
    def _on_frame(self, color_msg: Image, depth_msg: Image,
                  info_msg: CameraInfo) -> None:
        with self._pending_lock:
            self._pending = (color_msg, depth_msg, info_msg, time.monotonic())
            self._frame_ready.set()

    def _inference_loop(self):
        while not self._worker_stop.is_set():
            self._frame_ready.wait(0.1)
            with self._pending_lock:
                pending = self._pending
                self._pending = None
                self._frame_ready.clear()
            if pending is not None and not self._worker_stop.is_set():
                self._process_frame(*pending[:3], receipt=pending[3])

    def _process_frame(self, color_msg: Image, depth_msg: Image,
                       info_msg: CameraInfo, receipt=None) -> None:
        """Validate, detect, publish, and transform one synchronized frame.

        Any expected malformed-message, detector, or TF exception invalidates
        the current target.  Keeping the old command alive after a rejection
        would be unsafe because the old command may point at a different ball.
        """
        receipt = time.monotonic() if receipt is None else receipt
        try:
            stamp_ns, frame_id = self._validate_headers(
                color_msg, depth_msg, info_msg)
            stamp_sec = stamp_ns * 1e-9
            self._check_timing(stamp_sec, receipt)

            rgb = self._decode_rgb(color_msg)
            depth_m = self._decode_depth(depth_msg, color_msg, rgb)
            k = self._camera_matrix(info_msg, color_msg, rgb)

            u, v, x, y, z = self._detect(rgb, depth_m, k, color_msg)
            self._check_timing(stamp_sec, receipt)

            self._publish_vision(color_msg, u, v, x, y, z)
            rotation, position = self._lookup_camera_transform(
                frame_id, color_msg.header.stamp)
            self._check_timing(stamp_sec, receipt)

            vx, wz = self._chase_command((x, y, z), rotation, position)
            self._accept_color_frame(
                stamp_sec, frame_id, receipt, u, v, (x, y, z),
                rotation, position, vx, wz)
        except _FrameRejected as exc:
            self._invalidate(exc.reason, exc.perception_ok)
        except Exception as exc:  # Defensive: a malformed frame must never drive.
            self._invalidate(
                f"unexpected_frame_error:{type(exc).__name__}",
                perception_ok=False)

    def _validate_headers(self, color_msg: Image, depth_msg: Image,
                          info_msg: CameraInfo) -> Tuple[int, str]:
        """Require identical nonzero stamps and exactly equal frame IDs."""
        stamps = [
            self._stamp_ns(color_msg.header.stamp),
            self._stamp_ns(depth_msg.header.stamp),
            self._stamp_ns(info_msg.header.stamp),
        ]
        if any(stamp is None or stamp <= 0 for stamp in stamps):
            raise _FrameRejected("invalid_image_stamp", False)
        if len(set(stamps)) != 1:
            raise _FrameRejected("timestamp_mismatch", False)

        frame_id = color_msg.header.frame_id
        if not frame_id or not frame_id.strip():
            raise _FrameRejected("empty_frame_id", False)
        if (depth_msg.header.frame_id != frame_id
                or info_msg.header.frame_id != frame_id):
            raise _FrameRejected("frame_mismatch", False)
        return stamps[0], frame_id

    @staticmethod
    def _stamp_ns(stamp) -> Optional[int]:
        """Convert a builtin_interfaces stamp to integer nanoseconds."""
        if stamp.sec < 0 or stamp.nanosec >= 1_000_000_000:
            return None
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _check_timing(self, stamp_sec: float, receipt: float) -> None:
        """Enforce source-age, receipt-age, and future-stamp limits."""
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        source_age = now_ros - stamp_sec
        if source_age < -self.FUTURE_TOLERANCE_S:
            raise _FrameRejected("future_image_stamp", False)
        if source_age > self.TARGET_TIMEOUT_S:
            raise _FrameRejected("stale_image_stamp", False)
        if time.monotonic() - receipt > self.TARGET_TIMEOUT_S:
            raise _FrameRejected("stale_receipt", False)

    def _decode_rgb(self, msg: Image) -> np.ndarray:
        """Decode one color image as HxWx3 uint8 RGB."""
        if msg.width <= 0 or msg.height <= 0:
            raise _FrameRejected("invalid_rgb_dimensions", False)
        try:
            rgb = np.asarray(
                self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"))
        except Exception as exc:
            raise _FrameRejected(
                f"rgb_decode_failed:{type(exc).__name__}", False) from exc
        if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3
                or rgb.shape[0] != msg.height or rgb.shape[1] != msg.width):
            raise _FrameRejected("invalid_rgb_buffer", False)
        return rgb

    def _decode_depth(self, msg: Image, color_msg: Image,
                      rgb: np.ndarray) -> np.ndarray:
        """Decode aligned depth and normalize it to metres as float32."""
        if msg.encoding not in ("16UC1", "32FC1"):
            raise _FrameRejected("unsupported_depth_encoding", False)
        if msg.width != color_msg.width or msg.height != color_msg.height:
            raise _FrameRejected("depth_size_mismatch", False)
        try:
            raw = np.asarray(
                self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))
        except Exception as exc:
            raise _FrameRejected(
                f"depth_decode_failed:{type(exc).__name__}", False) from exc
        if raw.shape != rgb.shape[:2]:
            raise _FrameRejected("depth_size_mismatch", False)
        if msg.encoding == "16UC1":
            if raw.dtype != np.uint16:
                raise _FrameRejected("invalid_depth_dtype", False)
            depth_m = raw.astype(np.float32) * np.float32(0.001)
        else:
            if raw.dtype != np.float32:
                raise _FrameRejected("invalid_depth_dtype", False)
            depth_m = raw
        return depth_m

    @staticmethod
    def _camera_matrix(info_msg: CameraInfo, color_msg: Image,
                       rgb: np.ndarray) -> np.ndarray:
        """Return a validated 3x3 intrinsic matrix for the color image."""
        if (info_msg.width != color_msg.width
                or info_msg.height != color_msg.height
                or rgb.shape[1] != color_msg.width
                or rgb.shape[0] != color_msg.height):
            raise _FrameRejected("camera_info_size_mismatch", False)
        try:
            k = np.asarray(info_msg.k, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise _FrameRejected("invalid_camera_info", False) from exc
        if k.size != 9 or not np.isfinite(k).all():
            raise _FrameRejected("invalid_camera_info", False)
        k = k.reshape(3, 3)
        if k[0, 0] <= 0.0 or k[1, 1] <= 0.0:
            raise _FrameRejected("invalid_camera_focal_length", False)
        return k

    # -------------------------------------------------------------- perception
    def _detect(self, rgb: np.ndarray, depth_m: np.ndarray, k: np.ndarray,
                color_msg: Image) -> Tuple[float, float, float, float, float]:
        """Run the shared detector and validate its five-tuple result."""
        try:
            detected = (self._detector.detect(rgb, depth_m, k) if self._detector is not None
                        else core.detect_ball(rgb, depth_m, k))
        except Exception as exc:
            raise _FrameRejected(
                f"detection_error:{type(exc).__name__}", True) from exc
        if detected is None:
            raise _FrameRejected("detection_failed", True)
        try:
            u, v, x, y, z = (float(value) for value in detected)
        except (TypeError, ValueError) as exc:
            raise _FrameRejected("invalid_detection_tuple", True) from exc
        values = np.array([u, v, x, y, z], dtype=np.float64)
        if not np.isfinite(values).all():
            raise _FrameRejected("nonfinite_detection", True)
        if not (0.0 <= u < color_msg.width and 0.0 <= v < color_msg.height):
            raise _FrameRejected("detection_outside_image", True)
        if z <= 0.0:
            raise _FrameRejected("nonpositive_depth", True)
        return u, v, x, y, z

    def _publish_vision(self, color_msg: Image, u: float, v: float,
                        x: float, y: float, z: float) -> None:
        """Publish pixel and camera-optical coordinates with the color header."""
        center = PointStamped()
        center.header.stamp = color_msg.header.stamp
        center.header.frame_id = color_msg.header.frame_id
        center.point.x = float(u)
        center.point.y = float(v)
        center.point.z = 0.0
        self._target_publisher.publish(center)

        point = PointStamped()
        point.header.stamp = color_msg.header.stamp
        point.header.frame_id = color_msg.header.frame_id
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)
        self._ball_publisher.publish(point)

    # --------------------------------------------------------------------- TF
    def _lookup_camera_transform(self, camera_frame: str, stamp
                                 ) -> Tuple[np.ndarray, np.ndarray]:
        """Look up base<-camera at the image stamp, never at latest time."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame, camera_frame, Time.from_msg(stamp),
                timeout=Duration(seconds=self.TF_TIMEOUT_S))
        except Exception as exc:
            raise _FrameRejected(
                f"tf_lookup_failed:{type(exc).__name__}", True) from exc

        try:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            position = np.array(
                [translation.x, translation.y, translation.z],
                dtype=np.float64)
            rotation_matrix = self._quaternion_matrix(
                rotation.x, rotation.y, rotation.z, rotation.w)
        except (TypeError, ValueError) as exc:
            raise _FrameRejected("invalid_tf_transform", True) from exc
        if not np.isfinite(position).all() or not np.isfinite(
                rotation_matrix).all():
            raise _FrameRejected("nonfinite_tf_transform", True)
        return rotation_matrix, position

    @staticmethod
    def _quaternion_matrix(qx: float, qy: float, qz: float,
                           qw: float) -> np.ndarray:
        """Convert a normalized quaternion to a rotation matrix using numpy."""
        quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
        if not np.isfinite(quaternion).all():
            raise ValueError("non-finite quaternion")
        norm = float(np.linalg.norm(quaternion))
        if norm < 1e-9:
            raise ValueError("zero quaternion")
        x, y, z, w = quaternion / norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z),
             2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w),
             1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w),
             2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float64)

    # ----------------------------------------------------------------- control
    def _chase_command(self, point_optical: Tuple[float, float, float],
                       rotation: np.ndarray, position: np.ndarray
                       ) -> Tuple[float, float]:
        """Call the shared controller and enforce the node's safety envelope."""
        point = np.array(point_optical, dtype=np.float64)
        try:
            command = core.chase_command(
                point, rotation, position,
                stop_distance=self.stop_distance,
                max_speed=self.max_speed,
                max_yaw_rate=self.max_yaw_rate)
            vx, wz = command
            vx = float(vx)
            wz = float(wz)
        except Exception as exc:
            raise _FrameRejected(
                f"chase_error:{type(exc).__name__}", True) from exc
        if not math.isfinite(vx) or not math.isfinite(wz):
            raise _FrameRejected("nonfinite_chase_command", True)
        vx = float(np.clip(vx, -self.max_speed, self.max_speed))
        wz = float(np.clip(wz, -self.max_yaw_rate, self.max_yaw_rate))
        return vx, wz

    def _accept_color_frame(self, stamp_sec: float, frame_id: str,
                            receipt: float, u: float, v: float,
                            optical: Tuple[float, float, float],
                            rotation: np.ndarray, position: np.ndarray,
                            vx: float, wz: float) -> None:
        """Store a target after rechecking freshness at commit time."""
        self._check_timing(stamp_sec, receipt)
        base_point = np.asarray(rotation, dtype=np.float64) @ np.asarray(
            optical, dtype=np.float64) + np.asarray(position, dtype=np.float64)
        if not np.isfinite(base_point).all():
            raise _FrameRejected("nonfinite_base_point", True)
        target = _Target(
            stamp_sec=stamp_sec,
            frame_id=frame_id,
            receipt_monotonic=receipt,
            u=u,
            v=v,
            optical=(float(optical[0]), float(optical[1]), float(optical[2])),
            base=(float(base_point[0]), float(base_point[1]),
                  float(base_point[2])),
            vx=vx,
            wz=wz,
            detection_confidence=(self._detector.last_confidence
                                  if self._detector is not None else None))

        with self._lock:
            if self._command_publish_fault:
                return  # Fresh detections cannot clear a command transport fault.
            was_tracking = self._valid and self._target is not None
            self._target = target
            self._valid = True
            self._perception_ok = True
            self._reason = "ok"
        if not was_tracking:
            self.get_logger().info(
                "Tracking ball at base "
                f"({target.base[0]:.2f}, {target.base[1]:.2f}, "
                f"{target.base[2]:.2f}) m; "
                f"vx={target.vx:.2f} m/s, wz={target.wz:.2f} rad/s")

    # -------------------------------------------------------------- invalidation
    def _invalidate(self, reason: str, perception_ok: bool) -> None:
        """Drop the target immediately and stop a previously moving command."""
        with self._lock:
            was_tracking = self._invalidate_locked(reason, perception_ok)
            if was_tracking:
                self._publish_zero_now()

    def _invalidate_locked(self, reason: str, perception_ok: bool,
                           log: bool = True) -> bool:
        was_tracking = self._valid and self._target is not None
        reason_changed = reason != self._last_logged_reason
        now = time.monotonic()

        self._valid = False
        self._perception_ok = bool(perception_ok)
        self._reason = reason
        self._target = None

        # Rate-limit repeated failures of the same kind (for example a blank
        # image at camera rate), but always report a transition out of tracking.
        if log and (was_tracking or (reason_changed
                                     and now - self._last_log_monotonic > 5.0)):
            self._last_log_monotonic = now
            self._last_logged_reason = reason
            self.get_logger().warning(
                f"Ball chase target invalidated: {reason}")
        return was_tracking

    # ------------------------------------------------------------------- timer
    def _publish_zero_now(self) -> None:
        """Publish one current-stamped zero command, best effort."""
        try:
            command = TwistStamped()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = self.base_frame
            self._command_publisher.publish(command)
        except RCLError as error:
            self._record_command_publish_failure(error)

    def _record_command_publish_failure(self, error) -> None:
        # Called with _lock held by control/invalidation; never acquire it recursively.
        self._command_publish_fault = True
        self._command_publish_failures += 1
        self._invalidate_locked('command_publish_failed_restart_required', False)
        self.get_logger().error(f'Chase command publication failed; zero request latched: {error}',
                                throttle_duration_sec=5.0)

    def _control_tick(self) -> None:
        """Publish one current-stamped command and status at a nominal 20 Hz."""
        now = self.get_clock().now()
        now_ros = now.nanoseconds * 1e-9
        now_monotonic = time.monotonic()

        with self._lock:
            if self._valid and self._target is not None:
                target = self._target
                source_age = now_ros - target.stamp_sec
                receipt_age = now_monotonic - target.receipt_monotonic
                if source_age < -self.FUTURE_TOLERANCE_S:
                    self._invalidate_locked("future_image_stamp", False)
                elif source_age > self.TARGET_TIMEOUT_S:
                    self._invalidate_locked("stale_image_stamp", False)
                elif receipt_age > self.TARGET_TIMEOUT_S:
                    self._invalidate_locked("stale_receipt", False)

            if self._valid and self._target is not None:
                vx = self._target.vx
                wz = self._target.wz
            else:
                vx = 0.0
                wz = 0.0

            status = self._status_locked(now_ros, now_monotonic, vx, wz)

            command = TwistStamped()
            command.header.stamp = now.to_msg()
            command.header.frame_id = self.base_frame
            command.twist.linear.x = float(vx)
            command.twist.angular.z = float(wz)
            try:
                self._command_publisher.publish(command)
            except RCLError as error:
                self._record_command_publish_failure(error)
                status = self._status_locked(now_ros, now_monotonic, 0.0, 0.0)

        try:
            payload = json.dumps(status, separators=(",", ":"),
                                 allow_nan=False)
        except (TypeError, ValueError):
            payload = json.dumps({
                "state": "lost",
                "perception": False,
                "tracking": False,
                "lost": True,
                "command": {"linear_x": 0.0, "angular_z": 0.0},
                "locomotion_consumer_assumed": False,
                "note": "status serialization failed; command is zero",
            }, separators=(",", ":"))
        try:
            self._status_publisher.publish(String(data=payload))
        except RCLError as error:
            self.get_logger().error(f'Chase diagnostics publication failed: {error}',
                                    throttle_duration_sec=5.0)

    def _status_locked(self, now_ros: float, now_monotonic: float,
                       vx: float, wz: float) -> dict:
        """Build a JSON-serializable status snapshot while holding the lock."""
        tracking = self._valid and self._target is not None
        perception = bool(self._perception_ok)
        target = self._target if tracking else None
        lost = not tracking
        state = "tracking" if tracking else "lost"

        detail = {
            "ok": perception,
            "reason": self._reason if not tracking else "ok",
            "frame_id": target.frame_id if target is not None else "",
            "source_age_s": (now_ros - target.stamp_sec
                             if target is not None else None),
            "receipt_age_s": (now_monotonic - target.receipt_monotonic
                              if target is not None else None),
        }
        if target is not None:
            target_json = {
                "frame_id": target.frame_id,
                "pixel": {"x": target.u, "y": target.v},
                "optical_m": {"x": target.optical[0],
                              "y": target.optical[1],
                              "z": target.optical[2]},
                "base_m": {"x": target.base[0],
                           "y": target.base[1],
                           "z": target.base[2]},
            }
        else:
            target_json = None

        return {
            "state": state,
            "command_publish_fault": self._command_publish_fault,
            "command_publish_failures": self._command_publish_failures,
            "perception": perception,
            "perception_detail": detail,
            "tracking": tracking,
            "lost": lost,
            "target": target_json,
            "command": {"linear_x": float(vx), "angular_z": float(wz)},
            "locomotion_consumer_assumed": False,
            "detector": "football_yolo" if self._detector is not None else "orange_test",
            "detection_confidence": target.detection_confidence if target is not None else None,
            "hardware_output": False,
            "note": ("cmd_vel is a request; no locomotion consumer is assumed "
                     "and no servo command is published."),
        }

    # ----------------------------------------------------------------- shutdown
    def stop(self) -> None:
        """Best-effort final zero command for a clean shutdown."""
        self._worker_stop.set()
        self._frame_ready.set()
        self._worker.join(timeout=2.0)
        with self._lock:
            self._invalidate_locked("shutdown", False, log=False)
        self._publish_zero_now()

    def destroy_node(self):
        self.stop()
        return super().destroy_node()


def main(args=None) -> None:
    """Run BallChaseVision until shutdown and stop commands on exit."""
    rclpy.init(args=args)
    node: Optional[BallChaseVision] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = BallChaseVision()
        # Separate executor threads let the 20 Hz watchdog continue while the
        # synchronized frame callback performs detection and TF lookup.
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Stop and wait for callbacks before destroying publishers or looking
        # up more TF.  The final zero command is then the last command sent.
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
