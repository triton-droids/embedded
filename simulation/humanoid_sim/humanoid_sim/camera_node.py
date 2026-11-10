"""Simulated replacement for realsense2_camera_node on the head camera.

Publishes the topics ball_detection_node and track_node consume by default:

  /camera/camera/color/image_raw                   rgb8
  /camera/camera/color/camera_info                 pinhole intrinsics from fovy
  /camera/camera/aligned_depth_to_color/image_raw  16UC1 millimetres (optional)

Depth is rendered from the same camera, so it is already aligned to color.
Intrinsics are an ideal pinhole without distortion; they are not the D435's
calibration. Rendering uses an OpenGL context owned by this node's own thread
(MuJoCo contexts are thread-bound); select the backend with MUJOCO_GL
(glfw with a display, egl or osmesa headless).
"""
from __future__ import annotations

import array
import math
import threading
import time

import mujoco
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from . import model_builder as mb
from .image_degradation import DegradationSettings, ImageDegrader
from .world import SimWorld


def pinhole_info(width: int, height: int, fovy_deg: float, frame_id: str) -> CameraInfo:
    """CameraInfo for MuJoCo's square-pixel pinhole with vertical field of view fovy."""
    focal = 0.5 * height / math.tan(math.radians(fovy_deg) / 2.0)
    cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width, info.height = width, height
    info.distortion_model = 'plumb_bob'
    info.d = [0.0] * 5
    info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


class SimCameraNode(Node):

    def __init__(self, world: SimWorld) -> None:
        super().__init__('sim_camera')
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('enable_depth', False)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('base_forward_axis', 'negative_y')
        forward_axis = self.get_parameter('base_forward_axis').value
        if forward_axis not in ('negative_y', 'positive_y'):
            raise ValueError('base_forward_axis must be negative_y or positive_y')
        sign = 1.0 if forward_axis == 'positive_y' else -1.0
        self._body_from_base = np.array([[0., -sign, 0.], [sign, 0., 0.], [0., 0., 1.]])
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('publish_tf', False)
        # Optional colour-image effects (image_degradation.py); defaults leave it unchanged.
        defaults = DegradationSettings()
        for name in DegradationSettings.names():
            self.declare_parameter('degrade_' + name, getattr(defaults, name))
        self._tf = None
        if self.get_parameter('publish_tf').value:
            from tf2_ros import TransformBroadcaster
            self._tf = TransformBroadcaster(self)
        self.world = world
        self._period = 1.0 / float(self.get_parameter('rate_hz').value)
        self._depth = bool(self.get_parameter('enable_depth').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        camera = world.model.camera(mb.HEAD_CAMERA).id
        self._width, self._height = (int(v) for v in world.model.cam_resolution[camera])
        self._info = pinhole_info(self._width, self._height, float(world.model.cam_fovy[camera]),
                                  self._frame_id)
        settings = DegradationSettings(**{
            name: type(getattr(defaults, name))(self.get_parameter('degrade_' + name).value)
            for name in DegradationSettings.names()})
        self._degrader = ImageDegrader(settings, self._info.k[0])
        self._camera_id = camera
        if settings.enabled:
            self.get_logger().warn(f'Colour image degradation enabled: {settings}')
        self.color_pub = self.create_publisher(Image, self.get_parameter('color_topic').value, 2)
        self.info_pub = self.create_publisher(
            CameraInfo, self.get_parameter('camera_info_topic').value, 2)
        self.depth_pub = None
        if self._depth:
            self.depth_pub = self.create_publisher(Image, self.get_parameter('depth_topic').value, 2)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._render_loop, name='sim_camera', daemon=True)
        self._thread.start()

    def destroy_node(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        super().destroy_node()

    def _render_loop(self) -> None:
        from .thread_setup import configure_thread
        configure_thread('other')
        try:
            color = mujoco.Renderer(self.world.model, self._height, self._width)
            depth = mujoco.Renderer(self.world.model, self._height, self._width) if self._depth else None
            if depth is not None:
                depth.enable_depth_rendering()
        except Exception as exc:  # No GL backend: keep the rest of the simulation running.
            self.get_logger().error(f'Camera disabled; could not create a MuJoCo renderer: {exc}')
            return
        self.get_logger().info(f'Rendering {self._width}x{self._height} head camera')
        next_frame = time.monotonic()
        while not self._stop.is_set():
            next_frame += self._period
            with self.world.lock:
                color.update_scene(self.world.data, camera=mb.HEAD_CAMERA)
                if depth is not None:
                    depth.update_scene(self.world.data, camera=mb.HEAD_CAMERA)
                stamp = self.get_clock().now().to_msg()
                omega = None
                if self._degrader.s.exposure_s > 0:
                    velocity = np.zeros(6)
                    mujoco.mj_objectVelocity(self.world.model, self.world.data, mujoco.mjtObj.mjOBJ_CAMERA,
                                             self._camera_id, velocity, 1)
                    omega = velocity[:3]
                if self._tf is not None:
                    transform = self._camera_transform(stamp)
            if not rclpy.ok():
                break
            try:
                if self._tf is not None:
                    self._tf.sendTransform(transform)
                self._publish_color(self._degrader.apply(color.render(), omega), stamp)
                if depth is not None:
                    self._publish_depth(depth.render(), stamp)
                self._info.header.stamp = stamp
                self.info_pub.publish(self._info)
            except RCLError:  # Context shut down between the check and publish.
                break
            delay = next_frame - time.monotonic()
            if delay > 0.0:
                self._stop.wait(delay)
            else:
                next_frame = time.monotonic()
        color.close()
        if depth is not None:
            depth.close()

    def _camera_transform(self, stamp):
        """Actual moving optical extrinsics in ROS base_link axes; caller holds lock."""
        from geometry_msgs.msg import TransformStamped
        data = self.world.data
        base = self.world.model.body(mb.BASE_BODY).id
        camera = self.world.model.camera(mb.HEAD_CAMERA).id
        # Scene-specific heading; the football scene follows the actual toes (+Y).
        body_from_base = self._body_from_base
        world_from_base = data.xmat[base].reshape(3, 3) @ body_from_base
        base_from_optical = (world_from_base.T @ data.cam_xmat[camera].reshape(3, 3)
                             @ np.diag([1., -1., -1.]))
        position = world_from_base.T @ (data.cam_xpos[camera] - data.xpos[base])
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, base_from_optical.reshape(-1))
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.child_frame_id = self._frame_id
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, position)
        msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z = map(float, quat)
        return msg

    def _publish_color(self, pixels: np.ndarray, stamp) -> None:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height, msg.width = pixels.shape[:2]
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        # array.array skips rclpy's per-element validation of bytes, which
        # holds the GIL ~150 ms per 640x480 frame and starves the physics loop.
        msg.data = array.array('B', pixels.tobytes())
        self.color_pub.publish(msg)

    def _publish_depth(self, metres: np.ndarray, stamp) -> None:
        # RealSense convention: uint16 millimetres, 0 = no data (beyond 65.535 m here).
        millimetres = np.where(metres < 65.535, np.round(metres * 1000.0), 0).astype(np.uint16)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height, msg.width = millimetres.shape
        msg.encoding = '16UC1'
        msg.step = msg.width * 2
        msg.data = array.array('B', millimetres.tobytes())
        self.depth_pub.publish(msg)
