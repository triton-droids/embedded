"""Passive grayscale stereo pair for visual odometry (stands in for D435 IR imagers, projector off).

Publishes, with one shared header stamp per frame pair:

  /stereo/left/image_raw,  /stereo/left/camera_info    mono8, frame stereo_left_optical
  /stereo/right/image_raw, /stereo/right/camera_info   mono8, frame stereo_right_optical

The two MuJoCo cameras share one orientation and differ only by a horizontal
baseline (model_builder._add_stereo), so the images are already rectified: both
camera_info messages carry the same ideal pinhole, and the right projection
matrix has Tx = -fx * baseline (ROS stereo convention).

TF (all from the simulated pose, standing in for head forward kinematics):

  floating_base -> stereo_rig            dynamic, stamped with the image pair
  stereo_rig    -> stereo_left_optical   static
  stereo_rig    -> stereo_right_optical  static

``stereo_rig`` sits on the left camera with ROS body axes (x forward along the
optical axis, y left, z up), so a VO pose of the rig has the usual axis meaning.
"""
from __future__ import annotations

import array
import threading
import time

import mujoco
import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from . import model_builder as mb
from .camera_node import pinhole_info
from .world import SimWorld

RIG_FRAME = 'stereo_rig'
OPTICAL_FRAMES = ('stereo_left_optical', 'stereo_right_optical')
# Optical axes (x right, y down, z forward) expressed in the rig (x forward, y left, z up).
RIG_FROM_OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
# MuJoCo camera axes (x right, y up, looking along -z) to optical axes.
CAMERA_FROM_OPTICAL = np.diag([1., -1., -1.])
LUMA_Q8 = (77, 150, 29)  # BT.601 luma in 1/256 steps; integer math keeps the GIL hold short.


def stereo_infos(width: int, height: int, fovy_deg: float, baseline_m: float) -> tuple[CameraInfo, CameraInfo]:
    left = pinhole_info(width, height, fovy_deg, OPTICAL_FRAMES[0])
    right = pinhole_info(width, height, fovy_deg, OPTICAL_FRAMES[1])
    p = list(right.p)
    p[3] = -left.k[0] * baseline_m
    right.p = p
    return left, right


def _quat_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(rotation).reshape(-1))
    return float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])


def _transform(stamp, parent: str, child: str, translation, rotation) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    t = msg.transform.translation
    t.x, t.y, t.z = (float(v) for v in translation)
    r = msg.transform.rotation
    r.x, r.y, r.z, r.w = _quat_xyzw(rotation)
    return msg


class SimStereoCameraNode(Node):
    """Render and publish the stereo pair from a MuJoCo state source.

    ``sync()`` runs under ``lock``; it brings ``data`` to the pose to render
    and returns that pose's stamp in nanoseconds, or None to skip this frame.
    In the simulator process this is the live physics state (``in_process``);
    in ``sim_stereo_publisher`` it is a copy read from shared memory.
    """

    def __init__(self, model, data, lock, sync, name: str = 'sim_stereo_camera') -> None:
        super().__init__(name)
        # Prefixed names: launch dictionaries reach every node in the mujoco_sim process.
        self.declare_parameter('stereo_rate_hz', 15.0)
        self.declare_parameter('stereo_noise_std', 0.0)  # Gaussian sensor noise, grey levels.
        self.declare_parameter('stereo_seed', 3)
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        self.model, self.data, self._lock, self._sync = model, data, lock, sync
        try:
            self._ids = [model.camera(name).id for name in mb.STEREO_CAMERAS]
        except KeyError as exc:
            raise ValueError('enable_stereo needs stereo cameras: set stereo_baseline_m > 0') from exc
        self._base = model.body(mb.BASE_BODY).id
        self._width, self._height = (int(v) for v in model.cam_resolution[self._ids[0]])
        positions = [model.cam_pos[i] for i in self._ids]  # In the shared parent body.
        self.baseline_m = float(np.linalg.norm(positions[1] - positions[0]))
        self._infos = stereo_infos(self._width, self._height, float(model.cam_fovy[self._ids[0]]),
                                   self.baseline_m)
        self._period = 1.0 / float(self.get_parameter('stereo_rate_hz').value)
        self._noise_std = float(self.get_parameter('stereo_noise_std').value)
        self._rng = np.random.default_rng(int(self.get_parameter('stereo_seed').value))
        self._image_pubs = [self.create_publisher(Image, f'/stereo/{side}/image_raw', 2)
                            for side in ('left', 'right')]
        self._info_pubs = [self.create_publisher(CameraInfo, f'/stereo/{side}/camera_info', 2)
                           for side in ('left', 'right')]
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        self._static_tf.sendTransform([
            _transform(now, RIG_FRAME, OPTICAL_FRAMES[0], (0., 0., 0.), RIG_FROM_OPTICAL),
            _transform(now, RIG_FRAME, OPTICAL_FRAMES[1], (0., -self.baseline_m, 0.), RIG_FROM_OPTICAL),
        ])
        self.frames = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._render_loop, name='sim_stereo', daemon=True)
        self._thread.start()

    @classmethod
    def in_process(cls, world: SimWorld) -> 'SimStereoCameraNode':
        """Render the live physics state (holds world.lock only for the scene update)."""
        return cls(world.model, world.data, world.lock, lambda: getattr(world, 'sample_stamp_ns', None))

    def destroy_node(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        super().destroy_node()

    def _rig_transform(self, stamp) -> TransformStamped:
        """floating_base -> stereo_rig from the current pose; caller holds the lock."""
        data = self.data
        world_from_base = data.xmat[self._base].reshape(3, 3)
        world_from_rig = (data.cam_xmat[self._ids[0]].reshape(3, 3) @ CAMERA_FROM_OPTICAL
                          @ RIG_FROM_OPTICAL.T)
        position = world_from_base.T @ (data.cam_xpos[self._ids[0]] - data.xpos[self._base])
        return _transform(stamp, mb.BASE_BODY, RIG_FRAME, position, world_from_base.T @ world_from_rig)

    def _render_loop(self) -> None:
        from .thread_setup import configure_thread
        configure_thread('other')
        try:
            renderers = [mujoco.Renderer(self.model, self._height, self._width) for _ in self._ids]
        except Exception as exc:  # No GL backend: keep the rest of the simulation running.
            self.get_logger().error(f'Stereo disabled; could not create a MuJoCo renderer: {exc}')
            return
        self.get_logger().info(
            f'Rendering {self._width}x{self._height} stereo pair, baseline {self.baseline_m * 1e3:.1f} mm')
        next_frame = time.monotonic()
        last_stamp_ns = None
        while not self._stop.is_set():
            next_frame += self._period
            rig = None
            with self._lock:  # Both views and the rig pose from one state.
                stamp_ns = self._sync()
                if stamp_ns is not None and stamp_ns != last_stamp_ns:
                    for renderer, camera in zip(renderers, self._ids):
                        renderer.update_scene(self.data, camera=camera)
                    stamp = Time(nanoseconds=int(stamp_ns)).to_msg()
                    rig = self._rig_transform(stamp)
            if rig is not None:  # A paused simulation repeats its stamp: send nothing.
                last_stamp_ns = stamp_ns
                try:
                    self._tf.sendTransform(rig)
                    for renderer, image_pub, info_pub, info in zip(
                            renderers, self._image_pubs, self._info_pubs, self._infos):
                        image_pub.publish(self._mono(renderer.render(), stamp, info.header.frame_id))
                        info.header.stamp = stamp
                        info_pub.publish(info)
                except RCLError:  # Context shut down between the check and publish.
                    break
                self.frames += 1
            delay = next_frame - time.monotonic()
            if delay > 0.0:
                self._stop.wait(delay)
            else:
                next_frame = time.monotonic()
        for renderer in renderers:
            renderer.close()

    def _mono(self, rgb: np.ndarray, stamp, frame_id: str) -> Image:
        grey = rgb[..., 0].astype(np.uint16) * LUMA_Q8[0]
        grey += rgb[..., 1].astype(np.uint16) * LUMA_Q8[1]
        grey += rgb[..., 2].astype(np.uint16) * LUMA_Q8[2]
        pixels = (grey >> 8).astype(np.uint8)
        if self._noise_std > 0.0:
            noisy = pixels + self._rng.normal(0.0, self._noise_std, pixels.shape)
            pixels = np.clip(np.rint(noisy), 0, 255).astype(np.uint8)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = pixels.shape
        msg.encoding = 'mono8'
        msg.step = msg.width
        # array.array skips rclpy's per-element validation (see camera_node).
        msg.data = array.array('B', pixels.tobytes())
        return msg


def main(args=None) -> None:
    """sim_stereo_publisher: render the stereo pair outside the simulator process.

    Waits for the state segment written by mujoco_sim (stereo_split:=true), loads
    the model it saved, and renders that state with its own MjData, so rendering
    never holds the simulator's GIL or world lock.
    """
    import rclpy
    from rclpy.executors import ExternalShutdownException

    from .state_shm import StateReader
    rclpy.init(args=args)
    reader = None
    node = None
    try:
        reader = StateReader(timeout_s=120.0, should_stop=lambda: not rclpy.ok())
        # A file left by an earlier run looks valid; wait for this run's steps.
        while rclpy.ok():
            record = reader.read()
            if record is not None and abs(time.time_ns() - record[0]) < 2e9:
                break
            time.sleep(0.05)
        model = mujoco.MjModel.from_binary_path(reader.model_path)
        if model.nq != reader.nq or model.nmocap != reader.nmocap:
            raise RuntimeError('Stereo model and state segment do not match')
        data = mujoco.MjData(model)

        def sync():
            record = reader.read()
            if record is None:
                return None
            stamp_ns, qpos, mocap_pos, mocap_quat = record
            data.qpos[:] = qpos
            if model.nmocap:
                data.mocap_pos[:] = mocap_pos
                data.mocap_quat[:] = mocap_quat
            mujoco.mj_kinematics(model, data)
            mujoco.mj_camlight(model, data)
            return stamp_ns

        node = SimStereoCameraNode(model, data, threading.Lock(), sync)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, FileNotFoundError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if reader is not None:
            reader.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
