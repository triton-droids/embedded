"""Simulated odometry TF from ground truth (stand-in for a state estimator).

Publishes the standard robot frame chain so consumers never read simulator
ground truth directly:

    world ──(static, sim only)──▶ odom ──▶ base_footprint ──▶ base_link ──▶ camera (sim_camera)

* ``odom → base_footprint`` and ``base_footprint → base_link`` on ``/tf`` at
  ``rate_hz``, stamped with the physics sample time (the camera stamps images
  on the same wall clock, so lookups at image stamps interpolate).
* ``world → odom`` on ``/tf_static``. ``odom_origin: start`` anchors odom at the
  first footprint pose and re-anchors after a world reset (like an estimator
  restart); ``world`` makes odom identical to world.
* ``nav_msgs/Odometry`` on ``odom_topic`` (frame odom, child base_footprint,
  twist in base_footprint axes per REP-105); '' disables it.

A real robot replaces this node with its state estimator publishing the same
``odom → base_footprint`` (and ``base_footprint → base_link``). ``world → odom``
there comes from self-localization (later ``map``/``field``). All frame names
are parameters so a multi-robot setup can use e.g. ``robot1/odom``.
"""

from __future__ import annotations

import time

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from . import frames
from . import model_builder as mb
from .world import SimWorld


def _transform(T: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    t = msg.transform.translation
    t.x, t.y, t.z = (float(v) for v in T[:3, 3])
    r = msg.transform.rotation
    r.x, r.y, r.z, r.w = frames.quaternion_xyzw(T[:3, :3])
    return msg


class SimOdomTfNode(Node):

    def __init__(self, world: SimWorld, publish_robot: bool = True) -> None:
        super().__init__('sim_odom_tf')
        # False: another odometry source (e.g. walking_controller leg_odometry) publishes
        # odom -> base_footprint -> base_link; only world -> odom comes from here (evaluation).
        self.publish_robot = bool(publish_robot)
        p = self.declare_parameter
        rate = float(p('rate_hz', 50.0).value)
        self._body_from_base = frames.body_from_base(p('base_forward_axis', 'negative_y').value)
        self.world_frame = p('world_frame', 'world').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.footprint_frame = p('base_footprint_frame', 'base_footprint').value
        self.base_frame = p('base_frame', 'base_link').value
        self.origin = p('odom_origin', 'start').value
        if self.origin not in ('start', 'world'):
            raise ValueError("odom_origin must be 'start' or 'world'")
        topic = p('odom_topic', '/odom').value

        self.world = world
        self._base = world.model.body(mb.BASE_BODY).id
        self._period_ns = int(1e9 / rate)
        self._tf = TransformBroadcaster(self)
        self._static = StaticTransformBroadcaster(self)
        self._odom_pub = (self.create_publisher(Odometry, topic, 10)
                          if topic and self.publish_robot else None)
        self._world_from_odom = None
        self._reset_count = None
        self._last_stamp_ns = 0
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Odometry TF {self.world_frame} -> {self.odom_frame} -> {self.footprint_frame} -> '
            f'{self.base_frame} at {rate:.0f} Hz (odom_origin={self.origin}, ground truth)'
            if self.publish_robot else
            f'Only {self.world_frame} -> {self.odom_frame} from ground truth (odom_origin={self.origin}); '
            'another node publishes the robot odometry')

    def _sample(self):
        with self.world.lock:
            data = self.world.data
            world_from_base = frames.rigid(
                data.xmat[self._base].reshape(3, 3) @ self._body_from_base, data.xpos[self._base].copy())
            _, _, lin, ang = self.world.base_state()
            stamp_ns = self.world.sample_stamp_ns
            reset_count = self.world.reset_count
        return world_from_base, lin, ang, stamp_ns, reset_count

    def _tick(self) -> None:
        world_from_base, lin, ang, stamp_ns, reset_count = self._sample()
        if not np.isfinite(world_from_base).all():
            return
        now_ns = time.time_ns()
        if stamp_ns <= self._last_stamp_ns:
            # Physics paused or slower than this timer: the pose is unchanged, so restamp it with the
            # current time (keeps lookups at new image stamps working), but never repeat a stamp.
            if now_ns - self._last_stamp_ns < 2 * self._period_ns:
                return
            stamp_ns = now_ns
        self._last_stamp_ns = stamp_ns
        stamp = self.get_clock().now().to_msg()
        stamp.sec, stamp.nanosec = divmod(stamp_ns, 10**9)

        if self._world_from_odom is None or reset_count != self._reset_count:
            self._anchor(world_from_base, reset_count, stamp)
        if not self.publish_robot:
            return
        odom_from_fp, fp_from_base = frames.split_odometry(self._world_from_odom, world_from_base)
        self._tf.sendTransform([
            _transform(odom_from_fp, self.odom_frame, self.footprint_frame, stamp),
            _transform(fp_from_base, self.footprint_frame, self.base_frame, stamp)])
        if self._odom_pub is not None:
            self._odom_pub.publish(self._odometry(odom_from_fp, world_from_base, lin, ang, stamp))

    def _anchor(self, world_from_base, reset_count, stamp) -> None:
        self._world_from_odom = (frames.footprint(world_from_base) if self.origin == 'start' else np.eye(4))
        if self._reset_count is not None:
            self.get_logger().info('World reset: odom re-anchored at the new footprint pose')
        self._reset_count = reset_count
        self._static.sendTransform(_transform(self._world_from_odom, self.world_frame, self.odom_frame, stamp))

    def _odometry(self, odom_from_fp, world_from_base, lin, ang, stamp) -> Odometry:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.footprint_frame
        pose = msg.pose.pose
        pose.position.x, pose.position.y = float(odom_from_fp[0, 3]), float(odom_from_fp[1, 3])
        q = pose.orientation
        q.x, q.y, q.z, q.w = frames.quaternion_xyzw(odom_from_fp[:3, :3])
        v, w = frames.footprint_twist(world_from_base, lin, ang)
        twist = msg.twist.twist
        twist.linear.x, twist.linear.y, twist.linear.z = (float(x) for x in v)
        twist.angular.x, twist.angular.y, twist.angular.z = (float(x) for x in w)
        return msg  # covariance left zero: ground truth. A real estimator fills it.
