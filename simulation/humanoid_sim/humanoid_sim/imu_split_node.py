"""Publish the body IMU from a process that shares nothing with the simulator.

Everything the simulator publishes shares one Python interpreter: a blocking
native publish holds the GIL, so one slow ``/joint_states`` write can delay the
IMU timer even when they run on different executors.  Measured on this stack a
single IMU publish waited 331.9 ms.  Running the IMU publisher in its own
process removes that coupling: it reads the newest snapshot from the shared
segment (see :mod:`humanoid_sim.imu_shm`) and only ever writes its own topics.

Acquisition still happens in the physics process, so a stall there still freezes
the sample stamp.  This isolates publication, which is where the measured stall
was.
"""
from __future__ import annotations

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .imu_nodes import BODY_ORIENTATION_COVARIANCE, _ImuNoise, _fill_imu
from .imu_shm import ImuSnapshotReader
from .world import ImuSnapshot
from .publish_sample import publish_sample


class SplitBodyImuNode(Node):
    """Body IMU publisher fed by the shared segment instead of ``SimWorld``."""

    def __init__(self) -> None:
        super().__init__('sim_body_imu')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('seed', 1)
        self.declare_parameter('snapshot_path', '')
        self.declare_parameter('snapshot_timeout_s', 30.0)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._noise = _ImuNoise(self, int(self.get_parameter('seed').value))
        self._published = 0
        self._last_publish_mono = None
        self._last_callback_mono = None
        self._max_publish_gap_s = 0.0
        self._max_callback_gap_s = 0.0
        self._max_publish_call_s = 0.0
        self._max_sample_wait_s = 0.0
        self._max_status_publish_s = 0.0
        self._last_record = None
        self._thread_name = None
        self._native_tid = None
        self.first_sample = threading.Event()
        self.reader = ImuSnapshotReader(
            str(self.get_parameter('snapshot_path').value) or None,
            timeout_s=float(self.get_parameter('snapshot_timeout_s').value),
            should_stop=lambda: not rclpy.ok())
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos)
        self.status_pub = self.create_publisher(String, '/imu/status', 2)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._publish)
        self.create_timer(1.0, self._publish_status)

    def destroy_node(self):
        self.reader.close()
        return super().destroy_node()

    def _publish(self) -> None:
        self._thread_name = threading.current_thread().name
        self._native_tid = threading.get_native_id()
        entered = time.monotonic()
        if self._last_callback_mono is not None:
            self._max_callback_gap_s = max(self._max_callback_gap_s,
                                           entered - self._last_callback_mono)
        self._last_callback_mono = entered
        sample_begin = time.monotonic()
        record = self.reader.read()
        self._max_sample_wait_s = max(self._max_sample_wait_s,
                                      time.monotonic() - sample_begin)
        if record is None or record['control_fault'] or not record['valid']:
            return
        # Reading a newer record only refreshes the publisher's view; the
        # acquisition time carried in the record is never renewed.
        if self._last_record is not None and record['stamp_ns'] <= self._last_record['stamp_ns']:
            return
        self._last_record = record
        quat = np.asarray(record['quat'])
        sample = ImuSnapshot(gyro=np.asarray(record['gyro']),
                             accel=np.asarray(record['accel']),
                             quat_wxyz=quat, stamp_ns=record['stamp_ns'])
        gyro, accel = self._noise.apply(sample)
        if (not np.isfinite(gyro).all() or not np.isfinite(accel).all()
                or not np.isfinite(quat).all()
                or not np.isclose(np.linalg.norm(quat), 1.0, atol=0.01)):
            return
        msg = Imu()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(record['stamp_ns'], 10**9)
        msg.header.frame_id = self._frame_id
        w, x, y, z = (float(v) for v in quat)
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
        msg.orientation_covariance = BODY_ORIENTATION_COVARIANCE
        _fill_imu(msg, gyro, accel)
        publish_begin = time.monotonic()
        try:
            sent = publish_sample(self, self.imu_pub, msg, '/imu/data_raw')
        finally:
            self._max_publish_call_s = max(self._max_publish_call_s,
                                           time.monotonic() - publish_begin)
        if not sent:
            return
        published_at = time.monotonic()
        if self._last_publish_mono is not None:
            self._max_publish_gap_s = max(self._max_publish_gap_s,
                                          published_at - self._last_publish_mono)
        self._last_publish_mono = published_at
        self._published += 1
        self.first_sample.set()

    def _publish_status(self) -> None:
        begin = time.monotonic()
        record = self._last_record
        publish_sample(self, self.status_pub, String(data=json.dumps(dict(
            simulated=True, sensor='mujoco', frame_id=self._frame_id,
            published=self._published, subscribers=self.imu_pub.get_subscription_count(),
            publish_thread=self._thread_name, publish_native_tid=self._native_tid,
            qos_depth=self.imu_pub.qos_profile.depth,
            last_publish_age_s=(None if self._last_publish_mono is None else
                                time.monotonic() - self._last_publish_mono),
            max_publish_gap_s=self._max_publish_gap_s,
            max_sample_wait_s=self._max_sample_wait_s,
            max_callback_gap_s=self._max_callback_gap_s,
            max_publish_call_s=self._max_publish_call_s,
            max_status_publish_s=self._max_status_publish_s,
            max_gc_pause_s=0.0,
            publish_failures=dict(getattr(self, '_publish_failures', {})),
            control_fault=(None if record is None else record['control_fault']),
            clock_synced=True,
            separate_process=True))), '/imu/status')
        self._max_status_publish_s = max(self._max_status_publish_s,
                                         time.monotonic() - begin)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SplitBodyImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
