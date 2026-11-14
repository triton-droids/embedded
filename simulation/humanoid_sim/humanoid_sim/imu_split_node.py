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

The same process publishes ``/head/imu`` (node ``sim_head_imu``) when the model
has a head: the camera-rig IMU that cuVSLAM fuses needs every physics sample
on time, which the simulator process cannot guarantee.
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

from .imu_nodes import (BODY_ORIENTATION_COVARIANCE, HEAD_IMU_PREFIX, _ImuNoise, _RateGate,
                        _YawError, _fill_imu, declare_head_imu, head_imu_message)
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
        self._yaw_error = _YawError(self)
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
        w, x, y, z = (float(v) for v in self._yaw_error.apply(quat, record['stamp_ns']))
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


class SplitHeadImuNode(Node):
    """Head (camera-rig) IMU from the same shared segment, every physics sample.

    Polls at twice ``head_imu_rate_hz`` and publishes each new sample once, so at
    the physics rate no sample is skipped or repeated. Publishes nothing while
    the segment has no head data (headless model) or under a control fault.
    """

    def __init__(self, reader: ImuSnapshotReader) -> None:
        super().__init__('sim_head_imu')
        self.declare_parameter('seed', 2)
        rate, self._frame_id = declare_head_imu(self)
        self._noise = _ImuNoise(self, int(self.get_parameter('seed').value), HEAD_IMU_PREFIX)
        self._gate = _RateGate(rate)
        self.reader = reader
        self.published = 0
        self.imu_pub = self.create_publisher(Imu, '/head/imu', QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.5 / rate, self._publish)

    def _publish(self) -> None:
        record = self.reader.read()
        if (record is None or not record['valid'] or not record['head_valid']
                or record['control_fault'] or not self._gate.accept(record['stamp_ns'])):
            return
        sample = ImuSnapshot(gyro=np.asarray(record['head_gyro']),
                             accel=np.asarray(record['head_accel']),
                             quat_wxyz=None, stamp_ns=record['stamp_ns'])
        gyro, accel = self._noise.apply(sample)
        if not np.isfinite(gyro).all() or not np.isfinite(accel).all():
            return
        if publish_sample(self, self.imu_pub,
                          head_imu_message(record['stamp_ns'], self._frame_id, gyro, accel),
                          '/head/imu'):
            self.published += 1


def main(args=None) -> None:
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init(args=args)
    node = SplitBodyImuNode()
    head = SplitHeadImuNode(node.reader)
    executor = SingleThreadedExecutor()  # One thread: the two nodes share the reader.
    executor.add_node(node)
    executor.add_node(head)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        head.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
