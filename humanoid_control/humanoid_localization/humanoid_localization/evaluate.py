"""localization_eval: compare /localization/odom with simulator ground truth (evaluation only).

  ros2 run humanoid_localization localization_eval --ros-args -p duration_s:=120.0 -p csv:=/tmp/loc.csv

Each estimate is paired with the ground-truth sample nearest in stamp (max 20 ms
apart). Reports horizontal position error, yaw error, ground-truth path length
and drift per metre travelled.
"""
from __future__ import annotations

import bisect
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .fusion import quat_to_matrix, yaw_of


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _sample(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return _stamp_s(msg.header.stamp), np.array([p.x, p.y]), yaw_of(quat_to_matrix((q.w, q.x, q.y, q.z)))


class LocalizationEval(Node):
    def __init__(self):
        super().__init__('localization_eval')
        self.declare_parameter('estimate_topic', '/localization/odom')
        self.declare_parameter('truth_topic', '/sim/ground_truth/odom')
        self.declare_parameter('duration_s', 0.0)  # 0: until Ctrl+C.
        self.declare_parameter('csv', '')
        self.truth_t, self.truth = [], []
        self.rows = []
        self.create_subscription(Odometry, self.get_parameter('truth_topic').value, self._on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.get_parameter('estimate_topic').value,
                                 self._on_estimate, qos_profile_sensor_data)
        duration = float(self.get_parameter('duration_s').value)
        self.done = False
        if duration > 0:
            self.create_timer(duration, self._finish)

    def _finish(self):
        self.done = True

    def _on_truth(self, msg):
        t, xy, yaw = _sample(msg)
        if self.truth_t and t <= self.truth_t[-1]:
            return
        self.truth_t.append(t)
        self.truth.append((xy, yaw))

    def _on_estimate(self, msg):
        t, xy, yaw = _sample(msg)
        i = bisect.bisect_left(self.truth_t, t)
        candidates = [j for j in (i - 1, i) if 0 <= j < len(self.truth_t)]
        if not candidates:
            return
        j = min(candidates, key=lambda k: abs(self.truth_t[k] - t))
        if abs(self.truth_t[j] - t) > 0.02:
            return
        true_xy, true_yaw = self.truth[j]
        yaw_error = math.atan2(math.sin(yaw - true_yaw), math.cos(yaw - true_yaw))
        self.rows.append((t, *xy, *true_xy, float(np.linalg.norm(xy - true_xy)), yaw_error))

    def summary(self) -> dict:
        if not self.rows:
            return {'samples': 0}
        a = np.asarray(self.rows)
        errors, yaw = a[:, 5], np.abs(a[:, 6])
        path = float(np.sum(np.linalg.norm(np.diff(a[:, 3:5], axis=0), axis=1))) if len(a) > 1 else 0.0
        return {
            'samples': int(len(a)), 'duration_s': round(float(a[-1, 0] - a[0, 0]), 2),
            'truth_path_m': round(path, 3),
            'position_error_m': {'median': round(float(np.median(errors)), 3),
                                 'p95': round(float(np.percentile(errors, 95)), 3),
                                 'max': round(float(errors.max()), 3), 'final': round(float(errors[-1]), 3)},
            'final_error_per_m_travelled': round(float(errors[-1]) / path, 4) if path > 0.1 else None,
            'yaw_error_deg': {'median': round(math.degrees(float(np.median(yaw))), 2),
                              'p95': round(math.degrees(float(np.percentile(yaw, 95))), 2),
                              'max': round(math.degrees(float(yaw.max())), 2)},
        }

    def write_csv(self, path):
        with open(path, 'w') as f:
            f.write('stamp_s,x,y,true_x,true_y,position_error_m,yaw_error_rad\n')
            for row in self.rows:
                f.write(','.join(f'{v:.6f}' for v in row) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationEval()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    print(json.dumps(node.summary(), indent=2))
    csv = node.get_parameter('csv').value
    if csv:
        node.write_csv(csv)
    node.destroy_node()
    rclpy.try_shutdown()
