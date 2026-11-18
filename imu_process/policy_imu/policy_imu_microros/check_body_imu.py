#!/usr/bin/env python3
"""Read-only ROS IMU acceptance probe; never publishes commands or enables motors.

Run after sourcing ROS, with the sensor stationary and body axes upright. The
default checks data/clock/tilt contracts, not physical mounting correctness.
Use --stationary-upright for the additional static acceleration/gyro check.
"""
import argparse
import json
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class Probe(Node):
    def __init__(self, args):
        super().__init__('body_imu_probe')
        self.args = args
        self.count = self.bad = self.reordered = 0
        self.last_stamp = None
        self.receipts, self.ages, self.acc_norms, self.gyro_norms = [], [], [], []
        self.up_errors = []
        self.latest = None
        self.status = None
        # Reliable subscription intentionally verifies compatibility with Safety.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Imu, args.topic, self.on_imu, qos)
        self.create_subscription(String, '/imu/status', self.on_status, qos)

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            self.status = {'invalid_status': msg.data}

    def on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        q = msg.orientation
        a, w = msg.linear_acceleration, msg.angular_velocity
        values = [q.w, q.x, q.y, q.z, a.x, a.y, a.z, w.x, w.y, w.z]
        self.count += 1
        self.receipts.append(time.monotonic())
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.reordered += 1
        self.last_stamp = stamp
        if (not all(math.isfinite(v) for v in values) or stamp <= 0
                or not -0.01 <= age <= 0.1 or msg.header.frame_id != 'imu_link'
                or msg.orientation_covariance[0] < 0
                or abs(math.sqrt(sum(v*v for v in values[:4])) - 1) > 0.01):
            self.bad += 1
            return
        self.ages.append(age * 1000)
        an = math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z)
        wn = math.sqrt(w.x*w.x + w.y*w.y + w.z*w.z)
        self.acc_norms.append(an)
        self.gyro_norms.append(wn)
        # Up predicted by q_WB inverse rotation; compare to measured specific
        # force only while stationary (translation also affects accelerometers).
        up = (2*(q.x*q.z-q.w*q.y), 2*(q.y*q.z+q.w*q.x),
              1-2*(q.x*q.x+q.y*q.y))
        if an > 0:
            dot = sum(u*v/an for u, v in zip(up, (a.x, a.y, a.z)))
            self.up_errors.append(math.degrees(math.acos(max(-1, min(1, dot)))))
        roll = math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))
        pitch = math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))))
        self.latest = {'roll_deg': round(roll, 2), 'pitch_deg': round(pitch, 2),
                       'acc_mps2': [round(v, 3) for v in (a.x, a.y, a.z)],
                       'gyro_radps': [round(v, 4) for v in (w.x, w.y, w.z)]}

    def result(self):
        rate = ((len(self.receipts)-1)/(self.receipts[-1]-self.receipts[0])
                if len(self.receipts) > 1 else 0)
        gaps = [b-a for a, b in zip(self.receipts, self.receipts[1:])]
        silence = time.monotonic()-self.receipts[-1] if self.receipts else self.args.seconds
        max_gap = max(gaps + [silence])
        passed = (self.count >= 20 and self.bad == 0 and self.reordered == 0
                  and rate >= self.args.min_hz and max_gap <= 0.1)
        if self.args.stationary_upright:
            passed = (passed and bool(self.latest) and bool(self.up_errors)
                      and 8.8 < statistics.mean(self.acc_norms) < 10.8
                      and max(self.gyro_norms) < 0.15
                      and statistics.mean(self.up_errors) < 10
                      and abs(self.latest['roll_deg']) < 10
                      and abs(self.latest['pitch_deg']) < 10)
        return {'passed': passed, 'samples': self.count, 'rate_hz': round(rate, 2),
                'invalid': self.bad, 'non_increasing_stamp': self.reordered,
                'max_receipt_gap_ms': round(max_gap*1000, 2),
                'age_ms_mean': round(statistics.mean(self.ages), 2) if self.ages else None,
                'age_ms_max': round(max(self.ages), 2) if self.ages else None,
                'static_gravity_error_deg_mean': round(statistics.mean(self.up_errors), 2)
                    if self.up_errors else None,
                'latest': self.latest, 'mcu_status': self.status}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/imu/data_raw')
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--min-hz', type=float, default=50)
    parser.add_argument('--stationary-upright', action='store_true')
    args = parser.parse_args()
    if args.seconds <= 0 or args.min_hz <= 0:
        parser.error('duration and minimum rate must be positive')
    rclpy.init()
    probe = Probe(args)
    end = time.monotonic() + args.seconds
    next_status = time.monotonic() + 2
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(probe, timeout_sec=0.1)
            if time.monotonic() >= next_status:
                print(json.dumps({'samples': probe.count, 'latest': probe.latest,
                                  'mcu_status': probe.status}), flush=True)
                next_status += 2
        result = probe.result()
        print(json.dumps(result, indent=2), flush=True)
        return 0 if result['passed'] else 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
