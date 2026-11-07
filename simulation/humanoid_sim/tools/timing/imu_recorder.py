#!/usr/bin/env python3
"""Record every /imu/data_raw arrival: receive wall time and header stamp.

The simulator stamps each IMU sample when it publishes it, so a gap between
consecutive *stamps* means the simulator did not publish (it was stalled);
a large receive-minus-stamp means this recorder received it late.

Usage: imu_recorder.py OUT_FILE SECONDS
Output lines: "I <receive wall time> <stamp>"
"""
import sys
import time

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def main():
    out_path, seconds = sys.argv[1], float(sys.argv[2])
    rclpy.init()
    node = rclpy.create_node('timing_imu_recorder')
    rows = []
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=500,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(
        Imu, '/imu/data_raw',
        lambda m: rows.append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)),
        qos)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    with open(out_path, 'w') as out:
        out.writelines(f'I {r:.4f} {s:.4f}\n' for r, s in rows)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
