#!/usr/bin/env python3
"""Measure camera image delivery like ball_detection_node receives it.

Subscribes to color and depth with SensorDataQoS (best effort, depth 1) as raw
CDR bytes, so Python deserialization of ~1 MB messages does not slow the
subscriber down. The simulator publishes camera_info with the same stamp for
every rendered frame; a small reliable camera_info subscription is the
reference for "frames sent". A frame sent but never received was lost in
transport.

Usage: image_probe.py SECONDS
"""
import struct
import sys
import time

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu

COLOR = '/camera/camera/color/image_raw'
DEPTH = '/camera/camera/aligned_depth_to_color/image_raw'
INFO = '/camera/camera/color/camera_info'


def pct(values, q):
    values = sorted(values)
    return values[min(len(values) - 1, int(q * (len(values) - 1)))] if values else float('nan')


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    node = rclpy.create_node('timing_image_probe')
    data = {'color': [], 'depth': [], 'info': []}
    imu = []

    def raw_cb(key):
        def cb(raw):  # CDR: 4-byte encapsulation header, then stamp sec:int32, nanosec:uint32
            sec, nsec = struct.unpack_from('<iI', raw, 4)
            data[key].append((time.time(), round(sec + nsec * 1e-9, 4), len(raw)))
        return cb

    sensor = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Image, COLOR, raw_cb('color'), sensor, raw=True)
    node.create_subscription(Image, DEPTH, raw_cb('depth'), sensor, raw=True)
    node.create_subscription(CameraInfo, INFO, raw_cb('info'),
                             QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                                        reliability=ReliabilityPolicy.RELIABLE), raw=True)
    node.create_subscription(
        Imu, '/imu/data_raw', lambda m: imu.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
        QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)

    sent = sorted({s for _, s, _ in data['info']})
    if len(sent) < 10:
        print('camera_info not received: is enable_vision:=true?')
        return
    sent = sent[2:-2]  # skip partial frames at the window edges
    src_gaps = [b - a for a, b in zip(sent, sent[1:])]
    print(f'| stream | Hz | MB/frame | latency p50/p99/max ms | lost in transport |')
    print('|---|---|---|---|---|')
    print(f'| sent (camera_info) | {len(data["info"]) / seconds:.1f} | - | - | '
          f'source gaps > 100 ms: {sum(g > 0.1 for g in src_gaps)} |')
    for key in ('color', 'depth'):
        rows = data[key]
        if not rows:
            print(f'| {key} | 0 | - | - | all (no subscription match?) |')
            continue
        got = {s for _, s, _ in rows}
        lost = sum(s not in got for s in sent)
        lat = [1e3 * (r - s) for r, s, _ in rows]
        print(f'| {key} | {len(rows) / seconds:.1f} | {rows[0][2] / 1e6:.2f} | '
              f'{pct(lat, .5):.1f}/{pct(lat, .99):.1f}/{max(lat):.1f} | '
              f'{lost}/{len(sent)} ({100 * lost / len(sent):.1f} %) |')
    imu.sort()
    gaps = [b - a for a, b in zip(imu, imu[1:])]
    print(f'\nIMU: {len(imu) / seconds:.1f} Hz, max stamp gap {1e3 * max(gaps):.0f} ms')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
