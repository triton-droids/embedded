#!/usr/bin/env python3
"""Passive timing probe for the MuJoCo simulation (read-only, no commands sent).

Measures what the policy actually experiences: arrival jitter and age of the
feedback streams, physics overruns / real-time factor reported by /sim/status,
camera frame rate (via the tiny camera_info topic, so the probe itself does not
pull 0.9 MB images), and CPU use of the simulation processes.

All nodes in this simulation stamp with wall-clock ROS time (no /clock), so
age = receive wall time - header.stamp is meaningful on one host.

Usage:
  python3 timing_probe.py --seconds 30 --out result.json
"""
from __future__ import annotations

import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Imu, JointState
from std_msgs.msg import String

try:
    from motor_control_interfaces.msg import MotorCommand
except ImportError:  # Workspace not sourced: skip the policy stream.
    MotorCommand = None

# Best effort + deep queue: never back-pressure a Reliable publisher.
PROBE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                       reliability=ReliabilityPolicy.BEST_EFFORT)

STREAMS = {
    # topic: (message type, nominal Hz, has header)
    '/joint_states': (JointState, 50.0),
    '/imu/data_raw': (Imu, 100.0),
    '/camera/camera/color/camera_info': (CameraInfo, 15.0),
}
if MotorCommand is not None:
    STREAMS['/policy/desired_motor_subset'] = (MotorCommand, 50.0)

CLK_TCK = os.sysconf('SC_CLK_TCK')
PROC_PATTERNS = ('mujoco_sim', 'tracking_policy', 'cpp_control_node', 'safety_node',
                 'wbc_control_node', 'track_node', 'ball_detection', 'motor_web')


def percentile(values, q):
    if not values:
        return None
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(round(q / 100.0 * (len(s) - 1)))))
    return s[k]


def summarize_ms(values):
    if not values:
        return None
    return {'n': len(values), 'p50': round(percentile(values, 50) * 1e3, 3),
            'p95': round(percentile(values, 95) * 1e3, 3),
            'p99': round(percentile(values, 99) * 1e3, 3),
            'max': round(max(values) * 1e3, 3)}


def proc_cpu_ticks():
    """{label: total utime+stime ticks} for matching processes, plus system totals."""
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
            with open(f'/proc/{pid}/stat') as f:
                fields = f.read().rsplit(')', 1)[1].split()
        except OSError:
            continue
        for pat in PROC_PATTERNS:
            if pat in cmd and 'timing_probe' not in cmd:
                out[pat] = out.get(pat, 0) + int(fields[11]) + int(fields[12])
                break
    with open('/proc/stat') as f:
        cpu = [int(v) for v in f.readline().split()[1:]]
    return out, sum(cpu), cpu[3] + cpu[4]  # total, idle+iowait


class Probe(Node):

    def __init__(self):
        super().__init__('timing_probe')
        self.arrivals = {t: [] for t in STREAMS}
        self.ages = {t: [] for t in STREAMS}
        self.status = []
        for topic, (msg_type, _) in STREAMS.items():
            self.create_subscription(msg_type, topic, self._make_cb(topic), PROBE_QOS)
        self.create_subscription(String, '/sim/status', self._on_status, 10)

    def _make_cb(self, topic):
        def cb(msg):
            now = time.time()
            self.arrivals[topic].append(time.monotonic())
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp > 0:
                self.ages[topic].append(now - stamp)
        return cb

    def _on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        data['_recv'] = time.monotonic()
        self.status.append(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--out', required=True)
    parser.add_argument('--label', default='')
    args = parser.parse_args()

    rclpy.init()
    probe = Probe()
    cpu0, sys0, idle0 = proc_cpu_ticks()
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(probe, timeout_sec=0.05)
    elapsed = time.monotonic() - t0
    cpu1, sys1, idle1 = proc_cpu_ticks()

    streams = {}
    for topic, (_, nominal) in STREAMS.items():
        arr = probe.arrivals[topic]
        gaps = [b - a for a, b in zip(arr, arr[1:])]
        period = 1.0 / nominal
        streams[topic] = {
            'nominal_hz': nominal,
            'measured_hz': round(len(arr) / elapsed, 2) if arr else 0.0,
            'interarrival_ms': summarize_ms(gaps),
            'gaps_over_2x_period': sum(g > 2 * period for g in gaps),
            'gaps_over_50ms': sum(g > 0.05 for g in gaps),
            'age_ms': summarize_ms(probe.ages[topic]),
        }

    st = probe.status
    status = None
    if st:
        rtf = [s.get('real_time_factor') for s in st if s.get('real_time_factor')]
        status = {
            'samples': len(st),
            'physics_overruns_delta': st[-1].get('physics_overruns', 0) - st[0].get('physics_overruns', 0),
            'rtf_min': min(rtf) if rtf else None,
            'rtf_mean': round(sum(rtf) / len(rtf), 4) if rtf else None,
            'base_held_end': st[-1].get('base_held'),
            'control_fault_end': st[-1].get('control_fault'),
            'estop_end': st[-1].get('estop_active'),
        }

    cpu = {name: round(100.0 * (cpu1.get(name, 0) - cpu0.get(name, 0)) / CLK_TCK / elapsed, 1)
           for name in set(cpu0) | set(cpu1)}
    busy = 1.0 - (idle1 - idle0) / max(1, sys1 - sys0)
    result = {'label': args.label, 'seconds': round(elapsed, 2), 'streams': streams,
              'sim_status': status, 'process_cpu_percent_of_one_core': cpu,
              'system_cpu_busy_percent': round(100.0 * busy, 1), 'ncpu': os.cpu_count()}
    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
