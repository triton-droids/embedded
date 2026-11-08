#!/usr/bin/env python3
"""Shadow-mode fall predictor: scores live /imu/data_raw, publishes nothing.

Read-only ROS node. It subscribes to the body IMU, the latched Safety estop and
/sim/status, runs the trained dual-horizon TorchScript model on every sample
once 20 samples are buffered, and applies each head's saved threshold and
consecutive-frame confirmation online. It never publishes, calls services or
touches Safety; alarms only go to the log file.

Features are computed exactly like humanoid_sim.collection.EpisodeRecorder:
roll/pitch from the published orientation quaternion, then the published
angular_velocity and linear_acceleration.

Online gating mirrors training eligibility as closely as live data allows:
no estop and no gantry in the 20-sample window, consecutive header-stamp gaps
within [5, 20] ms. Gantry state comes from 1 Hz /sim/status, so the first
<= 1 s after release may be gated out; offline analysis re-applies the exact
per-sample flags recorded by the collector.

Usage (same RMW/domain as the simulation; any Python with torch + rclpy):
  shadow_node.py --model trial_03/model_torchscript.pt --best best.json --out shadow.jsonl
"""
import argparse
import collections
import json
import math
import time

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

HISTORY = 20
HEADS = ('300ms', '600ms')  # model output columns 0 and 1


def features(msg):
    q = msg.orientation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(float(np.clip(2 * (qw * qy - qz * qx), -1, 1)))
    g, a = msg.angular_velocity, msg.linear_acceleration
    return [roll, pitch, g.x, g.y, g.z, a.x, a.y, a.z]


class Shadow(Node):
    def __init__(self, args):
        super().__init__('fall_predictor_shadow')
        torch.set_num_threads(1)
        self.model = torch.jit.load(args.model).eval()
        policies = json.load(open(args.best))['validation_policies']
        self.policy = {h: (float(policies[h]['threshold']), int(policies[h]['confirm_frames']))
                       for h in HEADS}
        self.out = open(args.out, 'w', buffering=1)
        self.window = collections.deque(maxlen=HISTORY)  # (stamp_ns, features, estop, held)
        self.estop = False
        self.base_held = True
        self.run = {h: 0 for h in HEADS}
        self.alarm = {h: False for h in HEADS}
        self.count = 0
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, latched)
        self.create_subscription(String, '/sim/status', self.on_status, 10)
        self.log(kind='start', policy=self.policy, model=args.model)

    def log(self, **row):
        row['recv_mono'] = time.monotonic()
        row['recv_wall_ns'] = time.time_ns()
        self.out.write(json.dumps(row) + '\n')

    def on_estop(self, msg):
        if bool(msg.data) != self.estop:
            self.estop = bool(msg.data)
            self.log(kind='estop', value=self.estop)

    def on_status(self, msg):
        held = bool(json.loads(msg.data).get('base_held', True))
        if held != self.base_held:
            self.base_held = held
            self.log(kind='base_held', value=held)

    def reset(self):
        self.window.clear()
        for h in HEADS:
            self.run[h] = 0
            self.alarm[h] = False

    def on_imu(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self.window and not 0 < stamp - self.window[-1][0] < 10**9:
            self.reset()  # new launch or long silence: never mix histories
            self.log(kind='reset', stamp_ns=stamp)
        f = features(msg)
        self.window.append((stamp, f, self.estop, self.base_held))
        if len(self.window) < HISTORY:
            return
        stamps = np.array([w[0] for w in self.window], dtype=np.int64)
        gaps = np.diff(stamps) / 1e9
        eligible = (not any(w[2] or w[3] for w in self.window)
                    and bool(np.all(gaps >= .005 - 1e-9)) and bool(np.all(gaps <= .02 + 1e-9)))
        x = torch.tensor([[w[1] for w in self.window]], dtype=torch.float32)
        t0 = time.perf_counter()
        with torch.no_grad():
            p = torch.sigmoid(self.model(x))[0].tolist()
        infer_ms = (time.perf_counter() - t0) * 1e3
        onsets = []
        for i, h in enumerate(HEADS):
            thr, confirm = self.policy[h]
            self.run[h] = self.run[h] + 1 if (eligible and p[i] >= thr) else 0
            active = self.run[h] >= confirm
            if active and not self.alarm[h]:
                onsets.append(h)
            self.alarm[h] = active
        self.log(kind='score', stamp_ns=stamp, f=f, p300=p[0], p600=p[1], eligible=eligible,
                 infer_ms=round(infer_ms, 4), alarm300=self.alarm['300ms'],
                 alarm600=self.alarm['600ms'])
        for h in onsets:
            self.log(kind='alarm', head=h, stamp_ns=stamp, p=p[HEADS.index(h)])
            self.get_logger().warn(f'SHADOW fall alarm ({h}), p={p[HEADS.index(h)]:.3f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--best', required=True, help='best.json with validation_policies')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    rclpy.init()
    node = Shadow(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.out.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
