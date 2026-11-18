#!/usr/bin/env python3
"""Differential test: python_can_node vs robstride_can_node on a virtual CAN bus.

Runs one backend at a time against the same scripted MotorCommand/estop sequence
and a fake RobStride responder on vcan, records every frame the backend puts on
the bus, and compares the two traces per motor (consecutive repeats collapsed,
since the number of 50 Hz retransmissions depends on scheduling). Also compares
the last joint_states each backend published. vcan only; refuses physical CAN.

  sudo ip link add vcandiff type vcan && sudo ip link set vcandiff up
  python3 can_backend_diff.py --registry config/motors.yaml --channel vcandiff
"""
import argparse
import json
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time

import can
import rclpy
import yaml
from motor_control_interfaces.msg import MotorCommand
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

HOST = 255


class FakeMotors:
    """Answers like RobStride firmware: a type-2 status after each command."""

    def __init__(self, channel, devices):
        self.bus = can.Bus(interface='socketcan', channel=channel, receive_own_messages=False)
        self.devices = set(devices)
        self.enabled = {d: False for d in devices}
        self.raw = {d: [32767, 32767, 32767] for d in devices}  # position, velocity, torque
        self.trace = []
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def reply(self, device):
        mode = 2 if self.enabled[device] else 0
        extra = (mode << 14) | device
        p, v, t = self.raw[device]
        data = struct.pack('>HHHH', p, v, t, 300)  # 30.0 C
        self.bus.send(can.Message(arbitration_id=(2 << 24) | (extra << 8) | HOST,
                                  is_extended_id=True, data=data))

    def loop(self):
        while self.running:
            msg = self.bus.recv(timeout=0.05)
            if msg is None or not msg.is_extended_id:
                continue
            kind = (msg.arbitration_id >> 24) & 31
            extra = (msg.arbitration_id >> 8) & 0xFFFF
            device = msg.arbitration_id & 0xFF
            if device not in self.devices:
                continue  # our own replies or unrelated traffic
            self.trace.append((time.monotonic(), msg.arbitration_id, bytes(msg.data).hex()))
            if kind == 3:
                self.enabled[device] = True
                self.reply(device)
            elif kind == 4:
                self.enabled[device] = False
                self.reply(device)
            elif kind == 1:
                p, v, _kp, _kd = struct.unpack('>HHHH', msg.data)
                self.raw[device] = [p, v, extra]
                self.reply(device)

    def stop(self):
        self.running = False
        self.thread.join()
        self.bus.shutdown()


def scenario(node, pub, estop_pub, joints):
    """Deterministic command script; returns when done."""
    def send(mode, **values):
        msg = MotorCommand()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.joint_name = list(joints)
        msg.mode = [mode]
        for key, value in values.items():
            setattr(msg, key, [float(v) for v in value])
        pub.publish(msg)

    def run(seconds, fn):
        end = time.monotonic() + seconds
        k = 0
        while time.monotonic() < end:
            fn(k)
            k += 1
            time.sleep(0.02)

    n = len(joints)
    estop_pub.publish(Bool(data=False))
    time.sleep(0.5)
    run(0.5, lambda k: send(MotorCommand.MODE_DISABLE))
    send(MotorCommand.MODE_ENABLE)
    time.sleep(0.3)
    run(1.0, lambda k: send(MotorCommand.MODE_MOTION,
                            position=[0.3 * math.sin(0.1 * k + i) for i in range(n)],
                            velocity=[0.5 * math.cos(0.1 * k + i) for i in range(n)],
                            torque=[0.4 * (i - n / 2) for i in range(n)],
                            kp=[20.0 + i for i in range(n)], kd=[0.5 + 0.05 * i for i in range(n)]))
    run(0.4, lambda k: send(MotorCommand.MODE_POSITION,
                            position=[0.1 * i - 0.2 for i in range(n)],
                            kp=[15.0] * n, kd=[0.8] * n))
    run(0.4, lambda k: send(MotorCommand.MODE_VELOCITY,
                            velocity=[0.2 * (i % 3) for i in range(n)], acceleration=[2.0] * n))
    time.sleep(0.9)  # commands stop: the 0.5 s transport watchdog must disable
    send(MotorCommand.MODE_ENABLE)
    time.sleep(0.3)
    run(0.3, lambda k: send(MotorCommand.MODE_MOTION, position=[0.05 * k] * n, kp=[10.0] * n,
                            kd=[0.3] * n))
    estop_pub.publish(Bool(data=True))
    time.sleep(0.4)
    run(0.2, lambda k: send(MotorCommand.MODE_MOTION, position=[0.0] * n, kp=[10.0] * n,
                            kd=[0.3] * n))  # must be rejected while estop is latched
    estop_pub.publish(Bool(data=False))
    time.sleep(0.4)


def run_backend(executable, registry_path, channel, joints, devices):
    motors = FakeMotors(channel, devices)
    proc = subprocess.Popen(
        ['ros2', 'run', 'motor_control_hybrid', executable, '--ros-args',
         '-p', f'motor_config_file:={registry_path}', '-p', 'publish_rate_hz:=50.0',
         '-p', 'command_timeout_s:=0.5'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
    rclpy.init()
    node = rclpy.create_node(f'diff_driver_{executable}')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MotorCommand, 'motor_commands', 10)
    estop_pub = node.create_publisher(Bool, '/safety/estop', latched)
    states, timing = [], []
    node.create_subscription(JointState, 'joint_states', states.append, 10)
    node.create_subscription(String, 'can_timing', lambda m: timing.append(json.loads(m.data)), 10)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    deadline = time.monotonic() + 15
    while node.count_subscribers('motor_commands') == 0 and time.monotonic() < deadline:
        time.sleep(0.1)
    time.sleep(1.0)
    started = time.monotonic()
    scenario(node, pub, estop_pub, joints)
    time.sleep(1.2)  # let one more can_timing arrive
    os.killpg(proc.pid, signal.SIGINT)
    try:
        out, _ = proc.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        out, _ = proc.communicate()
    motors.stop()
    node.destroy_node()
    rclpy.shutdown()
    trace = [row for row in motors.trace if row[0] >= started]
    return dict(trace=trace, states=states, timing=timing[-1] if timing else None, log=out)


def collapse(trace):
    per_motor = {}
    for _, frame_id, data in trace:
        device = frame_id & 0xFF
        seq = per_motor.setdefault(device, [])
        item = (f'{frame_id:08x}', data)
        if not seq or seq[-1] != item:
            seq.append(item)
    return per_motor


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--registry', required=True)
    ap.add_argument('--channel', default='vcandiff')
    ap.add_argument('--out', default='/tmp/can_backend_diff.json')
    args = ap.parse_args()
    info = json.loads(subprocess.check_output(['ip', '-details', '-json', 'link', 'show', args.channel]))
    if info[0].get('linkinfo', {}).get('info_kind') != 'vcan':
        sys.exit('Refusing physical CAN: this differential test is vcan-only')
    raw = yaml.safe_load(open(args.registry))
    params = raw.get('motor_control_node', {}).get('ros__parameters', raw)
    params['hardware_verified'] = True  # vcan-only copy
    params['default_can_interface'] = args.channel
    for cfg in params['motors'].values():
        cfg['can_interface'] = args.channel
    joints = list(params['motors'])
    devices = [cfg['motor_id'] for cfg in params['motors'].values()]
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(raw, f, sort_keys=False)
        registry_path = f.name

    results = {}
    for backend in ('python_can_node', 'robstride_can_node'):
        results[backend] = run_backend(backend, registry_path, args.channel, joints, devices)
        print(f'{backend}: {len(results[backend]["trace"])} frames, '
              f'{len(results[backend]["states"])} joint_states, timing={results[backend]["timing"]}')

    py, cc = (collapse(results[b]['trace']) for b in ('python_can_node', 'robstride_can_node'))
    mismatches = 0
    for device in sorted(set(py) | set(cc)):
        a, b = py.get(device, []), cc.get(device, [])
        if a != b:
            mismatches += 1
            first = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), min(len(a), len(b)))
            print(f'motor {device}: python {len(a)} vs cpp {len(b)} distinct frames; first '
                  f'difference at {first}: {a[first:first + 3]} vs {b[first:first + 3]}')
        else:
            kinds = sorted({int(x[0], 16) >> 24 for x in a})
            print(f'motor {device}: identical, {len(a)} distinct frames, types {kinds}')

    def last_state(states):
        for msg in reversed(states):
            if msg.name:
                return dict(zip(msg.name, zip(msg.position, msg.velocity, msg.effort)))
        return {}

    sp, sc = last_state(results['python_can_node']['states']), last_state(results['robstride_can_node']['states'])
    state_diff = max((abs(x - y) for j in sp for x, y in zip(sp[j], sc.get(j, (math.nan,) * 3))),
                     default=math.nan)
    print(f'last joint_states: {len(sp)} vs {len(sc)} joints, max |difference| {state_diff:.3g}')
    json.dump({b: dict(collapsed={str(k): v for k, v in collapse(r['trace']).items()},
                       timing=r['timing'], log=r['log'][-4000:])
               for b, r in results.items()}, open(args.out, 'w'), indent=1)
    ok = mismatches == 0 and len(sp) == len(sc) == len(joints) and state_diff < 1e-9
    print('PASS' if ok else 'FAIL', f'(details in {args.out})')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
