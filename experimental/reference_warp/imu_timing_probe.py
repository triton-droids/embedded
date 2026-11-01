"""Bounded observer for IMU source/receipt ages and first-fault chronology."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from motor_control_interfaces.msg import AbstractCommandStatus, MotorCommand


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('imu_timing_probe')
    start = time.monotonic()
    data = dict(imu=[], status={},clock=dict(start_monotonic_s=start,start_wall_s=time.time()))
    data['motor_commands'] = dict(samples=0, mode_counts={}, transitions=[], last_non_disable_t=None)

    def motor_command(msg):
        state = data['motor_commands']
        now = time.monotonic()-start
        modes = sorted(set(int(mode) for mode in msg.mode))
        key = ','.join(map(str, modes))
        state['samples'] += 1
        state['mode_counts'][key] = state['mode_counts'].get(key, 0)+1
        if not state['transitions'] or state['transitions'][-1]['modes'] != modes:
            state['transitions'].append(dict(t=now,modes=modes))
        if any(mode != MotorCommand.MODE_DISABLE for mode in modes):
            state['last_non_disable_t'] = now

    def imu(msg):
        now = time.monotonic()
        source = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        data['imu'].append(dict(t=now-start, source=source, age=time.time()-source))

    def status(topic, value):
        data['status'].setdefault(topic,[]).append(dict(t=time.monotonic()-start,value=value))

    node.create_subscription(Imu,'/imu/data_raw',imu,QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(MotorCommand,'/motor_commands',motor_command,QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    for topic in ('/imu/status','/sim/status','/policy/status','/wbc/status','/experimental/chase_status','/ball_chase/status','/can_timing'):
        node.create_subscription(String,topic,lambda m,t=topic:status(t,json.loads(m.data)),QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(DiagnosticArray,'/safety/status',lambda m:status('/safety/status',[
        dict(name=s.name,message=s.message,values={v.key:v.value for v in s.values}) for s in m.status]),QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(AbstractCommandStatus, '/control_core/abstract_command_status',
        lambda m:status('/control_core/abstract_command_status',dict(stage=int(m.stage),message=m.message)),
        QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        while time.monotonic()-start<args.duration:
            rclpy.spin_once(node,timeout_sec=.02)
    finally:
        gaps=[b['t']-a['t'] for a,b in zip(data['imu'],data['imu'][1:])]
        data['summary']=dict(samples=len(data['imu']),max_receipt_gap_s=max(gaps,default=0),
                             max_source_age_s=max((v['age'] for v in data['imu']),default=0))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(data,indent=2)+'\n')
        print(json.dumps(data['summary']),flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
