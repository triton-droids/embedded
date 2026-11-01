"""Serial normal-GUI WBC trials; records every attempt, including startup failures.

Run from an activated workspace. Uses existing policy -> WBC -> C++ -> simulator
launch without changing any deadlines or safety configuration. No hardware I/O.
"""
import argparse
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import time

from prepare import prepare


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scales', type=float, nargs='+', default=[1.0, 0.95, 1.05])
    parser.add_argument('--repetitions', type=int, default=2)
    parser.add_argument('--domain', type=int, default=181)
    parser.add_argument('--web-port', type=int, default=8181)
    parser.add_argument('--duration', type=float, default=10.0, help='Release-relative wall seconds')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    root = Path(__file__).resolve().parent
    results = root/'results'/time.strftime('%Y%m%d-%H%M%S')
    results.mkdir(parents=True)
    variants = {}
    for scale in args.scales:
        variants[scale] = prepare(scale, root/'generated'/f'scale_{scale:.3f}')
    rclpy.init()
    node = Node('reference_warp_trial_monitor')
    active = {}

    def record(topic, value):
        active.setdefault(topic, []).append(dict(t=time.monotonic()-active['begin'], value=value))

    def sim(msg):
        value = json.loads(msg.data)
        record('sim', value)
        if value.get('base_held') is False and active.get('released') is None:
            active['released'] = time.monotonic()

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        record('odom', dict(position=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w]))

    def safety(msg):
        record('safety', [dict(name=s.name, level=(int.from_bytes(s.level) if isinstance(s.level, bytes)
                                                 else int(s.level)), message=s.message,
                             values={v.key: v.value for v in s.values}) for s in msg.status])

    node.create_subscription(String, '/sim/status', sim, QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(String, '/policy/status',
                             lambda m: record('policy', json.loads(m.data)), QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(Odometry, '/sim/ground_truth/odom', odom, QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(DiagnosticArray, '/safety/status', safety, QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    summaries = []
    try:
        for repetition in range(args.repetitions):
            for scale in args.scales:
                # Drain discovery/history from the previous completed process.
                active.clear()
                active.update(begin=time.monotonic(), released=None)
                model, registry, manifest = variants[scale]
                label = f'rep{repetition+1}_scale{scale:.3f}'
                log_path = results/f'{label}.log'
                command = ['ros2', 'launch', 'wbc_control_node', 'rl_wbc_sim.launch.py',
                           'auto_start:=true', 'use_viewer:=true',
                           f'ros_domain_id:={args.domain}', f'web_port:={args.web_port}',
                           f'model_path:={model}', f'motor_config_file:={registry}']
                active['begin'] = time.monotonic()
                def gui_windows():
                    tree = subprocess.run(['xwininfo', '-root', '-tree'],
                                          capture_output=True, text=True, timeout=3)
                    return set(re.findall(r'(0x[0-9a-f]+) .*\("MuJoCo" "MuJoCo"\)', tree.stdout))
                old_windows = gui_windows()
                with log_path.open('w') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True)
                    gui_seen = False
                    try:
                        next_gui_check = 0.0
                        while process.poll() is None:
                            rclpy.spin_once(node, timeout_sec=0.05)
                            now = time.monotonic()
                            if now >= next_gui_check:
                                gui_seen |= bool(gui_windows()-old_windows)
                                next_gui_check = now+1
                            released = active['released']
                            if (released is not None and now-released >= args.duration
                                    or released is None and now-active['begin'] >= 25):
                                break
                    finally:
                        if process.poll() is None:
                            os.killpg(process.pid, signal.SIGINT)
                            try:
                                process.wait(timeout=12)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGTERM)
                                process.wait(timeout=5)
                text = log_path.read_text()
                release_offset = (None if active['released'] is None else
                                  active['released']-active['begin'])
                samples = [v for v in active.get('odom', [])
                           if release_offset is not None and v['t'] >= release_offset]
                latched = [v for v in active.get('safety', [])
                           if any(s['values'].get('latched') == 'True' for s in v['value'])]
                fall = any('fall' in s['values'].get('reason', '').lower()
                           for v in latched for s in v['value'])
                faults = [v for v in active.get('policy', []) if v['value'].get('fault')]
                sim_faults = [v for v in active.get('sim', []) if v['value'].get('control_fault')]
                # A fall also has a direct physical criterion, independent of safety wording.
                physical_fall = any(v['value']['position'][2] < 0.35 for v in samples)
                confirmed_fall = fall or physical_fall
                invalid_gui = not gui_seen or 'viewer unavailable' in text.lower()
                if invalid_gui:
                    category = 'gui_invalid'
                elif release_offset is None:
                    category = 'not_released'
                elif confirmed_fall:
                    category = 'confirmed_fall'
                elif latched or faults or sim_faults:
                    category = 'censored_stop'
                else:
                    category = 'observed_no_fall'
                displacement = None
                if samples:
                    first, last = samples[0]['value']['position'], samples[-1]['value']['position']
                    displacement = [last[i]-first[i] for i in range(3)]
                summary = dict(label=label, manifest=manifest, category=category,
                               gui_seen=gui_seen, release_offset_s=release_offset,
                               exposure_wall_s=(None if not samples else samples[-1]['t']-samples[0]['t']),
                               displacement_world_m=displacement,
                               min_base_z_m=(None if not samples else min(v['value']['position'][2] for v in samples)),
                               final_policy=active.get('policy', [None])[-1],
                               final_safety=active.get('safety', [None])[-1],
                               log=str(log_path), command=command)
                active['summary'] = summary
                (results/f'{label}.json').write_text(json.dumps(active, indent=2)+'\n')
                summaries.append(summary)
                (results/'summary.json').write_text(json.dumps(summaries, indent=2)+'\n')
                print(json.dumps({k: summary[k] for k in ('label', 'category', 'release_offset_s',
                                                         'exposure_wall_s', 'displacement_world_m')}) , flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f'Results: {results}', flush=True)


if __name__ == '__main__':
    main()
