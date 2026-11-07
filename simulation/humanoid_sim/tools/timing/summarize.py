#!/usr/bin/env python3
"""Print a Markdown summary table of timing_probe results in one directory."""
import glob
import json
import os
import sys


def fmt(stats, key):
    return '-' if not stats else f"{stats[key]:.1f}"


def main(directory):
    rows = []
    for path in sorted(glob.glob(os.path.join(directory, '*.json'))):
        with open(path) as f:
            rows.append(json.load(f))
    env = os.path.join(directory, 'environment.txt')
    if os.path.exists(env):
        with open(env) as f:
            head = [' '.join(l.split()) for l in f
                    if l.startswith(('launch', 'platform', 'Model name', 'net.core', 'git'))
                    or 'renderer' in l]
        print('Environment: ' + ' | '.join(head) + '\n')

    print('| run | joint p99/max ms | joint gaps>50ms | IMU p99/max ms | IMU gaps>50ms '
          '| IMU age p99 ms | policy p99/max ms | camera Hz | overruns | RTF min | '
          'mujoco_sim CPU% | system CPU% | end state |')
    print('|' + '---|' * 13)
    for r in rows:
        s = r['streams']
        j = s.get('/joint_states', {})
        i = s.get('/imu/data_raw', {})
        p = s.get('/policy/desired_motor_subset', {})
        c = s.get('/camera/camera/color/camera_info', {})
        st = r.get('sim_status') or {}
        ji, ii, pi = j.get('interarrival_ms'), i.get('interarrival_ms'), p.get('interarrival_ms')
        end = []
        if st.get('control_fault_end'):
            end.append(f"fault:{st['control_fault_end']}")
        if st.get('estop_end'):
            end.append('estop')
        if st.get('base_held_end') is False:
            end.append('released')
        print(f"| {r['label']} | {fmt(ji,'p99')}/{fmt(ji,'max')} | {j.get('gaps_over_50ms','-')} "
              f"| {fmt(ii,'p99')}/{fmt(ii,'max')} | {i.get('gaps_over_50ms','-')} "
              f"| {fmt(i.get('age_ms'),'p99')} | {fmt(pi,'p99')}/{fmt(pi,'max')} "
              f"| {c.get('measured_hz','-')} | {st.get('physics_overruns_delta','-')} "
              f"| {st.get('rtf_min','-')} "
              f"| {r['process_cpu_percent_of_one_core'].get('mujoco_sim','-')} "
              f"| {r['system_cpu_busy_percent']} | {', '.join(end) or 'ok'} |")
    print('\nNominal: joint 20 ms, IMU 10 ms, policy 20 ms. CPU% of mujoco_sim is per '
          'one core (100 = one full core; >100 means several threads).')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '.')
