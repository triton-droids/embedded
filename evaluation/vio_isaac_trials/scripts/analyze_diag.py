"""Summarize mujoco_sim DIAG lines (per-second maxima) before and after the gantry release.

  python3 analyze_diag.py RUN_DIR [...]
"""
import json
import re
import sys

import numpy as np

KEYS = ['rx_age_ms', 'loop_gap_ms', 'tx_age_ms', 'cmd_age_ms', 'gil_ms', 'cmd_lock_ms', 'step_ms', 'step_late_ms', 'cam_scene_ms', 'cam_render_ms']


def release_time(run):
    import os
    path = f'{run}/autostart.log' if os.path.exists(f'{run}/autostart.log') else f'{run}/sim.log'
    for line in open(path):
        m = re.search(r'\[(17\d+\.\d+)\].*Gantry released', line)
        if m:
            return float(m.group(1))
    return None


def pause_time(run):
    for line in open(f'{run}/sim.log'):
        m = re.search(r'\[(17\d+\.\d+)\].*Physics paused', line)
        if m:
            return float(m.group(1))
    return None


def main():
    for run in sys.argv[1:]:
        rows = [json.loads(line.split('DIAG ', 1)[1]) for line in open(f'{run}/sim.log')
                if line.startswith('[mujoco_sim-1] DIAG ')]
        cpp = [json.loads(line.split('CPPDIAG ', 1)[1]) for line in open(f'{run}/sim.log') if 'CPPDIAG ' in line]
        # Merge cpp_control_node's per-second line into the simulator's nearest second.
        for r in rows:
            near = min(cpp, key=lambda c: abs(c['t'] - r['t'])) if cpp else None
            if near is not None and abs(near['t'] - r['t']) < 0.6:
                for k in ('rx_age_ms', 'tx_age_ms', 'loop_gap_ms'):
                    r[k] = near[k]
        rel, pause = release_time(run), pause_time(run)
        end = pause if pause is not None else float('inf')
        print(f'==== {run}: release {rel}, pause {pause} ({(pause - rel) if rel and pause else None} s after release)')
        for label, sel in (('held (last 15 s)', lambda r: rel and rel - 15 <= r['t'] < rel),
                           ('walking', lambda r: rel and rel <= r['t'] < end)):
            part = [r for r in rows if sel(r)]
            print(f'  {label}: {len(part)} s')
            for k in KEYS:
                v = np.array([r[k] for r in part if k in r])
                if len(v):
                    print(f'    {k:14s} median {np.median(v):6.1f}  p90 {np.percentile(v, 90):6.1f}  max {v.max():6.1f}')
            n = [r.get('n_cmd', 0) for r in part]
            if n:
                print(f'    commands/s     median {np.median(n):.0f}  min {min(n)}')
        if pause:
            last = [r for r in rows if pause - 4 <= r['t'] <= pause + 1]
            print('  last seconds before the pause:')
            for r in last:
                print('    ' + ' '.join(f'{k}={r[k]}' for k in ['t'] + KEYS + ['n_cmd'] if k in r))


if __name__ == '__main__':
    main()
