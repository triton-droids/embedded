"""Compare simulator / walking-controller timing and GPU load between runs.

  python3 analyze_status.py RUN_DIR [RUN_DIR ...]
Uses sim_status.log, walk_status.log (ros2 topic echo --field data) and tegra_full.log.
Only samples while walking (gantry released, not paused, controller active) count.
"""
import json
import re
import sys

import numpy as np


def records(path):
    out = []
    for line in open(path):
        line = line.strip()
        if line.startswith('{'):
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float('nan')


def main():
    for run in sys.argv[1:]:
        sim = [r for r in records(f'{run}/sim_status.log') if not r.get('base_held') and not r.get('physics_paused')]
        walk = [r for r in records(f'{run}/walk_status.log') if r.get('state') == 'active' and not r.get('base_held')]
        rtf = np.array([r['real_time_factor'] for r in sim])
        overruns = np.array([r['physics_overruns'] for r in sim])
        odom_cb = np.array([r.get('max_odom_callback_s', np.nan) for r in sim]) * 1e3

        def field(name, stat):
            return np.array([r[name][stat] for r in walk if isinstance(r.get(name), dict)])

        cyc99, cycmax = field('cycle_ms', 'p99'), field('cycle_ms', 'max')
        in99, inmax = field('input_age_ms', 'p99'), field('input_age_ms', 'max')
        j99, jmax = field('joint_age_ms', 'p99'), field('joint_age_ms', 'max')
        gpu, cpu = [], []
        for line in open(f'{run}/tegra_full.log'):
            m = re.search(r'GR3D_FREQ (\d+)%', line)
            c = re.search(r'CPU \[([^\]]*)\]', line)
            if m:
                gpu.append(int(m.group(1)))
            if c:
                cpu.append(np.mean([int(x.split('%')[0]) for x in c.group(1).split(',') if '%' in x]))
        span = (overruns[-1] - overruns[0]) if len(overruns) > 1 else float('nan')
        print(f'==== {run}: {len(sim)} sim samples, {len(walk)} walk samples while walking')
        print(f'  sim real-time factor     median {np.median(rtf):.3f}  p5 {pct(rtf, 5):.3f}  min {rtf.min():.3f}')
        print(f'  physics overruns         +{span:.0f} over {len(sim)} s  ({span / max(len(sim), 1):.1f} /s)')
        print(f'  max odom callback (ms)   median {np.nanmedian(odom_cb):.1f}  max {np.nanmax(odom_cb):.1f}')
        print(f'  walk cycle_ms            p99 median {np.median(cyc99):.1f}  max of max {cycmax.max():.1f}')
        print(f'  walk input_age_ms        p99 median {np.median(in99):.1f}  p99 worst {in99.max():.1f}  max {inmax.max():.1f}')
        print(f'  walk joint_age_ms        p99 median {np.median(j99):.1f}  p99 worst {j99.max():.1f}  max {jmax.max():.1f}')
        print(f'  GPU GR3D                 median {np.median(gpu):.0f}%  p95 {pct(gpu, 95):.0f}%')
        print(f'  CPU mean over cores      median {np.median(cpu):.0f}%')


if __name__ == '__main__':
    main()
