#!/usr/bin/env python3
"""Offline evaluation of the MPC + WBC walking controller (no ROS).

Each episode is a full MuJoCo simulation plus TSID at 200 Hz in one process
(~one CPU core). Runs are sequential unless --workers is given; keep workers
well below the core count, several parallel episodes can saturate a laptop.

  walk_eval.py fixed --length 0.06 --duration 20
  walk_eval.py variable
  walk_eval.py sweep --workers 2          # delay x friction x step length
"""
import argparse
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from walking_controller.offline import load, run
from walking_controller.stepping import StepParams, SteppingController
from walking_controller.wbc import WBCGains

GAINS = dict(com_kp_z=100.0, com_weight=10.0)


def episode(duration=20.0, step_length=0.0, delay_steps=2, friction=None, schedule=None):
    """Returns steps, fault, forward distance and per-segment progress."""
    paths, robot, limits = load()
    ctl = SteppingController(robot, limits.torque,
                             StepParams(step_length_m=step_length, gains=WBCGains(**GAINS)))
    marks = []

    def on_step(t, sim, c):
        if schedule:
            for ts, length in schedule:
                if abs(t - ts) < sim.dt / 2:
                    c.p.step_length_m = length
                    marks.append((ts, length, c.rm.com[:2].copy(), getattr(c, 'steps', 0)))
        if not marks and getattr(c, 'fwd_axis', None) is not None:
            marks.append((t, c.p.step_length_m, c.rm.com[:2].copy(), 0))
        return False

    res = run(ctl, duration, paths=paths, robot=robot, limits=limits, delay_steps=delay_steps,
              friction=friction, on_step=on_step)
    fwd = getattr(ctl, 'fwd_axis', np.array([0.0, 1.0, 0.0]))[:2]
    marks.append((res['t_end'], None, ctl.rm.com[:2].copy(), getattr(ctl, 'steps', 0)))
    segments = [(t0, length, s1 - s0, float((c1 - c0) @ fwd))
                for (t0, length, c0, s0), (_, _, c1, s1) in zip(marks, marks[1:])]
    return {'steps': getattr(ctl, 'steps', 0), 'fault': ctl.fault, 'fallen': res['fallen'],
            't_end': round(res['t_end'], 2),
            'forward_m': round(float((marks[-1][2] - marks[0][2]) @ fwd), 3), 'segments': segments}


def _sweep_job(args):
    delay, friction, length = args
    return args, episode(20.0, length, delay, friction)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    f = sub.add_parser('fixed')
    f.add_argument('--length', type=float, default=0.0)
    f.add_argument('--duration', type=float, default=20.0)
    f.add_argument('--delay-steps', type=int, default=2)
    sub.add_parser('variable')
    s = sub.add_parser('sweep')
    s.add_argument('--workers', type=int, default=1)
    args = ap.parse_args()

    if args.cmd == 'fixed':
        r = episode(args.duration, args.length, args.delay_steps)
        print({k: v for k, v in r.items() if k != 'segments'})
    elif args.cmd == 'variable':
        plan = ((4, 0.0), (8, 0.05), (12, 0.10), (16, 0.02), (20, 0.08), (24, 0.0))
        r = episode(28.0, schedule=plan)
        print({k: v for k, v in r.items() if k != 'segments'})
        for t0, length, steps, dist in r['segments']:
            print(f'  from {t0:4.1f} s: cmd {length} m/step -> {steps} steps, {dist:+.2f} m')
    else:
        jobs = [(d, fr, L) for d in (2, 3, 4) for fr in (0.65, 1.15) for L in (0.0, 0.06)]
        if args.workers > 1:
            with ProcessPoolExecutor(args.workers) as pool:
                results = list(pool.map(_sweep_job, jobs))
        else:
            results = [_sweep_job(j) for j in jobs]
        for (d, fr, L), r in results:
            ok = not r['fault'] and not r['fallen']
            print(f'delay {d * 5:2d} ms friction {fr:.2f} step {L:.2f}: {"OK" if ok else "FAIL"} '
                  f'steps={r["steps"]} t_end={r["t_end"]} {r["fault"]}')


if __name__ == '__main__':
    main()
