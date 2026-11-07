#!/usr/bin/env python3
"""Summarize startup_trials.sh results: estops and simulator IMU stalls per trial.

T is the simulator's "Simulation ready" time. Stamp gaps before T+1 s happen
while the camera renderer starts, before any IMU check is armed, and are
reported separately; gaps after T+1 s are the ones that can latch an estop.

Usage: startup_report.py RESULT_DIR
"""
import glob
import os
import re
import sys

GAP_S = 0.1


def ready_time(trial_log):
    """Simulation ready time from the launch's own log file."""
    match = re.search(r'All log files can be found below (\S+)', open(trial_log).read())
    if not match:
        return None
    for line in open(os.path.join(match.group(1), 'launch.log')):
        if 'Simulation ready' in line:
            return float(line.split()[0])
    return None


def main(directory):
    trials = sorted(glob.glob(os.path.join(directory, 'trial_*.log')),
                    key=lambda p: int(re.search(r'trial_(\d+)', p).group(1)))
    estops = late_trials = 0
    rows = []
    for log in trials:
        i = re.search(r'trial_(\d+)', log).group(1)
        text = open(log).read()
        estop = re.search(r'SAFETY ESTOP LATCHED: ([^\n]*)', text)
        policy_fault = 'IMU timeout' in text
        T = ready_time(log)
        rec = os.path.join(directory, f'trial_{i}.imu')
        stamps = sorted(float(l.split()[2]) for l in open(rec)) if os.path.exists(rec) else []
        gaps = [(a, b - a) for a, b in zip(stamps, stamps[1:]) if b - a > GAP_S]
        early = [g for t, g in gaps if T is None or t - T <= 1.0]
        late = [(t - T, g) for t, g in gaps if T is not None and t - T > 1.0]
        estops += bool(estop or policy_fault)
        late_trials += bool(late)
        reason = (estop.group(1)[:70] if estop else '') + (' policy IMU timeout' if policy_fault else '')
        rows.append(f"| {i} | {len(stamps)} | {', '.join(f'{1e3*g:.0f}' for g in early) or '-'} | "
                    f"{', '.join(f'T+{t:.2f}s {1e3*g:.0f} ms' for t, g in late) or '-'} | "
                    f"{reason.strip() or 'ok'} |")
    env = os.path.join(directory, 'environment.txt')
    if os.path.exists(env):
        print('```\n' + open(env).read().strip() + '\n```\n')
    print('| trial | IMU samples | early gaps ms (<T+1 s) | stalls after T+1 s | estop / fault |')
    print('|---|---|---|---|---|')
    print('\n'.join(rows))
    n = len(trials)
    print(f'\n**{estops}/{n} trials estopped or faulted; {late_trials}/{n} had a simulator '
          f'IMU stall > {1e3*GAP_S:.0f} ms after T+1 s.**')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '.')
