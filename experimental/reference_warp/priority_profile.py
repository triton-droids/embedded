"""Experimental IMU > WBC > other scheduling; explicit readback, no sudo.

Affinity partitions only this launched stack, not unrelated host tasks/IRQs.
Nice mode is relative CFS weighting, not a deadline guarantee. FIFO mode must
be explicitly available; it never silently falls back after setup.
"""
import argparse
import json
import os
from pathlib import Path
import resource
import sys
import threading

ENV = 'HUMANOID_PRIORITY_CONFIG'


def cpu_groups(allowed=None, topology_root=Path('/sys/devices/system/cpu')):
    allowed = sorted(os.sched_getaffinity(0) if allowed is None else allowed)
    cores = {}
    for cpu in allowed:
        path = topology_root/f'cpu{cpu}'/'topology'
        key = ((path/'physical_package_id').read_text().strip(),
               (path/'core_id').read_text().strip())
        cores.setdefault(key, []).append(cpu)
    physical = list(cores.values())
    if len(physical) < 3:
        # Do not starve the remaining chain on one-/two-core allocations.
        return dict(imu=allowed, wbc=allowed, other=allowed)
    return dict(imu=physical[0], wbc=physical[1],
                other=[cpu for group in physical[2:] for cpu in group])


def fifo_available():
    caps = next(line.split()[1] for line in Path('/proc/self/status').read_text().splitlines()
                if line.startswith('CapEff:'))
    return bool(int(caps, 16) & (1 << 23)) or resource.getrlimit(resource.RLIMIT_RTPRIO)[0] >= 70


def make_profile(mode='auto'):
    if mode not in ('auto','off','nice','fifo'):
        raise ValueError(f'Unknown scheduling mode: {mode}')
    if mode == 'off':
        return None
    available = fifo_available()
    if mode == 'fifo' and not available:
        raise RuntimeError('FIFO requested but CAP_SYS_NICE/RLIMIT_RTPRIO>=70 is unavailable')
    resolved = 'fifo' if mode == 'fifo' or mode == 'auto' and available else 'nice'
    # IMU starts from the original nice=0. Raising inherited niceness back to
    # zero needs privilege; fail instead of claiming a hierarchy not applied.
    if os.getpriority(os.PRIO_PROCESS, 0) != 0:
        raise RuntimeError('Start the launch at nice=0 to establish IMU=0 > WBC=5 > other=10')
    return dict(mode=resolved, cpus=cpu_groups(), nice=dict(imu=0,wbc=5,other=10),
                fifo=dict(imu=70,wbc=60,other=0))


def configure_current_thread(role):
    config = os.environ.get(ENV)
    if not config:
        return None
    profile = json.loads(config)
    if role == 'bootstrap':
        # Simulator creates independent execution threads from this base.
        return None
    group = 'imu' if role in ('imu','imu_source','safety') else 'wbc' if role == 'wbc' else 'other'
    cpus = profile['cpus'][group]
    os.sched_setaffinity(0, cpus)
    policy = os.SCHED_FIFO if profile['mode'] == 'fifo' and group != 'other' else os.SCHED_OTHER
    os.sched_setscheduler(0, policy, os.sched_param(profile['fifo'][group] if policy == os.SCHED_FIFO else 0))
    os.setpriority(os.PRIO_PROCESS, 0, profile['nice'][group])
    result = dict(role=role, mode=profile['mode'], pid=os.getpid(), tid=threading.get_native_id(),
                  policy=os.sched_getscheduler(0), priority=os.sched_getparam(0).sched_priority,
                  nice=os.getpriority(os.PRIO_PROCESS,0), cpus=sorted(os.sched_getaffinity(0)))
    if result['policy'] != policy or result['nice'] != profile['nice'][group] or result['cpus'] != sorted(cpus):
        raise RuntimeError(f'Scheduling readback mismatch: {result}')
    print('[priority-profile] '+json.dumps(result), file=sys.stderr, flush=True)
    return result


def classify(command):
    names = [Path(part).name for part in command]
    if 'safety_node' in names:
        return 'safety'
    if 'mujoco_sim' in names:
        return 'bootstrap'
    if 'wbc_control_node' in names:
        return 'wbc'
    return 'other'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    command = parser.parse_args().command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('executable required after --')
    configure_current_thread(classify(command))
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
