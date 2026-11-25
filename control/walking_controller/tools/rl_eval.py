"""Evaluate a policy on an RL task (deterministic actions) and optionally save its successful end states as
start states for the next task (get-up end states -> balance, balance end states -> walk).

  tools/rl_eval.py --task getup --policy rl/getup/latest.npz --episodes 60 [--save-states getup_end.npz]
  tools/rl_eval.py --task walk --policy rl/walk/latest.npz --episodes 30 --task-arg 'vx=[0.2,0.2]'

No torch needed (numpy policy). Prints the success rate by start label and episode statistics as JSON.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rl_env import TaskEnv  # noqa: E402
from rl_train import task_kwargs  # noqa: E402
from walking_controller.getup_policy import GetUpPolicy  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--task', required=True, choices=('getup', 'balance', 'walk'))
    p.add_argument('--task-arg', action='append', default=[])
    p.add_argument('--policy', type=Path, required=True)
    p.add_argument('--episodes', type=int, default=60)
    p.add_argument('--envs', type=int, default=8)
    p.add_argument('--seed', type=int, default=12345)
    p.add_argument('--no-dr', action='store_true')
    p.add_argument('--save-states', type=Path)
    a = p.parse_args()
    pol = GetUpPolicy(a.policy)
    env = TaskEnv(a.envs, a.seed, a.task, task_kwargs(a.task_arg), not a.no_dr, pool=max(40, a.episodes))
    if pol.history != env.history or pol.command_dim != env.command_dim:
        raise SystemExit('policy history / command size does not match the task')
    obs, *_ = env.reset()
    done_eps, states = [], {'qpos': [], 'qvel': []}
    while len(done_eps) < a.episodes:
        act = np.stack([pol(o) for o in obs]) * pol.scale
        obs, _, _, _, infos = env.step(act)
        for e in infos:
            if 'end_qpos' in e:
                states['qpos'].append(e.pop('end_qpos'))
                states['qvel'].append(e.pop('end_qvel'))
            done_eps.append(e)
    done_eps = done_eps[:a.episodes]
    by = {}
    for e in done_eps:
        by.setdefault(e['start'], []).append(e)
    out = dict(task=a.task, policy=str(a.policy), episodes=len(done_eps),
               success=float(np.mean([e['success'] for e in done_eps])),
               by_start={k: dict(n=len(v), success=float(np.mean([e['success'] for e in v])),
                                 mean_steps=float(np.mean([e['steps'] for e in v]))) for k, v in by.items()},
               fell=float(np.mean([e.get('fell', False) for e in done_eps])))
    if a.task == 'walk':
        out['mean_vx_err'] = float(np.mean([e.get('vx_err', np.nan) for e in done_eps]))
    if a.save_states and states['qpos']:
        np.savez(a.save_states, qpos=np.array(states['qpos']), qvel=np.array(states['qvel']))
        out['saved_states'] = len(states['qpos'])
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
