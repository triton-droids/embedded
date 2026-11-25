"""PPO for the full-body RL tasks (rl_env.TaskEnv + rl_tasks/<task>.py): MuJoCo in CPU worker processes,
networks in PyTorch (CPU or CUDA).

  tools/rl_train.py --task getup --init bc3.npz --out rl/getup --workers 12 --envs-per-worker 8
  tools/rl_train.py --task walk --out rl/walk --workers 10 --task-arg states_file=balance_end.npz
  tools/rl_train.py --task getup --out rl/getup --resume          # continue from rl/getup/checkpoint.pt

--init: a GetUpPolicy npz (getup_bc.py or an earlier run) with the same observation size; its normalisation is
kept fixed. Without it the actor starts fresh and the observation normalisation is estimated during the first
--norm-iters iterations, then frozen. The critic also sees privileged state (rl_env.TaskEnv.priv).

Writes OUT/policy_<iter>.npz and OUT/latest.npz (GetUpPolicy format, with task and command_dim), OUT/log.jsonl
(one line per iteration: return, success rate by start label, timing) and OUT/checkpoint.pt.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_name, '1')
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rl_env import VecEnv  # noqa: E402


class RunningNorm:
    def __init__(self, dim):
        self.mean, self.var, self.count = np.zeros(dim), np.ones(dim), 1e-4

    def update(self, x):
        m, v, n = x.mean(0), x.var(0), len(x)
        delta, total = m - self.mean, self.count + n
        self.mean = self.mean + delta * n / total
        self.var = (self.var * self.count + v * n + delta ** 2 * self.count * n / total) / total
        self.count = total

    def __call__(self, x):
        return np.clip((x - self.mean) / np.sqrt(self.var + 1e-8), -10, 10)


def mlp(torch, sizes):
    layers = []
    for k in range(len(sizes) - 1):
        layers.append(torch.nn.Linear(sizes[k], sizes[k + 1]))
        if k < len(sizes) - 2:
            layers.append(torch.nn.ELU())
    return torch.nn.Sequential(*layers)


def load_actor(torch, path, obs_dim, act_dim):
    z = np.load(path)
    if z['w0'].shape[1] != obs_dim or z[f'w{int(z["layers"]) - 1}'].shape[0] != act_dim:
        raise SystemExit(f'--init {path}: observation/action size {z["w0"].shape[1]} does not match {obs_dim}')
    sizes = [obs_dim] + [z[f'w{k}'].shape[0] for k in range(int(z['layers']))]
    actor = mlp(torch, sizes)
    linear = [m for m in actor if isinstance(m, torch.nn.Linear)]
    for k, lin in enumerate(linear):
        lin.weight.data[:] = torch.as_tensor(z[f'w{k}'])
        lin.bias.data[:] = torch.as_tensor(z[f'b{k}'])
    return actor, z['obs_mean'].astype(np.float64), z['obs_std'].astype(np.float64)


def save_policy(path, actor, mean, std, meta):
    linear = [m for m in actor if hasattr(m, 'weight')]
    weights = {f'w{i}': m.weight.detach().cpu().numpy() for i, m in enumerate(linear)}
    weights.update({f'b{i}': m.bias.detach().cpu().numpy() for i, m in enumerate(linear)})
    tmp = path.with_suffix('.tmp.npz')
    np.savez(tmp, layers=len(linear), obs_mean=np.asarray(mean, np.float32), obs_std=np.asarray(std, np.float32),
             **meta, **weights)
    tmp.replace(path)


def task_kwargs(items):
    out = {}
    for item in items:
        key, value = item.split('=', 1)
        try:
            out[key] = json.loads(value)
        except json.JSONDecodeError:
            out[key] = value
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--task', required=True, choices=('getup', 'balance', 'walk'))
    p.add_argument('--task-arg', action='append', default=[], help='key=value (JSON values allowed)')
    p.add_argument('--init', type=Path)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--envs-per-worker', type=int, default=8)
    p.add_argument('--steps', type=int, default=48)
    p.add_argument('--iters', type=int, default=20000)
    p.add_argument('--hidden', type=int, nargs='+', default=[512, 256, 128])
    p.add_argument('--norm-iters', type=int, default=300)
    p.add_argument('--critic-warmup', type=int, default=30)
    p.add_argument('--lr-actor', type=float, default=None, help='default 5e-5 with --init, 3e-4 fresh')
    p.add_argument('--lr-critic', type=float, default=3e-4)
    p.add_argument('--init-std', type=float, default=None, help='default 0.15 with --init, 0.4 fresh')
    p.add_argument('--gamma', type=float, default=0.99)
    p.add_argument('--lam', type=float, default=0.95)
    p.add_argument('--epochs', type=int, default=4)
    p.add_argument('--minibatches', type=int, default=4)
    p.add_argument('--clip', type=float, default=0.2)
    p.add_argument('--target-kl', type=float, default=0.02)
    p.add_argument('--no-dr', action='store_true')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--device', default='cpu')
    p.add_argument('--save-every', type=int, default=25)
    p.add_argument('--resume', action='store_true')
    a = p.parse_args()
    import torch
    torch.set_num_threads(2 if a.device.startswith('cuda') else 4)
    torch.manual_seed(a.seed)
    dev = torch.device(a.device)
    a.out.mkdir(parents=True, exist_ok=True)
    kwargs = task_kwargs(a.task_arg)
    env = VecEnv(a.workers, a.envs_per_worker, a.seed, a.task, kwargs, not a.no_dr)
    obs, priv, _, _, _ = env.reset()
    E, obs_dim, act_dim = len(obs), obs.shape[1], 18
    ckpt = a.out / 'checkpoint.pt'
    resume = a.resume and ckpt.exists()
    if a.init and not resume:
        actor, o_mean, o_std = load_actor(torch, a.init, obs_dim, act_dim)
        norm_fixed = True
    else:
        actor = mlp(torch, [obs_dim] + a.hidden + [act_dim])
        with torch.no_grad():
            actor[-1].weight.mul_(0.01)
            actor[-1].bias.zero_()
        o_mean, o_std, norm_fixed = np.zeros(obs_dim), np.ones(obs_dim), False
    fresh = not a.init
    lr_actor = a.lr_actor or (3e-4 if fresh else 5e-5)
    init_std = a.init_std or (0.4 if fresh else 0.15)
    actor = actor.to(dev)
    critic = mlp(torch, [obs_dim + priv.shape[1], 512, 256, 128, 1]).to(dev)
    log_std = torch.nn.Parameter(torch.full((act_dim,), math.log(init_std), device=dev))
    opt_a = torch.optim.Adam(list(actor.parameters()) + [log_std], lr=lr_actor)
    opt_c = torch.optim.Adam(critic.parameters(), lr=a.lr_critic)
    norm_a, norm_c, start = RunningNorm(obs_dim), RunningNorm(obs_dim + priv.shape[1]), 0
    if resume:
        c = torch.load(ckpt, map_location=dev, weights_only=False)
        actor.load_state_dict(c['actor'])
        critic.load_state_dict(c['critic'])
        log_std.data[:] = c['log_std']
        opt_a.load_state_dict(c['opt_a'])
        opt_c.load_state_dict(c['opt_c'])
        norm_c, start = c['norm_c'], c['iter'] + 1
        o_mean, o_std, norm_fixed, norm_a = c['o_mean'], c['o_std'], c['norm_fixed'], c['norm_a']
    from rl_env import load_task
    task = load_task(a.task, kwargs)
    meta = dict(scale=1.0, time_input=False, history=int(task.history), command_dim=int(task.command_dim),
                task=a.task)
    from walking_controller.getup_policy import GetUpPolicy  # noqa: F401  (format check only)
    tt = lambda x: torch.as_tensor(x, dtype=torch.float32, device=dev)  # noqa: E731
    log = (a.out / 'log.jsonl').open('a')
    (a.out / 'config.json').write_text(json.dumps(dict(vars(a), init=str(a.init), out=str(a.out),
                                                       task_kwargs=kwargs), indent=2) + '\n')
    print(f'{E} envs, obs {obs_dim}, priv {priv.shape[1]}, device {dev}, fresh actor {fresh}', flush=True)
    episodes = []
    joint_meta = None
    for it in range(start, a.iters):
        t0 = time.monotonic()
        if not norm_fixed and it >= a.norm_iters:
            o_mean, o_std, norm_fixed = norm_a.mean.copy(), np.sqrt(norm_a.var) + 1e-3, True
        B = {k: [] for k in ('obs', 'cin', 'act', 'logp', 'rew', 'done', 'val')}
        for _ in range(a.steps):
            if not norm_fixed:
                norm_a.update(obs)
                o_mean, o_std = norm_a.mean, np.sqrt(norm_a.var) + 1e-3
            cin = np.concatenate([obs, priv], 1)
            norm_c.update(cin)
            xn = (obs - o_mean) / o_std
            with torch.no_grad():
                mu = actor(tt(xn))
                dist = torch.distributions.Normal(mu, log_std.exp())
                act = dist.sample()
                logp = dist.log_prob(act).sum(-1)
                val = critic(tt(norm_c(cin))).squeeze(-1)
            obs2, priv2, rew, done, infos = env.step(act.cpu().numpy())
            for k, v in (('obs', xn), ('cin', norm_c(cin)), ('act', act), ('logp', logp), ('rew', rew),
                         ('done', done), ('val', val)):
                B[k].append(v)
            episodes += [{k: v for k, v in e.items() if not k.startswith('end_')} for e in infos]
            obs, priv = obs2, priv2
        t_roll = time.monotonic() - t0
        with torch.no_grad():
            last_val = critic(tt(norm_c(np.concatenate([obs, priv], 1)))).squeeze(-1)
        rew = tt(np.stack(B['rew'])); done = tt(np.stack(B['done']).astype(np.float32)); val = torch.stack(B['val'])
        adv = torch.zeros_like(rew)
        gae = torch.zeros(E, device=dev)
        for k in reversed(range(a.steps)):
            nxt = last_val if k == a.steps - 1 else val[k + 1]
            delta = rew[k] + a.gamma * nxt * (1 - done[k]) - val[k]
            gae = delta + a.gamma * a.lam * (1 - done[k]) * gae
            adv[k] = gae
        ret = adv + val
        f = lambda x: x.reshape(-1, *x.shape[2:])  # noqa: E731
        X, C = f(tt(np.stack(B['obs']))), f(tt(np.stack(B['cin'])))
        A, LP, R, ADV = f(torch.stack(B['act'])), f(torch.stack(B['logp'])), f(ret), f(adv)
        ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)
        N = len(X); mb = N // a.minibatches
        train_actor = it >= a.critic_warmup
        kl_hit, kls, closses = False, [], []
        for _ in range(a.epochs):
            perm = torch.randperm(N, device=dev)
            for j in range(a.minibatches):
                idx = perm[j * mb:(j + 1) * mb]
                closs = ((critic(C[idx]).squeeze(-1) - R[idx]) ** 2).mean()
                opt_c.zero_grad(); closs.backward()
                torch.nn.utils.clip_grad_norm_(critic.parameters(), 1.0); opt_c.step()
                closses.append(closs.item())
                if train_actor and not kl_hit:
                    dist = torch.distributions.Normal(actor(X[idx]), log_std.exp())
                    lp = dist.log_prob(A[idx]).sum(-1)
                    ratio = (lp - LP[idx]).exp()
                    aloss = -torch.min(ratio * ADV[idx], ratio.clamp(1 - a.clip, 1 + a.clip) * ADV[idx]).mean()
                    opt_a.zero_grad(); aloss.backward()
                    torch.nn.utils.clip_grad_norm_(list(actor.parameters()) + [log_std], 1.0); opt_a.step()
                    with torch.no_grad():
                        log_std.data.clamp_(math.log(0.03), math.log(1.0))
                    kl = (LP[idx] - lp).mean().item()
                    kls.append(kl)
                    kl_hit = kl > a.target_kl
        recent = episodes[-300:]
        by = {}
        for e in recent:
            by.setdefault(e['start'], []).append(e['success'])
        stats = dict(iter=it, sps=round(N / (time.monotonic() - t0)), t_roll=round(t_roll, 2),
                     reward=float(rew.mean()), episodes=len(episodes),
                     success=float(np.mean([e['success'] for e in recent])) if recent else 0.0,
                     success_by_start={k: round(float(np.mean(v)), 3) for k, v in by.items()},
                     ret=float(np.mean([e['ret'] for e in recent])) if recent else 0.0,
                     ep_steps=float(np.mean([e['steps'] for e in recent])) if recent else 0.0,
                     fell=float(np.mean([e.get('fell', False) for e in recent])) if recent else 0.0,
                     bad=float(np.mean([e['bad'] for e in recent])) if recent else 0.0,
                     closs=float(np.mean(closses)), kl=float(np.mean(kls)) if kls else 0.0,
                     std=float(log_std.detach().exp().mean()), norm_fixed=bool(norm_fixed))
        log.write(json.dumps(stats) + '\n'); log.flush()
        if it % 5 == 0:
            print(json.dumps(stats), flush=True)
        if it % a.save_every == 0 or it == a.iters - 1:
            if joint_meta is None:
                from rl_env import TaskEnv  # noqa: F401
                z = np.load(a.init) if a.init else None
                joint_meta = dict(q_nominal=env_q_nominal(), joint_names=env_joint_names()) if z is None else \
                    dict(q_nominal=z['q_nominal'], joint_names=z['joint_names'])
            m = {**meta, **joint_meta}
            save_policy(a.out / f'policy_{it:05d}.npz', actor, o_mean, o_std, m)
            save_policy(a.out / 'latest.npz', actor, o_mean, o_std, m)
            torch.save(dict(actor=actor.state_dict(), critic=critic.state_dict(), log_std=log_std.data,
                            opt_a=opt_a.state_dict(), opt_c=opt_c.state_dict(), norm_c=norm_c, norm_a=norm_a,
                            o_mean=o_mean, o_std=o_std, norm_fixed=norm_fixed, iter=it), ckpt)
    env.close()


def _robot():
    from getup_demo import ROOT, description_paths, load, make_controller
    paths, robot, limits = load(description_paths((ROOT / 'description' / 'humanoid_full_body_description').resolve()))
    return robot, make_controller(robot, limits).q_nominal.copy()


def env_q_nominal():
    return _robot()[1]


def env_joint_names():
    return np.array(_robot()[0].joint_names)


if __name__ == '__main__':
    main()
