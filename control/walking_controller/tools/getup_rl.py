"""PPO fine-tuning of the get-up student (full body, MuJoCo on CPU worker processes, networks in PyTorch).

The plant is the same as getup_demo.py: OfflineSim with the four-bar ankles driven through the motor-side MIT
law (apply_mit), 2-step command delay, 50 Hz policy, domain randomisation of friction and link masses.
The actor starts from a getup_bc.py policy (same observation and action, its normalisation kept fixed);
the critic also sees privileged state (base height and velocity, foot forces, contact flags).

Reward per 50 Hz step: torso height (steep near standing height) and uprightness, a bonus for touching the
floor with the feet only, a large bonus while standing on the feet only with the torso up (and more when the legs are near the walking controller's start stance), small penalties on action
rate, joint and base angular velocity and on non-foot floor contact once standing.

  tools/getup_rl.py --init bc3.npz --out rl/ --workers 22 --envs-per-worker 8 [--device cuda] [--iters 3000]

Writes rl/policy_<iter>.npz (GetUpPolicy format: getup_demo.py --student / walking_controller
recovery_policy), rl/latest.npz, rl/log.jsonl, rl/checkpoint.pt (resume with --resume).
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_name, '1')
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
HISTORY_DEFAULT = 10
PRIV = 12


# ---------------------------------------------------------------------------------------------- environment
def _setup_paths():
    sys.path.insert(0, str(HERE))


class GetUpEnv:
    """Several environments in one process (vectorised by a Python loop)."""

    def __init__(self, n, seed, history, episode_s, dr, pool=150, crouch_frac=0.0):
        _setup_paths()
        import mujoco  # noqa: F401
        from getup_demo import (ROOT, Command, GetUpPlanner, OfflineSim, RecoveryParams, apply_mit,
                                description_paths, load, lying_pose, make_controller, random_pose)
        from walking_controller.getup_policy import frame
        self.mj, self.Command, self.apply_mit = mujoco, Command, apply_mit
        self.lying_pose, self.random_pose, self.frame_fn = lying_pose, random_pose, frame
        paths, robot, limits = load(description_paths((ROOT / 'description' / 'humanoid_full_body_description').resolve()))
        self.q_nom = make_controller(robot, limits).q_nominal.copy()
        self.planner = GetUpPlanner(paths['robot_xml'], paths['mesh_dir'], paths['registry'], robot.joint_names,
                                    self.q_nom, limits.torque, RecoveryParams(threads=1, samples=1))
        self.sims = [OfflineSim(robot, limits, paths, delay_steps=2) for _ in range(n)]
        self.base = [(s.model.body_mass.copy(), s.model.body_inertia.copy()) for s in self.sims]
        self.rng = np.random.default_rng(seed)
        self.n, self.history, self.steps_max, self.dr = n, history, int(round(episode_s / 0.02)), dr
        self.crouch_frac = crouch_frac
        self.nj = len(self.q_nom)
        self.leg = self.planner.leg
        self.ankles = np.array([i for i, name in enumerate(self.planner.names) if 'ankle' in name])
        self.zero = np.zeros(self.nj)
        self.frames = [None] * n
        self.prev = np.zeros((n, self.nj))
        self.t = np.zeros(n, int)
        self.stand_run = np.zeros(n, int)
        self.ready_run = np.zeros(n, int)
        self.phase = np.zeros(n, int)      # 0 rise, 1 straight (GetUpPlanner phases)
        self.flat_sum = np.zeros(n)
        self.ep_ready = np.zeros(n, bool)
        self.ep_ret = np.zeros(n)
        self.ep_best_stand = np.zeros(n, int)
        self.contact_cache = [None] * n
        self.sole_sites = [[sim.model.site(name).id for name in ('left_foot', 'right_foot')] for sim in self.sims]
        m0 = self.sims[0].model
        self.robot_mask = np.zeros(m0.ngeom, bool)
        self.robot_mask[list(self.sims[0].robot_geoms)] = True
        self.foot_mask = np.zeros(m0.ngeom, bool)
        self.foot_mask[list(self.sims[0].foot_geoms)] = True
        # Start states (lying, settled) generated once: a reset restores one instead of settling for 1-1.5 s,
        # which would stall every other worker in the lock-stepped vector env.
        self.pool = []
        s0 = self.sims[0]
        for _ in range(pool):
            start = self._settle(0)
            self.pool.append((s0.data.qpos.copy(), s0.data.qvel.copy(), start))
        self.start = ['lying'] * n

    def _settle(self, i):
        s, m, d = self.sims[i], self.sims[i].model, self.sims[i].data
        self.mj.mj_resetData(m, d)
        d.eq_active[s.weld] = 1
        s.queue, s.command = [], None
        if self.rng.random() < self.crouch_frac:
            # 40 % of these start at the walking stance, already in the straight phase (as after the MPC's
            # rise): the only way to see the hand-over succeed early, since the stance itself (z ~0.48) is below
            # the rise check's 0.5 m.
            if self.rng.random() < 0.4:
                # A fixed PD at the stance does not balance (it tips within 0.3 s): start nearly at rest.
                self._crouch_pose(i, depth=1.0, noise=0.03, settle_steps=4)
                return 'nominal'
            self._crouch_pose(i)
            return 'crouch'
        start = self.rng.choice(['random', 'prone', 'supine'], p=[0.5, 0.25, 0.25])
        if start == 'random':
            self.random_pose(s, self.planner, self.rng)
        else:
            self.lying_pose(s, self.planner, start)
        return 'lying'

    def _crouch_pose(self, i, depth=None, noise=0.05, settle_steps=60):
        """Reverse curriculum: upright on the feet, legs between the walking stance and a deep squat, held
        for 0.3 s (it may already be tipping)."""
        s, m, d = self.sims[i], self.sims[i].model, self.sims[i].data
        q = self.q_nom.copy()
        depth = self.rng.uniform(1.0, 2.2) if depth is None else depth
        q[self.leg] *= depth
        q += self.rng.normal(0, noise, len(q))
        q = np.clip(q, self.planner.lo, self.planner.hi)
        yaw = self.rng.uniform(-math.pi, math.pi)
        d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        d.qpos[s.qpos] = q
        s.release()
        self.mj.mj_forward(m, d)
        low = min(d.geom_xpos[g, 2] - m.geom_rbound[g] for g in s.foot_geoms)
        d.qpos[2] += 0.005 - low
        self.mj.mj_forward(m, d)
        for _ in range(settle_steps):
            self.apply_mit(s, self.planner, self.Command(q, self.zero, self.planner.kp, self.planner.kd, self.zero))
            self.mj.mj_step(m, d)

    def _reset_one(self, i):
        s, m, d = self.sims[i], self.sims[i].model, self.sims[i].data
        self.mj.mj_resetData(m, d)
        d.eq_active[s.weld] = 0
        s.queue, s.command = [], None
        if self.dr:
            mass, inertia = self.base[i]
            k = self.rng.uniform(0.9, 1.1, m.nbody)
            k[0] = 1.0
            m.body_mass[:] = mass * k
            m.body_inertia[:] = inertia * k[:, None]
            m.geom('floor').friction[0] = self.rng.uniform(0.5, 1.2)
        qpos, qvel, self.start[i] = self.pool[self.rng.integers(len(self.pool))]
        d.qpos[:], d.qvel[:] = qpos, qvel
        d.qpos[0:2] = 0.0
        self.mj.mj_forward(m, d)
        self.contact_cache[i] = None
        self.phase[i] = 1 if self.start[i] == 'nominal' else 0
        self.frames[i] = [self._frame(i)] * self.history
        self.prev[i] = 0.0
        self.t[i] = 0
        self.stand_run[i] = 0
        self.ready_run[i] = 0
        self.flat_sum[i] = 0.0
        self.ep_ready[i] = False
        self.ep_ret[i] = 0.0
        self.ep_best_stand[i] = 0

    def _frame(self, i):
        d, s = self.sims[i].data, self.sims[i]
        return self.frame_fn(d.qvel[3:6], d.qpos[3:7], d.qpos[s.qpos], d.qvel[s.qvel], self.q_nom)

    def _obs(self, i):
        return np.concatenate([np.ravel(self.frames[i]), self.prev[i]])

    def _contacts(self, i):
        if self.contact_cache[i] is not None:
            return self.contact_cache[i]
        s, d = self.sims[i], self.sims[i].data
        g = d.contact.geom[:d.ncon]
        floor = (g == s.floor).any(1)
        other_geom = np.where(g[:, 0] == s.floor, g[:, 1], g[:, 0])[floor]
        robot = self.robot_mask[other_geom]
        foot = self.foot_mask[other_geom] & robot
        result = (bool(foot.any()), bool((robot & ~foot).any()))
        self.contact_cache[i] = result
        return result

    def _priv(self, i, feet, other, up):
        d, s = self.sims[i].data, self.sims[i]
        f = s.foot_forces()
        return np.array([d.qpos[2], *d.qvel[:3], up, f[0] / 100, f[1] / 100, float(feet), float(other),
                         self.t[i] / self.steps_max, self.stand_run[i] / 50, float(np.abs(d.qvel).max() > 50)])

    def reset(self):
        for i in range(self.n):
            self._reset_one(i)
        return self._pack()

    def _pack(self, rewards=None, dones=None, infos=None):
        obs = np.stack([self._obs(i) for i in range(self.n)]).astype(np.float32)
        priv = []
        for i in range(self.n):
            d = self.sims[i].data
            w, x, y = d.qpos[3:6]
            up = 1 - 2 * (x * x + y * y)
            feet, other = self._contacts(i)
            priv.append(self._priv(i, feet, other, up))
        return obs, np.asarray(priv, np.float32), rewards, dones, infos

    def step(self, actions):
        rewards, dones, infos = np.zeros(self.n, np.float32), np.zeros(self.n, bool), []
        for i in range(self.n):
            s, m, d = self.sims[i], self.sims[i].model, self.sims[i].data
            a = np.clip(actions[i], -3.5, 3.5)
            target = np.clip(self.q_nom + a, self.planner.lo, self.planner.hi)
            command = self.Command(target, self.zero, self.planner.kp, self.planner.kd, self.zero)
            for _ in range(4):
                s.queue.append(command)
                if len(s.queue) > s.delay_steps:
                    s.command = s.queue.pop(0)
                self.apply_mit(s, self.planner, s.command)
                self.mj.mj_step(m, d)
            self.contact_cache[i] = None
            bad = not np.isfinite(d.qvel).all() or np.abs(d.qvel).max() > 200
            z = float(d.qpos[2])
            w, x, y, _ = d.qpos[3:7]
            up = float(1 - 2 * (x * x + y * y))
            feet, other = self._contacts(i)
            standing = feet and not other and z > 0.5 and up > 0.85
            self.stand_run[i] = self.stand_run[i] + 1 if standing else 0
            self.ep_best_stand[i] = max(self.ep_best_stand[i], self.stand_run[i])
            leg_err = float(np.mean((d.qpos[s.qpos][self.leg] - self.q_nom[self.leg]) ** 2))
            # Sole tilts: standing on a toe or the side of a foot also counts as feet-only contact, so the
            # standing bonus is scaled by how flat both soles are (the hand-over needs < 10 deg).
            tilts = [math.degrees(math.acos(min(1.0, max(-1.0, float(d.site_xmat[k][8]))))) for k in self.sole_sites[i]]
            # Linear in the summed tilt so that every degree towards flat pays (a Gaussian gave no gradient
            # from the 45 / 110 deg tiptoe stance), times the ankles' closeness to the walking stance.
            ank = d.qpos[s.qpos][self.ankles] - self.q_nom[self.ankles]
            flat_factor = max(0.0, 1.0 - (tilts[0] + tilts[1]) / 120.0) * math.exp(-float(np.sum(ank ** 2)) / 0.2)
            self.flat_sum[i] += flat_factor * standing
            # The hand-over logic of GetUpPlanner._transition, as the ROS node runs it: rise until standing
            # (feet only, z > 0.5, torso up) for rise_hold 0.5 s, then straight: back to the walking stance with
            # flat, loaded soles and a still base for 0.3 s -> ready (success, episode ends). Losing the feet or
            # dropping below 0.4 m in straight returns to rise.
            ready_like, straight_r = False, 0.0
            if self.phase[i] == 0 and self.stand_run[i] >= 25:
                self.phase[i] = 1
            elif self.phase[i] == 1 and not (feet and not other and z > 0.4):
                self.phase[i] = 0
            if self.phase[i] == 1:
                forces = s.foot_forces()
                speed = float(np.linalg.norm(d.qvel[:3]))
                err_max = float(np.abs(d.qpos[s.qpos][self.leg] - self.q_nom[self.leg]).max())
                ready_like = (up > math.cos(0.25) and min(forces) > 40.0 and speed < 0.15
                              and err_max < 0.25 and max(tilts) < 10.0)
                # Wide scales: from the straight-legged, splayed stance the policy first finds (leg error ~0.9 rad,
                # soles ~60 deg) every step towards the walking stance must still pay.
                straight_r = (2.0 + 2.0 * math.exp(-leg_err / 0.5) + 1.0 * math.exp(-leg_err / 0.05)
                              + 2.0 * max(0.0, 1.0 - err_max / 1.5) + 2.0 * max(0.0, 1.0 - max(tilts) / 120.0)
                              + 1.0 * max(0.0, 1.0 - max(tilts) / 20.0) + 1.5 * math.exp(-speed ** 2 / 0.5)
                              + 0.5 * min(1.0, min(forces) / 40.0) + 0.5 * max(0.0, (up - 0.85) / 0.15)
                              + 2.0 * ready_like)
            self.ready_run[i] = self.ready_run[i] + 1 if ready_like else 0
            success = self.ready_run[i] >= 15
            if success:
                self.ep_ready[i] = True
            ang = float(np.sum(d.qvel[3:6] ** 2))
            h = min(max(z, 0.0), 0.62) / 0.62
            # Kneeling upright (h ~0.7) must pay clearly less than a crouch on the feet only, else the
            # policy settles on its knees: steep height term and a feet-only bonus.
            r = (0.3 * h + 1.0 * h ** 4 + 0.3 * max(up, 0.0) + 0.5 * (feet and not other) * (0.5 + 0.5 * flat_factor)
                 + (2.0 + 1.0 * math.exp(-leg_err / 0.05)
                    + 1.5 * math.exp(-float(np.sum(d.qvel[:3] ** 2)) / 0.02)) * standing * (0.3 + 0.7 * flat_factor)
                 * (self.phase[i] == 0)
                 + straight_r + 30.0 * success
                 - 0.02 * float(np.sum((a - self.prev[i]) ** 2))
                 - 0.0005 * float(np.sum(d.qvel[s.qvel] ** 2)) - 0.01 * ang
                 - 0.5 * (other and self.ep_best_stand[i] > 25))
            self.prev[i] = a
            self.frames[i] = self.frames[i][1:] + [self._frame(i)]
            self.t[i] += 1
            self.ep_ret[i] += r
            done = bad or success or self.t[i] >= self.steps_max
            rewards[i] = -5.0 if bad else r
            if done:
                infos.append(dict(start=self.start[i], ret=float(self.ep_ret[i]), best_stand_s=self.ep_best_stand[i] * 0.02,
                                  stood=bool(self.ep_best_stand[i] >= 50), ready=bool(self.ep_ready[i]),
                                  flat_stand_s=float(self.flat_sum[i]) * 0.02, bad=bool(bad),
                                  timeout=bool(not bad and not success)))
                dones[i] = True
                self._reset_one(i)
        return self._pack(rewards, dones, infos)


def worker_main(conn, n, seed, history, episode_s, dr, crouch_frac=0.0):
    os.environ.setdefault('OMP_NUM_THREADS', '1')
    env = GetUpEnv(n, seed, history, episode_s, dr, crouch_frac=crouch_frac)
    while True:
        cmd, data = conn.recv()
        if cmd == 'reset':
            conn.send(env.reset())
        elif cmd == 'step':
            conn.send(env.step(data))
        elif cmd == 'close':
            return


class StdioConn:
    """Pipe-like connection over a stream pair (length-prefixed pickles)."""

    def __init__(self, reader, writer):
        self.r, self.w = reader, writer

    def send(self, obj):
        import pickle
        data = pickle.dumps(obj, protocol=4)
        self.w.write(len(data).to_bytes(8, 'little') + data)
        self.w.flush()

    def recv(self):
        import pickle
        head = self.r.read(8)
        if len(head) < 8:
            raise EOFError('remote worker closed')
        size = int.from_bytes(head, 'little')
        return pickle.loads(self.r.read(size))


def stdio_worker(n, seed, history, episode_s, dr):
    """Worker driven over stdin/stdout (ssh); everything else printed goes to stderr."""
    out = os.fdopen(os.dup(1), 'wb')
    os.dup2(2, 1)
    worker_main(StdioConn(sys.stdin.buffer, out), n, seed, history, episode_s, dr)


class VecEnv:
    def __init__(self, workers, per_worker, seed, history, episode_s, dr, remotes=(), crouch_frac=0.0):
        """remotes: (ssh host, workers, envs per worker, python, source dir) tuples."""
        import shlex
        import subprocess
        ctx = mp.get_context('spawn')
        self.conns, self.procs, self.counts = [], [], []
        for k in range(workers):
            a, b = ctx.Pipe()
            p = ctx.Process(target=worker_main, args=(b, per_worker, seed * 1000 + k, history, episode_s, dr, crouch_frac),
                            daemon=True)
            p.start()
            self.conns.append(a)
            self.procs.append(p)
            self.counts.append(per_worker)
        k = workers
        for host, n_workers, per, python, src in remotes:
            for _ in range(n_workers):
                cmd = (f'cd {shlex.quote(src)} && OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 '
                       f'PYTHONPATH=control/walking_controller:simulation/humanoid_sim '
                       f'{shlex.quote(python)} control/walking_controller/tools/getup_rl.py '
                       f'--stdio-worker {per} {seed * 1000 + k} {history} {episode_s} {int(dr)}')
                p = subprocess.Popen(['ssh', '-o', 'BatchMode=yes', host, cmd], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE)
                self.conns.append(StdioConn(p.stdout, p.stdin))
                self.procs.append(p)
                self.counts.append(per)
                k += 1
        self.offsets = np.concatenate([[0], np.cumsum(self.counts)])

    def _gather(self):
        out = [c.recv() for c in self.conns]
        obs = np.concatenate([o[0] for o in out])
        priv = np.concatenate([o[1] for o in out])
        if out[0][2] is None:
            return obs, priv, None, None, []
        return (obs, priv, np.concatenate([o[2] for o in out]), np.concatenate([o[3] for o in out]),
                [i for o in out for i in o[4]])

    def reset(self):
        for c in self.conns:
            c.send(('reset', None))
        return self._gather()

    def step(self, actions):
        for k, c in enumerate(self.conns):
            c.send(('step', actions[self.offsets[k]:self.offsets[k + 1]]))
        return self._gather()

    def close(self):
        for c in self.conns:
            c.send(('close', None))


# ---------------------------------------------------------------------------------------------- networks / PPO
def build(torch, init_npz, priv_dim, device):
    nn = torch.nn
    z = np.load(init_npz)
    n = int(z['layers'])
    layers = []
    for i in range(n):
        w = z[f'w{i}']
        lin = nn.Linear(w.shape[1], w.shape[0])
        lin.weight.data[:] = torch.as_tensor(w)
        lin.bias.data[:] = torch.as_tensor(z[f'b{i}'])
        layers.append(lin)
        if i < n - 1:
            layers.append(nn.ELU())
    actor = nn.Sequential(*layers).to(device)
    obs_dim = z['w0'].shape[1]
    critic = nn.Sequential(nn.Linear(obs_dim + priv_dim, 512), nn.ELU(), nn.Linear(512, 256), nn.ELU(),
                           nn.Linear(256, 128), nn.ELU(), nn.Linear(128, 1)).to(device)
    meta = {k: z[k] for k in ('obs_mean', 'obs_std', 'q_nominal', 'scale', 'history', 'joint_names')}
    meta['time_input'] = z['time_input'] if 'time_input' in z.files else False
    return actor, critic, meta


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


def save_policy(path, actor, meta):
    linear = [m for m in actor if hasattr(m, 'weight')]
    weights = {f'w{i}': m.weight.detach().cpu().numpy() for i, m in enumerate(linear)}
    weights.update({f'b{i}': m.bias.detach().cpu().numpy() for i, m in enumerate(linear)})
    tmp = path.with_suffix('.tmp.npz')
    np.savez(tmp, layers=len(linear), **meta, **weights)
    tmp.replace(path)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--init', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--envs-per-worker', type=int, default=4)
    p.add_argument('--steps', type=int, default=48, help='rollout length per env and iteration')
    p.add_argument('--iters', type=int, default=3000)
    p.add_argument('--episode-s', type=float, default=12.0)
    p.add_argument('--critic-warmup', type=int, default=30, help='iterations training only the critic')
    p.add_argument('--lr-actor', type=float, default=5e-5)
    p.add_argument('--lr-critic', type=float, default=3e-4)
    p.add_argument('--init-std', type=float, default=0.15)
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
    p.add_argument('--crouch-frac', type=float, default=0.0,
                   help='share of episodes starting upright on the feet in a squat (reverse curriculum)')
    p.add_argument('--remote', action='append', default=[],
                   help='HOST:WORKERS:ENVS_PER_WORKER:PYTHON:SRC_DIR (ssh workers; MuJoCo only, no torch)')
    if len(sys.argv) > 1 and sys.argv[1] == '--stdio-worker':
        n, seed, history, episode_s, dr = sys.argv[2:7]
        stdio_worker(int(n), int(seed), int(history), float(episode_s), bool(int(dr)))
        return
    a = p.parse_args()
    import torch
    torch.set_num_threads(2 if a.device.startswith("cuda") else 4)
    torch.manual_seed(a.seed)
    dev = torch.device(a.device)
    a.out.mkdir(parents=True, exist_ok=True)
    z0 = np.load(a.init)
    history = int(z0['history'])
    actor, critic, meta = build(torch, a.init, PRIV, dev)
    log_std = torch.nn.Parameter(torch.full((len(meta['q_nominal']),), math.log(a.init_std), device=dev))
    opt_a = torch.optim.Adam(list(actor.parameters()) + [log_std], lr=a.lr_actor)
    opt_c = torch.optim.Adam(critic.parameters(), lr=a.lr_critic)
    o_mean = torch.as_tensor(meta['obs_mean'], dtype=torch.float32, device=dev)
    o_std = torch.as_tensor(meta['obs_std'], dtype=torch.float32, device=dev)
    scale = float(meta['scale'])
    norm_c = None
    start_iter = 0
    ckpt = a.out / 'checkpoint.pt'
    if a.resume and ckpt.exists():
        c = torch.load(ckpt, map_location=dev, weights_only=False)
        actor.load_state_dict(c['actor'])
        critic.load_state_dict(c['critic'])
        log_std.data[:] = c['log_std']
        opt_a.load_state_dict(c['opt_a'])
        opt_c.load_state_dict(c['opt_c'])
        norm_c, start_iter = c['norm_c'], c['iter'] + 1

    remotes = []
    for spec in a.remote:
        host, n, per, python, src = spec.split(':', 4)
        remotes.append((host, int(n), int(per), python, src))
    env = VecEnv(a.workers, a.envs_per_worker, a.seed, history, a.episode_s, not a.no_dr, remotes, a.crouch_frac)
    obs, priv, _, _, _ = env.reset()
    E = len(obs)
    if norm_c is None:
        norm_c = RunningNorm(obs.shape[1] + priv.shape[1])
    tt = lambda x: torch.as_tensor(x, dtype=torch.float32, device=dev)  # noqa: E731
    log = (a.out / 'log.jsonl').open('a')
    print(f'{E} envs, obs {obs.shape[1]}, priv {priv.shape[1]}, device {dev}', flush=True)
    episodes = []
    for it in range(start_iter, a.iters):
        t0 = time.monotonic()
        t_env = 0.0
        B_obs, B_cin, B_act, B_logp, B_rew, B_done, B_val = [], [], [], [], [], [], []
        for _ in range(a.steps):
            cin = np.concatenate([obs, priv], 1)
            norm_c.update(cin)
            with torch.no_grad():
                mu = actor((tt(obs) - o_mean) / o_std)
                std = log_std.exp()
                dist = torch.distributions.Normal(mu, std)
                act = dist.sample()
                logp = dist.log_prob(act).sum(-1)
                val = critic(tt(norm_c(cin))).squeeze(-1)
            te = time.monotonic()
            obs2, priv2, rew, done, infos = env.step((act * scale).cpu().numpy())
            t_env += time.monotonic() - te
            B_obs.append(obs); B_cin.append(norm_c(cin)); B_act.append(act); B_logp.append(logp)
            B_rew.append(rew); B_done.append(done); B_val.append(val)
            episodes += infos
            obs, priv = obs2, priv2
        t_roll = time.monotonic() - t0
        with torch.no_grad():
            last_val = critic(tt(norm_c(np.concatenate([obs, priv], 1)))).squeeze(-1)
        rew = tt(np.stack(B_rew)); done = tt(np.stack(B_done).astype(np.float32)); val = torch.stack(B_val)
        adv = torch.zeros_like(rew)
        gae = torch.zeros(E, device=dev)
        for k in reversed(range(a.steps)):
            nxt = last_val if k == a.steps - 1 else val[k + 1]
            delta = rew[k] + a.gamma * nxt * (1 - done[k]) - val[k]
            gae = delta + a.gamma * a.lam * (1 - done[k]) * gae
            adv[k] = gae
        ret = adv + val
        f = lambda x: x.reshape(-1, *x.shape[2:])  # noqa: E731
        X = f(tt(np.stack(B_obs))); C = f(tt(np.stack(B_cin))); A = f(torch.stack(B_act))
        LP = f(torch.stack(B_logp)); R = f(ret); ADV = f(adv)
        ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)
        N = len(X); mb = N // a.minibatches
        train_actor = it >= a.critic_warmup
        kl_hit, kls, closses, alosses = False, [], [], []
        for _ in range(a.epochs):
            perm = torch.randperm(N, device=dev)
            for j in range(a.minibatches):
                idx = perm[j * mb:(j + 1) * mb]
                v = critic(C[idx]).squeeze(-1)
                closs = ((v - R[idx]) ** 2).mean()
                opt_c.zero_grad(); closs.backward()
                torch.nn.utils.clip_grad_norm_(critic.parameters(), 1.0); opt_c.step()
                closses.append(closs.item())
                if train_actor and not kl_hit:
                    mu = actor((X[idx] - o_mean) / o_std)
                    dist = torch.distributions.Normal(mu, log_std.exp())
                    lp = dist.log_prob(A[idx]).sum(-1)
                    ratio = (lp - LP[idx]).exp()
                    aloss = -torch.min(ratio * ADV[idx], ratio.clamp(1 - a.clip, 1 + a.clip) * ADV[idx]).mean()
                    opt_a.zero_grad(); aloss.backward()
                    torch.nn.utils.clip_grad_norm_(list(actor.parameters()) + [log_std], 1.0); opt_a.step()
                    with torch.no_grad():
                        log_std.data.clamp_(math.log(0.03), math.log(0.5))
                    kl = (LP[idx] - lp).mean().item()
                    kls.append(kl); alosses.append(aloss.item())
                    if kl > a.target_kl:
                        kl_hit = True
        recent = episodes[-200:]
        lying = [e for e in episodes[-600:] if e.get('start') == 'lying'][-200:]
        crouch = [e for e in episodes[-600:] if e.get('start') == 'crouch'][-200:]
        nominal = [e for e in episodes[-900:] if e.get('start') == 'nominal'][-200:]
        stats = dict(iter=it, sps=round(N / (time.monotonic() - t0)), t_env=round(t_env, 2), t_roll=round(t_roll, 2),
                     t_all=round(time.monotonic() - t0, 2), reward=float(rew.mean()),
                     episodes=len(episodes), stood=float(np.mean([e['stood'] for e in recent])) if recent else 0,
                     best_stand_s=float(np.mean([e['best_stand_s'] for e in recent])) if recent else 0,
                     bad=float(np.mean([e['bad'] for e in recent])) if recent else 0,
                     ret=float(np.mean([e['ret'] for e in recent])) if recent else 0,
                     stood_lying=float(np.mean([e['stood'] for e in lying])) if lying else 0,
                     stood_crouch=float(np.mean([e['stood'] for e in crouch])) if crouch else 0,
                     ready_lying=float(np.mean([e.get('ready', False) for e in lying])) if lying else 0,
                     ready_nominal=float(np.mean([e.get('ready', False) for e in nominal])) if nominal else 0,
                     flat_stand_s=float(np.mean([e.get('flat_stand_s', 0) for e in recent])) if recent else 0,
                     closs=float(np.mean(closses)), kl=float(np.mean(kls)) if kls else 0.0,
                     std=float(log_std.detach().exp().mean()), actor=train_actor)
        log.write(json.dumps(stats) + '\n'); log.flush()
        if it % 5 == 0:
            print(json.dumps(stats), flush=True)
        if it % a.save_every == 0 or it == a.iters - 1:
            save_policy(a.out / f'policy_{it:05d}.npz', actor, meta)
            save_policy(a.out / 'latest.npz', actor, meta)
            torch.save(dict(actor=actor.state_dict(), critic=critic.state_dict(), log_std=log_std.data,
                            opt_a=opt_a.state_dict(), opt_c=opt_c.state_dict(), norm_c=norm_c, iter=it), ckpt)
    env.close()


if __name__ == '__main__':
    main()
