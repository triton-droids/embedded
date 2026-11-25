"""Shared full-body RL environment for the get-up, balance and walk tasks (tools/rl_tasks/).

Plant (same as getup_demo.py / the ROS simulator): OfflineSim, four-bar ankles driven on the motor side through
apply_mit, motor torque-speed limits (humanoid_sim.motor_model), 2-step command delay, 50 Hz policy, domain
randomisation of friction and link masses. Actions are joint targets q_nominal + a, tracked with the get-up
planner's MIT gains.

A task (rl_tasks/<name>.py, class Task) defines: history (frames per observation), command_dim (0, or a
velocity command appended to the observation), episode_s, make_start(env, i) -> label (puts sims[i] into a start
state; called while building the start pool), on_reset(env, i), reward(env, i, meas, action) ->
(reward, done, success, info) and optionally priv(env, i) -> extra privileged values (fixed length priv_dim).

Observation (walking_controller.getup_policy layout): history frames of [base angular velocity (body),
gravity (body), joint_pos - q_nominal, joint_vel], the previous action, then the command (if command_dim).
"""
from __future__ import annotations

import importlib
import math
import multiprocessing as mp
import os
from pathlib import Path
import sys

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_name, '1')
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
BASE_PRIV = 14


def load_task(name, kwargs=None):
    sys.path.insert(0, str(HERE))
    return importlib.import_module(f'rl_tasks.{name}').Task(**(kwargs or {}))


class TaskEnv:
    """n environments in one process (a Python loop), all running the same task."""

    def __init__(self, n, seed, task, task_kwargs=None, dr=True, pool=150):
        sys.path.insert(0, str(HERE))
        import mujoco
        from getup_demo import (ROOT, Command, GetUpPlanner, OfflineSim, RecoveryParams, apply_mit,
                                description_paths, load, lying_pose, make_controller, random_pose)
        from humanoid_sim.motor_model import NO_LOAD_SPEED_RAD_S
        from walking_controller.getup_policy import frame
        import yaml
        self.mj, self.Command, self.apply_mit = mujoco, Command, apply_mit
        self.lying_pose, self.random_pose, self.frame_fn = lying_pose, random_pose, frame
        paths, robot, limits = load(description_paths((ROOT / 'description' / 'humanoid_full_body_description').resolve()))
        self.paths, self.robot, self.limits = paths, robot, limits
        self.q_nom = make_controller(robot, limits).q_nominal.copy()
        self.planner = GetUpPlanner(paths['robot_xml'], paths['mesh_dir'], paths['registry'], robot.joint_names,
                                    self.q_nom, limits.torque, RecoveryParams(threads=1, samples=1))
        self.sims = [OfflineSim(robot, limits, paths, delay_steps=2) for _ in range(n)]
        self.base = [(s.model.body_mass.copy(), s.model.body_inertia.copy()) for s in self.sims]
        self.rng = np.random.default_rng(seed)
        self.task = load_task(task, task_kwargs) if isinstance(task, str) else task
        self.n, self.dr = n, dr
        self.history = int(self.task.history)
        self.command_dim = int(getattr(self.task, 'command_dim', 0))
        self.priv_dim = BASE_PRIV + int(getattr(self.task, 'priv_dim', 0))
        self.steps_max = int(round(self.task.episode_s / 0.02))
        self.names = list(robot.joint_names)
        self.nj = len(self.names)
        self.leg = self.planner.leg
        self.ankles = np.array([i for i, name in enumerate(self.names) if 'ankle' in name])
        reg = yaml.safe_load(Path(paths['registry']).read_text())
        reg = reg.get('motor_control_node', {}).get('ros__parameters', reg)['motors']
        self.no_load = np.array([NO_LOAD_SPEED_RAD_S.get(str(reg[n].get('model')), np.inf) for n in self.names])
        self.no_load[self.ankles] = np.inf  # ankle motor speed differs from the joint's (four-bar); not penalised
        self.zero = np.zeros(self.nj)
        self.frames = [None] * n
        self.prev = np.zeros((n, self.nj))
        self.command = np.zeros((n, max(1, self.command_dim)))
        self.t = np.zeros(n, int)
        self.ep_ret = np.zeros(n)
        self.label = [''] * n
        self.state = [dict() for _ in range(n)]   # per-env task state, cleared on reset
        self.contact_cache = [None] * n
        self.sole_sites = [[sim.model.site(name).id for name in ('left_foot', 'right_foot')] for sim in self.sims]
        m0, s0 = self.sims[0].model, self.sims[0]
        self.robot_mask = np.zeros(m0.ngeom, bool)
        self.robot_mask[list(s0.robot_geoms)] = True
        self.foot_id = np.full(m0.ngeom, -1)
        for k, g in enumerate(s0.foot_geoms):
            self.foot_id[g] = k
        self.act = np.array(s0.act)
        self.qv = np.array(s0.qvel)
        # Start states built once per worker (a reset restores one: no settling that would stall the lock-step).
        self.pool = []
        for _ in range(pool):
            m, d = s0.model, s0.data
            self.mj.mj_resetData(m, d)
            d.eq_active[s0.weld] = 1
            s0.queue, s0.command = [], None
            label = self.task.make_start(self, 0)
            self.pool.append((d.qpos.copy(), d.qvel.copy(), label))

    # ------------------------------------------------------------------ start-state helpers for tasks
    def lying(self, i, start=None):
        s = self.sims[i]
        start = start or self.rng.choice(['random', 'prone', 'supine'], p=[0.5, 0.25, 0.25])
        if start == 'random':
            self.random_pose(s, self.planner, self.rng)
        else:
            self.lying_pose(s, self.planner, start)
        return start

    def stance(self, i, depth=1.0, noise=0.03, settle_steps=4, yaw=None):
        """Upright on the feet, legs at depth x the walking stance (1 = the stance itself, 2 = a deep squat)."""
        s, m, d = self.sims[i], self.sims[i].model, self.sims[i].data
        q = self.q_nom.copy()
        q[self.leg] *= depth
        q = np.clip(q + self.rng.normal(0, noise, len(q)), self.planner.lo, self.planner.hi)
        yaw = self.rng.uniform(-math.pi, math.pi) if yaw is None else yaw
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

    def from_states(self, i, states, noise=0.02):
        """Restore one of saved (qpos, qvel) states (e.g. the get-up policy's end states), with joint noise."""
        s, m, d = self.sims[i], self.sims[i].model, self.sims[i].data
        k = self.rng.integers(len(states['qpos']))
        d.qpos[:], d.qvel[:] = states['qpos'][k], states['qvel'][k]
        d.qpos[s.qpos] += self.rng.normal(0, noise, self.nj)
        d.qpos[0:2] = 0.0
        s.release()
        self.mj.mj_forward(m, d)

    # ------------------------------------------------------------------ observation / measurements
    def _frame(self, i):
        d, s = self.sims[i].data, self.sims[i]
        return self.frame_fn(d.qvel[3:6], d.qpos[3:7], d.qpos[s.qpos], d.qvel[s.qvel], self.q_nom)

    def obs(self, i):
        parts = [np.ravel(self.frames[i]), self.prev[i]]
        if self.command_dim:
            parts.append(self.command[i, :self.command_dim])
        return np.concatenate(parts)

    def contacts(self, i):
        """(left foot, right foot, any other robot geom) touching the floor."""
        if self.contact_cache[i] is None:
            s, d = self.sims[i], self.sims[i].data
            g = d.contact.geom[:d.ncon]
            floor = (g == s.floor).any(1)
            other_geom = np.where(g[:, 0] == s.floor, g[:, 1], g[:, 0])[floor]
            robot = self.robot_mask[other_geom]
            fid = self.foot_id[other_geom]
            self.contact_cache[i] = (bool(((fid == 0) & robot).any()), bool(((fid == 1) & robot).any()),
                                     bool((robot & (fid < 0)).any()))
        return self.contact_cache[i]

    def measure(self, i):
        s, d = self.sims[i], self.sims[i].data
        left, right, other = self.contacts(i)
        w, x, y, z = d.qpos[3:7]
        up = float(1 - 2 * (x * x + y * y))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        c, sn = math.cos(yaw), math.sin(yaw)
        v = d.qvel[:3]
        ang_world = np.zeros(3)
        self.mj.mju_rotVecQuat(ang_world, d.qvel[3:6], d.qpos[3:7])
        q, dq = d.qpos[s.qpos], d.qvel[s.qvel]
        err = q - self.q_nom
        tilts = [math.degrees(math.acos(min(1.0, max(-1.0, float(d.site_xmat[k][8]))))) for k in self.sole_sites[i]]
        force = d.actuator_force[self.act]
        return dict(
            z=float(d.qpos[2]), up=up, yaw=yaw, left=left, right=right, other=other,
            feet=left or right, feet_only=(left or right) and not other, airborne=not (left or right or other),
            vel_world=v.copy(), vx=float(c * v[0] + sn * v[1]), vy=float(-sn * v[0] + c * v[1]), vz=float(v[2]),
            speed=float(np.linalg.norm(v)), yaw_rate=float(ang_world[2]), ang_body=d.qvel[3:6].copy(),
            q=q.copy(), dq=dq.copy(), leg_err=float(np.mean(err[self.leg] ** 2)),
            err_max=float(np.abs(err[self.leg]).max()), tilts=tilts, forces=s.foot_forces(),
            speed_ratio=float(np.max(np.abs(dq) / self.no_load)),
            power=float(np.sum(np.abs(force * dq))), torque=force.copy())

    def priv(self, i, meas):
        f = meas['forces']
        base = [meas['z'], *meas['vel_world'], meas['up'], f[0] / 100, f[1] / 100, float(meas['left']),
                float(meas['right']), float(meas['other']), self.t[i] / self.steps_max, meas['speed_ratio'],
                meas['power'] / 1000, meas['yaw_rate']]
        extra = list(self.task.priv(self, i)) if hasattr(self.task, 'priv') else []
        return np.array(base + extra, np.float32)

    # ------------------------------------------------------------------ reset / step
    def reset_one(self, i):
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
        qpos, qvel, self.label[i] = self.pool[self.rng.integers(len(self.pool))]
        d.qpos[:], d.qvel[:] = qpos, qvel
        d.qpos[0:2] = 0.0
        self.mj.mj_forward(m, d)
        self.contact_cache[i] = None
        self.frames[i] = [self._frame(i)] * self.history
        self.prev[i] = 0.0
        self.command[i] = 0.0
        self.t[i] = 0
        self.ep_ret[i] = 0.0
        self.state[i] = dict()
        self.task.on_reset(self, i)

    def reset(self):
        for i in range(self.n):
            self.reset_one(i)
        return self._pack()

    def _pack(self, rewards=None, dones=None, infos=None):
        obs = np.stack([self.obs(i) for i in range(self.n)]).astype(np.float32)
        priv = np.stack([self.priv(i, self.measure(i)) for i in range(self.n)])
        return obs, priv, rewards, dones, infos

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
            meas = None if bad else self.measure(i)
            if bad:
                r, done, success, info = -5.0, True, False, {}
            else:
                r, done, success, info = self.task.reward(self, i, meas, a)
                r -= 0.02 * float(np.sum((a - self.prev[i]) ** 2))
            self.prev[i] = a
            self.frames[i] = self.frames[i][1:] + [self._frame(i)]
            self.t[i] += 1
            self.ep_ret[i] += r
            done = bool(done or bad or self.t[i] >= self.steps_max)
            rewards[i] = r
            if done:
                info = dict(start=self.label[i], ret=float(self.ep_ret[i]), success=bool(success),
                            bad=bool(bad), steps=int(self.t[i]), **info)
                if success:  # the next task's start states (rl_collect_states.py)
                    info['end_qpos'], info['end_qvel'] = d.qpos.copy(), d.qvel.copy()
                infos.append(info)
                dones[i] = True
                self.reset_one(i)
        return self._pack(rewards, dones, infos)


# ---------------------------------------------------------------------------------------------- workers
def worker_main(conn, n, seed, task, task_kwargs, dr):
    env = TaskEnv(n, seed, task, task_kwargs, dr)
    while True:
        cmd, data = conn.recv()
        if cmd == 'reset':
            conn.send(env.reset())
        elif cmd == 'step':
            conn.send(env.step(data))
        elif cmd == 'close':
            return


class VecEnv:
    def __init__(self, workers, per_worker, seed, task, task_kwargs=None, dr=True):
        ctx = mp.get_context('spawn')
        self.conns, self.procs, self.counts = [], [], []
        for k in range(workers):
            a, b = ctx.Pipe()
            p = ctx.Process(target=worker_main, args=(b, per_worker, seed * 1000 + k, task, task_kwargs, dr),
                            daemon=True)
            p.start()
            self.conns.append(a)
            self.procs.append(p)
            self.counts.append(per_worker)
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
