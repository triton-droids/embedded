"""Fall recovery for the full body: predictive-sampling MPC that gets the robot up and stands it straight.

Phases (GetUpPlanner.phase):
  rise      from lying (any pose) until the robot stands on its feet only, z > 0.5 m and upright > 0.85
            for rise_hold_s; optional rise_max_speed / rise_max_tilt_rad limits require a calmer hand-over
  straight  torso upright, CoM between the feet, legs at the walking controller's start stance, soles flat
            and loaded, feet at the start stance (side by side, not staggered); ends with ready() once
            all of that holds for straight_hold_s, or a relaxed version of it after straight_timeout_s
The walking controller then takes over with GaitController.start_from_stand.

Each plan: `samples` rollouts of `horizon` s on the planner's own MuJoCo model (the simulator's MJCF built
with humanoid_sim's model builder), joint targets as `knots` knots linearly interpolated, the lowest cost
kept and shifted to warm-start the next plan. Commands are MIT position targets with the gains below
(velocity and feedforward 0). Four-bar ankles are modelled in the rollouts as humanoid_sim drives them
(only the position target converted to the motor angle, kp/kd on the motor side); the command itself is
the joint-space ankle target, which the motor layer converts. About 0.8 s of CPU (8 threads) per plan on
the development PC: far slower than real time, run the simulation with sim_time:=true and a
real_time_factor of about 0.03.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading

import mujoco
from mujoco import rollout as mjr
import numpy as np
import yaml

COST_SENSORS = ('cost_com', 'cost_lf', 'cost_rf', 'cost_lfz', 'cost_rfz')
RISE, STRAIGHT = 'rise', 'straight'
# MPC joint gains (Nm/rad) by joint-name part; kd = 0.24 sqrt(kp).
KP = {'hip1': 150, 'hip2': 150, 'thigh': 100, 'knee': 150, 'ankle': 40, 'shoulder_pitch': 80,
      'shoulder_roll': 80, 'shoulder_yaw': 20, 'elbow': 20}
SOLES = ('left_foot', 'right_foot')


def add_cost_sensors(spec, _options=None):
    """humanoid_sim model_builder scene_builder hook: whole-body CoM, sole sites and their normals."""
    spec.add_sensor(name='cost_com', type=mujoco.mjtSensor.mjSENS_SUBTREECOM,
                    objtype=mujoco.mjtObj.mjOBJ_BODY, objname='floating_base')
    for side, short in (('left_foot', 'lf'), ('right_foot', 'rf')):
        spec.add_sensor(name=f'cost_{short}', type=mujoco.mjtSensor.mjSENS_FRAMEPOS,
                        objtype=mujoco.mjtObj.mjOBJ_SITE, objname=side)
        spec.add_sensor(name=f'cost_{short}z', type=mujoco.mjtSensor.mjSENS_FRAMEZAXIS,
                        objtype=mujoco.mjtObj.mjOBJ_SITE, objname=side)


@dataclass
class RecoveryParams:
    samples: int = 96
    horizon_s: float = 1.0
    knots: int = 5
    plan_dt: float = 0.04          # warm-start shift per plan
    sigma_rise: float = 0.25
    sigma_straight: float = 0.12
    threads: int = 8
    seed: int = 0
    rise_hold_s: float = 0.5
    # Calm hand-over requirements are off by default: balance handles braking after rising.
    # Set finite speed / tighter tilt limits to require a calmer rise -> straight transition.
    rise_max_speed: float = math.inf
    rise_max_tilt_rad: float = math.acos(0.85)
    straight_hold_s: float = 0.3
    # straight -> rise ("lost the stand"): a non-foot body on the floor, or base below / tilt beyond these
    # (the balance policy's training fall check, tools/rl_tasks/balance.py fall_z / fall_up).
    straight_fall_z: float = 0.38
    straight_fall_up: float = 0.6
    straight_timeout_s: float = 6.0
    # straight cost weights
    w_up: float = 8.0
    w_com: float = 30.0
    w_flat: float = 10.0
    w_pose: float = 5.0
    w_feet: float = 50.0
    rollout_dt: float = 0.0       # 0 keeps the model timestep


class GetUpPlanner:
    def __init__(self, robot_xml, mesh_dir, registry_path, joint_names, q_nominal, torque_limits,
                 params: RecoveryParams | None = None):
        from humanoid_sim import model_builder as mb
        from humanoid_sim.ankle_linkage import AnkleLinkage
        self.p = p = params or RecoveryParams()
        names = self.names = list(joint_names)
        options = mb.WorldOptions(
            robot_xml=str(robot_xml), mesh_dir=str(mesh_dir),
            joint_torque_limits=dict(zip(names, torque_limits)), timestep=0.0, foot_clearance_m=0.0,
            add_head=False, add_target=False, camera_resolution=(64, 48), stereo_resolution=(64, 48),
            scene_builder='walking_controller.recovery:add_cost_sensors')
        self.m = m = mb.build_model(options)
        m.eq_active0[m.eq(mb.BASE_WELD).id] = 0  # rollout datas take eq_active from the model
        self.d = mujoco.MjData(m)
        self.rollout_m = mb.build_model(replace(options, timestep=p.rollout_dt)) if p.rollout_dt > m.opt.timestep else m
        self.rollout_m.eq_active0[self.rollout_m.eq(mb.BASE_WELD).id] = 0
        self.datas = [mujoco.MjData(self.rollout_m) for _ in range(p.threads)]
        self.sadr = {n: m.sensor(n).adr[0] for n in COST_SENSORS}
        self.qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in names])
        self.vadr = np.array([m.jnt_dofadr[m.joint(n).id] for n in names])
        self.act = np.array([m.actuator(mb.actuator_name(n)).id for n in names])
        base = m.body(mb.BASE_BODY).jntadr[0]
        self.free, self.vb = m.jnt_qposadr[base], m.jnt_dofadr[base]
        self.torso = m.body('torso').id
        self.sole_sites = [m.site(s).id for s in SOLES]
        self.floor = m.geom('floor').id
        root = m.body_rootid[m.body(mb.BASE_BODY).id]
        self.foot_bodies = {m.body(s).id for s in SOLES}
        self.robot_geoms = {g for g in range(m.ngeom) if m.body_rootid[m.geom_bodyid[g]] == root}

        self.kp = np.array([next(v for k, v in KP.items() if k in n) for n in names], float)
        self.kd = 0.24 * np.sqrt(self.kp)
        self.lo = np.array([m.jnt_range[m.joint(n).id][0] for n in names])
        self.hi = np.array([m.jnt_range[m.joint(n).id][1] for n in names])
        self.leg = np.array(['shoulder' not in n and 'elbow' not in n for n in names])
        self.q_nominal = np.asarray(q_nominal, float).copy()
        self.q_stand = np.zeros(len(names))
        for part, deg in (('hip1', 26), ('knee', -52), ('ankle', 26)):
            for side in ('left', 'right'):
                if f'{side}_{part}_joint' in names:
                    self.q_stand[names.index(f'{side}_{part}_joint')] = math.radians(deg)

        # Four-bar ankles: drive range from the registry, motor angle and ratio tables.
        with open(registry_path) as stream:
            reg = yaml.safe_load(stream)
        reg = reg.get('motor_control_node', {}).get('ros__parameters', reg)['motors']
        self.ank = np.array([i for i, n in enumerate(names) if (reg.get(n) or {}).get('ankle_mapping')], int)
        self.tau_motor = np.array([torque_limits[i] for i in self.ank])
        self.a_grid = np.linspace(-2.2, 1.8, 4001)
        self.m_grid, self.j_grid = [], []
        for i in self.ank:
            self.lo[i] = max(self.lo[i], reg[names[i]]['min_position'])
            self.hi[i] = min(self.hi[i], reg[names[i]]['max_position'])
            lk = AnkleLinkage(reg[names[i]]['ankle_mapping'])
            mg, jg = np.empty_like(self.a_grid), np.empty_like(self.a_grid)
            i0 = int(np.argmin(np.abs(self.a_grid)))
            for order in (range(i0, len(self.a_grid)), range(i0, -1, -1)):
                lk.reset(0.0)
                for k in order:
                    mg[k], jg[k] = lk.motor(self.a_grid[k])
            self.m_grid.append(mg)
            self.j_grid.append(jg)
        self._linkage_tables()

        self.nstep = int(round(p.horizon_s / self.rollout_m.opt.timestep))
        self.tk = np.linspace(0.0, p.horizon_s, p.knots)
        self.ts = np.arange(self.nstep) * self.rollout_m.opt.timestep
        self.state_spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        self.nstate = mujoco.mj_stateSize(m, self.state_spec)
        self.rng = np.random.default_rng(p.seed)
        self.nom_lat, self.nom_fwd = self._nominal_feet()
        self.lock = threading.Lock()
        self.reset(RISE, 0.0, None)

    # ------------------------------------------------------------------ setup helpers
    def _linkage_tables(self):
        """Passive crank/rod angles closing each ankle loop, tabulated over the ankle angle: the measured
        state has only the motor joints, and a rollout started off the loop would spring it shut."""
        m, d = self.m, self.d
        self.loops = []
        for e in range(m.neq):
            if m.eq_type[e] != mujoco.mjtEq.mjEQ_CONNECT or not m.eq(e).name.endswith('_ankle_linkage'):
                continue
            side = m.eq(e).name.split('_')[0]
            ankle = self.names.index(f'{side}_ankle_joint')
            crank = m.jnt_qposadr[m.joint(f'{side}_ankle_crank_joint').id]
            rod = m.jnt_qposadr[m.joint(f'{side}_ankle_rod_joint').id]
            b1, b2 = m.eq_obj1id[e], m.eq_obj2id[e]
            a1, a2 = m.eq_data[e][:3].copy(), m.eq_data[e][3:6].copy()
            grid = np.linspace(-1.6, 1.6, 321)
            table = np.zeros((len(grid), 2))
            mujoco.mj_resetData(m, d)
            for order in (range(160, len(grid)), range(160, -1, -1)):
                x = np.zeros(2)
                for k in order:
                    d.qpos[self.qadr[ankle]] = grid[k]
                    for _ in range(30):  # Gauss-Newton on the anchor gap
                        d.qpos[crank], d.qpos[rod] = x
                        r = self._gap(b1, a1, b2, a2)
                        J = np.zeros((3, 2))
                        for c in range(2):
                            dx = np.zeros(2); dx[c] = 1e-6
                            d.qpos[crank], d.qpos[rod] = x + dx
                            J[:, c] = (self._gap(b1, a1, b2, a2) - r) / 1e-6
                        step = np.linalg.lstsq(J, -r, rcond=None)[0]
                        x = x + step
                        if np.abs(step).max() < 1e-10:
                            break
                    table[k] = x
            self.loops.append((self.qadr[ankle], crank, rod, grid, table))

    def _gap(self, b1, a1, b2, a2):
        mujoco.mj_kinematics(self.m, self.d)
        p1 = self.d.xpos[b1] + self.d.xmat[b1].reshape(3, 3) @ a1
        p2 = self.d.xpos[b2] + self.d.xmat[b2].reshape(3, 3) @ a2
        return p1 - p2

    def _nominal_feet(self):
        """Sole sites (left - right) at the start stance in the torso heading frame: (lateral, forward)."""
        d = mujoco.MjData(self.m)
        d.qpos[self.qadr] = self.q_nominal
        mujoco.mj_kinematics(self.m, d)
        R = d.xmat[self.torso].reshape(3, 3)
        delta = d.site_xpos[self.sole_sites[0]] - d.site_xpos[self.sole_sites[1]]
        return float(delta @ R[:, 0]), float(delta @ R[:, 1])

    # ------------------------------------------------------------------ state
    def reset(self, phase, t, joints):
        with self.lock:
            self.phase, self.t_phase, self.ok_since = phase, float(t), None
            q0 = np.zeros(len(self.names)) if joints is None else np.asarray(joints, float)
            self.nominal = np.tile(np.clip(q0, self.lo, self.hi), (self.p.knots, 1))
            self.plan_t0, self.plan_k = float(t), self.nominal.copy()
            self.ready_why = ''

    def set_state(self, base_pos, base_quat_wxyz, lin_world, ang_world, joints, rates):
        m, d = self.m, self.d
        d.qpos[:] = m.qpos0
        d.qvel[:] = 0.0
        d.qpos[self.free:self.free + 3] = base_pos
        quat = np.asarray(base_quat_wxyz, float)
        d.qpos[self.free + 3:self.free + 7] = quat / np.linalg.norm(quat)
        d.qvel[self.vb:self.vb + 3] = lin_world
        body = np.zeros(3)
        conj = d.qpos[self.free + 3:self.free + 7] * np.array([1, -1, -1, -1])
        mujoco.mju_rotVecQuat(body, np.asarray(ang_world, float), conj)
        d.qvel[self.vb + 3:self.vb + 6] = body
        d.qpos[self.qadr] = joints
        d.qvel[self.vadr] = rates
        for ankle, crank, rod, grid, table in self.loops:
            a = d.qpos[ankle]
            rate = d.qvel[self.vadr[int(np.nonzero(self.qadr == ankle)[0][0])]]
            for col, adr in ((0, crank), (1, rod)):
                d.qpos[adr] = np.interp(a, grid, table[:, col])
                slope = (np.interp(a + 1e-3, grid, table[:, col]) - np.interp(a - 1e-3, grid, table[:, col])) / 2e-3
                d.qvel[m.jnt_dofadr[np.nonzero(m.jnt_qposadr == adr)[0][0]]] = slope * rate
        mujoco.mj_forward(m, d)

    # ------------------------------------------------------------------ measurements on the planner model
    def floor_contacts(self):
        m, d, out = self.m, self.d, set()
        for c in d.contact[:d.ncon]:
            pair = (c.geom1, c.geom2)
            if self.floor in pair:
                other = pair[1] if pair[0] == self.floor else pair[0]
                if other in self.robot_geoms:
                    out.add(m.geom_bodyid[other])
        return out

    def feet_only(self):
        fc = self.floor_contacts()
        return bool(fc) and fc <= self.foot_bodies

    def soles_off_flat_deg(self):
        return max(math.degrees(math.acos(np.clip(self.d.site_xmat[s][8], -1, 1))) for s in self.sole_sites)

    def upright(self):
        return float(self.d.xmat[self.torso][8])

    # ------------------------------------------------------------------ ankle four-bar in the rollouts
    def _motor(self, k, a):
        return np.interp(a, self.a_grid, self.m_grid[k]), np.interp(a, self.a_grid, self.j_grid[k])

    def _controls(self, Q):
        """(B, nstep, nj) joint targets -> actuator controls with the MPC gains; ankles through the four-bar,
        linearised about the current ankle angle (as humanoid_sim.world)."""
        m, act = self.rollout_m, self.act
        Qc = np.clip(Q, self.lo, self.hi)
        U = np.zeros(Q.shape[:2] + (m.nu,))
        U[:, :, act] = self.kp * Qc
        m.actuator_biasprm[act, 1] = -self.kp
        m.actuator_biasprm[act, 2] = -self.kd
        for k, i in enumerate(self.ank):
            q0 = self.d.qpos[self.qadr[i]]
            m0, J = self._motor(k, q0)
            m_ref, _ = self._motor(k, Qc[:, :, i])
            kp = self.kp[i]
            U[:, :, act[i]] = J * kp * (m_ref - m0 + J * q0)
            m.actuator_biasprm[act[i], 1] = -J * J * kp
            m.actuator_biasprm[act[i], 2] = -J * J * self.kd[i]
            limit = abs(J) * self.tau_motor[k]
            m.actuator_forcerange[act[i]] = (-limit, limit)
        return U

    def _targets(self, K):
        tk, ts = self.tk, self.ts
        idx = np.clip(np.searchsorted(tk, ts, side='right') - 1, 0, len(tk) - 2)
        a = (ts - tk[idx]) / (tk[idx + 1] - tk[idx])
        return K[:, idx] * (1 - a)[None, :, None] + K[:, idx + 1] * a[None, :, None]

    # ------------------------------------------------------------------ cost
    def _cost(self, states, sens, K, phase):
        m, p, free, vb, s = self.m, self.p, self.free, self.vb, self.sadr
        qpos = states[:, :, 1:1 + m.nq]
        qvel = states[:, :, 1 + m.nq:1 + m.nq + m.nv]
        z = qpos[:, :, free + 2]
        w, x_, y_, zz = (qpos[:, :, free + 3 + i] for i in range(4))
        up = 1 - 2 * (x_ * x_ + y_ * y_)
        com = sens[:, :, s['cost_com']:s['cost_com'] + 3]
        lf = sens[:, :, s['cost_lf']:s['cost_lf'] + 3]
        rf = sens[:, :, s['cost_rf']:s['cost_rf'] + 3]
        mid = 0.5 * (lf + rf)
        lfz, rfz = sens[:, :, s['cost_lfz'] + 2], sens[:, :, s['cost_rfz'] + 2]
        joint = qpos[:, :, self.qadr]
        vel = (qvel[:, :, vb:vb + 3] ** 2).sum(2).mean(1)
        ang = (qvel[:, :, vb + 3:vb + 6] ** 2).sum(2).mean(1)
        if phase == RISE:
            c = 4.0 * (np.clip(0.62 - z, 0, None) ** 2).mean(1) / 0.62 ** 2 + 2.0 * ((1 - up) ** 2).mean(1)
            c = c + 3.0 * (np.clip(z - 0.75, 0, None) ** 2).mean(1) + vel + 0.05 * ang
            c = c + 2.0 * (np.clip(qvel[:, :, vb + 2] - 0.4, 0, None) ** 2).mean(1)
            c = c + 3.0 * ((com[:, :, :2] - mid[:, :, :2]) ** 2).sum(2).mean(1)
            c = c + 0.5 * ((1 - lfz) + (1 - rfz) + 10 * (lf[:, :, 2] + rf[:, :, 2])).mean(1)
            c = c + 0.02 * ((joint - self.q_stand) ** 2).mean((1, 2))
        else:
            leg, arm = self.leg, ~self.leg
            c = p.w_up * ((1 - up) ** 2).mean(1) + 4.0 * (np.clip(0.55 - z, 0, None) ** 2).mean(1) / 0.55 ** 2
            c = c + p.w_com * ((com[:, :, :2] - mid[:, :, :2]) ** 2).sum(2).mean(1)
            c = c + p.w_flat * ((1 - lfz) + (1 - rfz)).mean(1) + 40.0 * (lf[:, :, 2] + rf[:, :, 2]).mean(1)
            c = c + p.w_pose * ((joint[:, :, leg] - self.q_nominal[leg]) ** 2).mean((1, 2))
            # feet at the start stance, in the torso heading frame
            ex = np.stack([1 - 2 * (y_ * y_ + zz * zz), 2 * (x_ * y_ + w * zz), 0 * w], -1)
            ey = np.stack([2 * (x_ * y_ - w * zz), 1 - 2 * (x_ * x_ + zz * zz), 0 * w], -1)
            ex /= np.linalg.norm(ex, axis=-1, keepdims=True)
            ey /= np.linalg.norm(ey, axis=-1, keepdims=True)
            dl = lf - rf
            c = c + p.w_feet * (((dl * ex).sum(-1) - self.nom_lat) ** 2
                                + ((dl * ey).sum(-1) - self.nom_fwd) ** 2).mean(1)
            if arm.any():
                c = c + 0.2 * ((joint[:, :, arm] - self.q_nominal[arm]) ** 2).mean((1, 2))
            c = c + 2.0 * vel + 0.2 * ang
        return c + 0.01 * (np.diff(K, axis=1) ** 2).mean((1, 2))

    # ------------------------------------------------------------------ one plan
    def plan(self, t, forces):
        """Plan from the state last given to set_state (sampled at simulated time t). Returns the phase
        transition this plan's state calls for: None, 'straight', 'ready' or 'rise'."""
        p, m, d = self.p, self.m, self.d
        with self.lock:
            phase, nominal, t_prev = self.phase, self.nominal.copy(), self.plan_t0
        shift = max(0.0, t - t_prev)
        shifted = np.array([np.interp(self.tk + shift, self.tk, nominal[:, j])
                            for j in range(len(self.names))]).T
        noise = self.rng.standard_normal((p.samples, p.knots, len(self.names)))
        noise *= p.sigma_rise if phase == RISE else p.sigma_straight
        noise[0] = 0.0
        K = np.clip(shifted[None] + noise, self.lo, self.hi)
        Q = self._targets(K)
        state = np.empty(self.nstate)
        mujoco.mj_getState(m, d, state, self.state_spec)
        U = self._controls(Q)
        states, sens = mjr.rollout(self.rollout_m, self.datas, np.tile(state, (p.samples, 1)), U,
                                   control_spec=mujoco.mjtState.mjSTATE_CTRL)
        best = int(np.argmin(self._cost(states, sens, K, phase)))
        with self.lock:
            self.nominal, self.plan_k, self.plan_t0 = K[best], K[best], float(t)
        return self._transition(t, np.asarray(forces, float))

    def check(self, t, forces):
        """Phase transition for the state last given to set_state, without planning (the get-up student
        policy drives the joints; this planner only judges rise -> straight -> ready)."""
        with self.lock:
            return self._transition(t, np.asarray(forces, float))

    def target(self, t):
        """Joint targets of the current plan at simulated time t."""
        with self.lock:
            plan, t0 = self.plan_k, self.plan_t0
        s = float(np.clip(t - t0, 0.0, self.p.horizon_s))
        return np.array([np.interp(s, self.tk, plan[:, j]) for j in range(len(self.names))])

    def _transition(self, t, forces):
        p, d = self.p, self.d
        z, up = float(d.qpos[self.free + 2]), self.upright()
        feet_only = self.feet_only()
        if self.phase == RISE:
            speed = float(np.linalg.norm(d.qvel[self.vb:self.vb + 3]))
            ok = (z > 0.5 and up > max(0.85, math.cos(p.rise_max_tilt_rad)) and feet_only
                  and speed < p.rise_max_speed)
            self.ok_since = (self.ok_since if self.ok_since is not None else t) if ok else None
            if self.ok_since is not None and t - self.ok_since >= p.rise_hold_s:
                self.ready_why = f'z {z:.2f}, up {up:+.2f}, speed {speed:.2f}'
                return STRAIGHT
            return None
        # Lost the stand: as the balance policy's fall check (tools/rl_tasks/balance.py), a body other than
        # the feet on the floor, or too low / tilted. Both feet briefly in the air (a bouncy hand-over) is not.
        if (self.floor_contacts() - self.foot_bodies) or z < p.straight_fall_z or up < p.straight_fall_up:
            return RISE
        speed = float(np.linalg.norm(d.qvel[self.vb:self.vb + 3]))
        err = float(np.abs(d.qpos[self.qadr[self.leg]] - self.q_nominal[self.leg]).max())
        flat = self.soles_off_flat_deg()
        self.ready_why = (f'tilt {math.degrees(math.acos(min(1.0, up))):.0f} deg, leg joint error '
                          f'{math.degrees(err):.0f} deg, soles {flat:.0f} deg off flat, '
                          f'forces {np.round(forces).tolist()}')
        ok = up > math.cos(0.25) and min(forces) > 40.0 and speed < 0.15 and err < 0.25 and flat < 10
        self.ok_since = (self.ok_since if self.ok_since is not None else t) if ok else None
        if self.ok_since is not None and t - self.ok_since >= p.straight_hold_s:
            return 'ready'
        if (t - self.t_phase > p.straight_timeout_s and up > math.cos(0.35) and min(forces) > 30.0
                and err < 0.4 and flat < 15):
            self.ready_why = 'timeout, relaxed: ' + self.ready_why
            return 'ready'
        return None
