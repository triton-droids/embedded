"""One offline fall-recovery episode, with motor-side four-bar ankle control."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys
import time

# Also work directly from a source tree copied to a remote host.
ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'humanoid_control/walking_controller'),
                str(ROOT / 'simulation/humanoid_sim')]
for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'

import mujoco
import numpy as np

from walking_controller.offline import Command, OfflineSim, load
from walking_controller.recovery import GetUpPlanner, RecoveryParams
from walking_controller.stepping import StepParams, SteppingController
from walking_controller.wbc import WBCGains


def description_paths(desc):
    return dict(robot_xml=desc / 'mjcf/full_body_sim.xml', mesh_dir=desc / 'meshes',
                registry=desc / 'config/motors_full_body.yaml', urdf=desc / 'urdf/full_body_wbc_arms.urdf')


def make_controller(robot, limits):
    return SteppingController(robot, limits.torque, StepParams(
        step_length_m=0.04, activity='step', knee_rad=-0.9, max_tilt_rad=0.6,
        gains=WBCGains(com_kp_z=100, com_weight=10, torso_kp=150,
                       torso_weight=5, torso_pitch_rad=0.2)))


def apply_mit(sim, planner, command):
    """Apply motor-side MIT control, linearising the ankles at the current plant angle."""
    m, d, act = sim.model, sim.data, np.asarray(sim.act)
    if command is None:
        d.ctrl[act] = 0.0
        m.actuator_biasprm[act, 1:3] = 0.0
        return
    c = command
    d.ctrl[act] = c.kp * c.q_ref + c.kd * c.dq_ref + c.tau_ff
    m.actuator_biasprm[act, 1] = -c.kp
    m.actuator_biasprm[act, 2] = -c.kd
    for k, i in enumerate(planner.ank):
        q0 = d.qpos[sim.qpos[i]]
        m0, ratio = planner._motor(k, q0)
        ref, _ = planner._motor(k, c.q_ref[i])
        d.ctrl[act[i]] = ratio * (c.kp[i] * (ref - m0 + ratio * q0)
                                  + c.kd[i] * c.dq_ref[i] + c.tau_ff[i])
        m.actuator_biasprm[act[i], 1:3] = -ratio**2 * np.array([c.kp[i], c.kd[i]])
        limit = abs(ratio) * planner.tau_motor[k]
        m.actuator_forcerange[act[i]] = (-limit, limit)


def lying_pose(sim, planner, start):
    """Lower the rotated robot above the floor and settle for one second."""
    m, d = sim.model, sim.data
    angle = math.pi / 4 * (1 if start == 'supine' else -1)
    d.qpos[3:7] = [math.cos(angle), math.sin(angle), 0, 0]
    sim.release()
    mujoco.mj_forward(m, d)
    low = min(d.geom_xpos[g, 2] - m.geom_rbound[g] for g in range(m.ngeom)
              if m.geom_contype[g] and m.geom_bodyid[g] > 0)
    d.qpos[2] += 0.02 - low
    mujoco.mj_forward(m, d)
    zero = np.zeros(len(planner.names))
    for _ in range(round(1.0 / sim.dt)):
        apply_mit(sim, planner, Command(zero, zero, planner.kp, planner.kd, zero))
        mujoco.mj_step(m, d)
    d.time = 0.0  # Episode times exclude pose settling.


def episode(args):
    begin = time.monotonic()
    paths, robot, limits = load(description_paths(args.desc.resolve()))
    ctl = make_controller(robot, limits)
    params = RecoveryParams(seed=args.seed, threads=args.threads, samples=args.samples,
                            rollout_dt=args.rollout_dt)
    planner = GetUpPlanner(paths['robot_xml'], paths['mesh_dir'], paths['registry'],
                           robot.joint_names, ctl.q_nominal, limits.torque, params)
    sim = OfflineSim(robot, limits, paths, delay_steps=2)
    m, d = sim.model, sim.data
    mode = 'walk' if args.start.startswith('push:') else 'rise'
    if mode == 'rise':
        lying_pose(sim, planner, args.start)
        planner.reset(mode, 0.0, d.qpos[sim.qpos])
    events = [(0.0, mode, args.start)]
    first_stand = handoff = last_handoff = None
    falls, steps = 0, 0
    handoff_xy = None
    plan_ms, frames, labels = [], [], []
    next_plan, push_start = 0.0, None
    zero = np.zeros(len(robot.joint_names))
    for tick in range(int(round(args.sim_s / sim.dt))):
        t = sim.time
        if sim.base_held and t >= 1.0:
            sim.release()
            events.append((t, mode, 'gantry released'))
        forces = sim.foot_forces()
        command = None
        if mode == 'walk':
            q, v = sim.state()
            command = ctl.step(t, q, v, forces, sim.base_held)
            if last_handoff is not None:
                steps = getattr(ctl, 'steps', 0)
            if not sim.base_held and (command is None or sim.fallen()):
                why = f'controller fault: {ctl.fault}' if command is None else 'non-foot floor contact'
                mode = 'rise'
                events.append((t, mode, why))
                falls += int(handoff is not None)
                planner.reset(mode, t, d.qpos[sim.qpos])
                next_plan = t
            elif command is None:
                mode = 'fault'
                events.append((t, mode, ctl.fault))
                break
        if mode in ('rise', 'straight'):
            if t + 1e-9 >= next_plan:
                ang = np.zeros(3)
                mujoco.mju_rotVecQuat(ang, d.qvel[3:6], d.qpos[3:7])
                planner.set_state(d.qpos[:3], d.qpos[3:7], d.qvel[:3], ang,
                                  d.qpos[sim.qpos], d.qvel[sim.qvel])
                start = time.monotonic()
                transition = planner.plan(t, forces)
                plan_ms.append((time.monotonic() - start) * 1000)
                next_plan = t + params.plan_dt
                if transition in ('rise', 'straight'):
                    why = planner.ready_why if transition == 'straight' else 'lost the stand while straightening'
                    if transition == 'straight' and first_stand is None:
                        first_stand = t
                    mode = transition
                    events.append((t, mode, why))
                    planner.reset(mode, t, d.qpos[sim.qpos])
                elif transition == 'ready':
                    ctl = make_controller(robot, limits)
                    q, v = sim.state()
                    ctl.start_from_stand(t, q, v, forces)
                    mode = 'walk'
                    events.append((t, mode, planner.ready_why))
                    handoff = t if handoff is None else handoff
                    last_handoff, handoff_xy, steps = t, d.qpos[:2].copy(), 0
                    command = ctl.step(t, q, v, forces, False)
            if mode != 'walk':
                command = Command(planner.target(t), zero, planner.kp, planner.kd, zero)
        # Delay the command, then linearise at the angle where it is applied.
        sim.queue.append(command)
        if len(sim.queue) > sim.delay_steps:
            sim.command = sim.queue.pop(0)
        apply_mit(sim, planner, sim.command)
        if args.start.startswith('push:') and push_start is None and mode == 'walk' and getattr(ctl, 'steps', 0) >= 4:
            push_start = t
            events.append((t, mode, '150 N push for 0.3 s'))
        d.xfrc_applied[sim.base_body] = 0.0
        if push_start is not None and t < push_start + 0.3:
            angle = math.radians(float(args.start.split(':')[1]))
            d.xfrc_applied[sim.base_body, :2] = 150 * np.array([math.cos(angle), math.sin(angle)])
        mujoco.mj_step(m, d)
        if args.frames and tick % 3 == 0:
            frames.append(d.qpos.copy())
            labels.append(f'walk:{ctl.phase}' if mode == 'walk' else mode)
    if args.frames:
        args.frames.parent.mkdir(parents=True, exist_ok=True)
        # Structured .npy: np.load(path)['qpos'] and ['phase']; no pickle needed.
        saved = np.empty(len(frames), dtype=[('qpos', float, (m.nq,)), ('phase', 'U32')])
        saved['qpos'], saved['phase'] = np.asarray(frames).reshape(-1, m.nq), labels
        with args.frames.open('wb') as stream:
            np.save(stream, saved)
    result = dict(case=args.start, seed=args.seed, params=asdict(params), wall_s=time.monotonic() - begin,
                  sim_s=sim.time, events=events, first_stand_s=first_stand, handoff_s=handoff,
                  falls_after_handoff=falls, steps_after_last_handoff=steps,
                  distance_after_last_handoff_m=None if handoff_xy is None else float(np.linalg.norm(d.qpos[:2] - handoff_xy)),
                  end_mode=mode, plan_ms_mean=float(np.mean(plan_ms)) if plan_ms else None)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + '\n')
    print(f'{args.start} seed={args.seed} end={mode} stand={first_stand} handoff={handoff} '
          f'steps={steps} sim={sim.time:.2f}s wall={result["wall_s"]:.2f}s', flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--desc', type=Path, default=ROOT / 'humanoid_full_body_description')
    parser.add_argument('--start', default='supine')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--sim-s', type=float, default=40)
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--samples', type=int, default=96)
    parser.add_argument('--rollout-dt', type=float, default=0)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--frames', type=Path)
    args = parser.parse_args()
    try:
        valid_start = args.start in ('prone', 'supine') or (args.start.startswith('push:') and
                       math.isfinite(float(args.start[5:])))
    except ValueError:
        valid_start = False
    if not valid_start:
        parser.error('--start must be prone, supine, or push:<degrees>')
    if args.threads < 1 or args.samples < 1 or args.sim_s <= 0 or args.rollout_dt < 0:
        parser.error('threads, samples and sim-s must be positive; rollout-dt must be nonnegative')
    episode(args)


if __name__ == '__main__':
    main()
