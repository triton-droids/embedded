"""Record get-up demonstrations from the MPC teacher (GetUpPlanner), optionally while a student policy
drives the robot (DAgger), or evaluate a student alone.

One episode: the robot starts lying (prone, supine, or a random pose dropped onto the floor) and runs until
the planner's ready() check passes (success) or --sim-s elapses. Control runs at 50 Hz; every control tick
stores the proprioceptive frame the student sees and the teacher's joint target at that time.

  tools/getup_demo.py --start random --seed 3 --out demos/random_3.npz
  tools/getup_demo.py --start random --seed 3 --student bc.npz --beta 0.5 --out dagger/random_3.npz
  tools/getup_demo.py --start supine --seed 0 --student bc.npz --beta 0 --no-teacher --out eval/s0.npz

Student observation (one control tick), shared with getup_bc.py through `observation()`:
  HISTORY frames of [base angular velocity (body, 3), gravity direction (body, 3), joint_pos - q_nominal (18),
  joint_vel (18)], newest last, then the previous action (18). Action a: joint target = q_nominal + SCALE * a,
  tracked with the planner's MIT gains (four-bar ankles through the motor-side law), as the MPC does.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from recovery_eval import (ROOT, apply_mit, description_paths, lying_pose,  # noqa: E402  (sets sys.path)
                           make_controller)

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from walking_controller.offline import Command, OfflineSim, load  # noqa: E402
from walking_controller.getup_policy import CONTROL_DT, GetUpPolicy, frame as policy_frame, observation  # noqa: E402,F401
from walking_controller.recovery import GetUpPlanner, RecoveryParams  # noqa: E402

HISTORY = 3
SCALE = 1.0          # rad per unit action
FRAME = 3 + 3 + 18 + 18


def frame(d, sim, q_nominal):
    """Proprioceptive frame: what an IMU and the joint encoders give on hardware."""
    return policy_frame(d.qvel[3:6], d.qpos[3:7], d.qpos[sim.qpos], d.qvel[sim.qvel], q_nominal)


def random_pose(sim, planner, rng):
    """Random orientation and joint angles, dropped from just above the floor, settled for 1.5 s."""
    m, d = sim.model, sim.data
    quat = rng.standard_normal(4)
    d.qpos[3:7] = quat / np.linalg.norm(quat)
    mid, half = 0.5 * (planner.lo + planner.hi), 0.5 * (planner.hi - planner.lo)
    joints = np.clip(rng.uniform(-0.5, 0.5, len(mid)) * half + 0.3 * mid, planner.lo, planner.hi)
    d.qpos[sim.qpos] = joints
    sim.release()
    mujoco.mj_forward(m, d)
    low = min(d.geom_xpos[g, 2] - m.geom_rbound[g] for g in range(m.ngeom)
              if m.geom_contype[g] and m.geom_bodyid[g] > 0)
    d.qpos[2] += 0.05 - low
    mujoco.mj_forward(m, d)
    zero = np.zeros(len(joints))
    for _ in range(round(1.5 / sim.dt)):
        apply_mit(sim, planner, Command(joints, zero, 0.3 * planner.kp, planner.kd, zero))
        mujoco.mj_step(m, d)
    d.time = 0.0


def randomise(sim, rng):
    """Domain randomisation of the plant only (the teacher keeps its nominal model)."""
    m = sim.model
    m.geom('floor').friction[0] = rng.uniform(0.5, 1.2)
    scale = rng.uniform(0.9, 1.1, m.nbody)
    m.body_mass[1:] *= scale[1:]
    m.body_inertia[1:] *= scale[1:, None]
    return dict(friction=float(m.geom('floor').friction[0]))


def _student_obs(student, d, sim, q_nom, t):
    """Observation the student would see now, without advancing its history."""
    frames = (student.frames or [frame(d, sim, q_nom)] * student.history)[1:] + [frame(d, sim, q_nom)]
    return observation(frames, student.prev, t if student.time_input else None)


def walk_after_handoff(sim, planner, robot, limits, t0, forces, walk_s, frames=None):
    """Start the walking controller from the current (roughly standing) state, as the ROS node's hand-over
    does, and run it for walk_s. Returns 'walk' or why it failed."""
    m, d = sim.model, sim.data
    ctl = make_controller(robot, limits)
    q, v = sim.state()
    ctl.start_from_stand(t0, q, v, forces)
    while sim.time < t0 + walk_s:
        q, v = sim.state()
        command = ctl.step(sim.time, q, v, sim.foot_forces(), False)
        if command is None:
            return f'fault: {ctl.fault}'
        if sim.fallen():
            return 'fell'
        sim.queue.append(command)
        if len(sim.queue) > sim.delay_steps:
            sim.command = sim.queue.pop(0)
        apply_mit(sim, planner, sim.command)
        mujoco.mj_step(m, d)
        if frames is not None and round(sim.time / sim.dt) % 4 == 0:
            frames.append((sim.time, d.qpos.copy(), d.qvel.copy(), d.actuator_force.copy(), 'walk'))
    return 'walk'


def episode(args):
    begin = time.monotonic()
    rng = np.random.default_rng(args.seed + 7919)
    paths, robot, limits = load(description_paths(args.desc.resolve()))
    q_nom = make_controller(robot, limits).q_nominal.copy()
    params = RecoveryParams(seed=args.seed, threads=args.threads, samples=args.samples,
                            rollout_dt=args.rollout_dt)
    planner = GetUpPlanner(paths['robot_xml'], paths['mesh_dir'], paths['registry'],
                           robot.joint_names, q_nom, limits.torque, params)
    sim = OfflineSim(robot, limits, paths, delay_steps=2)
    m, d = sim.model, sim.data
    dr = randomise(sim, rng) if args.dr else {}
    if args.start == 'random':
        random_pose(sim, planner, rng)
    else:
        lying_pose(sim, planner, args.start)
    planner.reset('rise', 0.0, d.qpos[sim.qpos])
    student = GetUpPolicy(args.student) if args.student else None
    teacher = not args.no_teacher
    if not teacher and student is None:
        raise SystemExit('--no-teacher needs --student')

    ticks = round(CONTROL_DT / sim.dt)
    plan_every = max(1, round(params.plan_dt / CONTROL_DT))
    zero = np.zeros(len(q_nom))
    hist = [frame(d, sim, q_nom)] * HISTORY
    prev = np.zeros(len(q_nom))
    use_student = False
    rec = {k: [] for k in ('frame', 'prev', 'label', 'executed', 'student', 't', 'phase', 'rise_ok')}
    handoff = None
    frames = [] if args.frames else None
    events, outcome, plan_ms = [(0.0, 'rise', args.start)], 'timeout', []
    k = 0
    while sim.time < args.sim_s:
        t = sim.time
        forces = sim.foot_forces()
        ang = np.zeros(3)
        mujoco.mju_rotVecQuat(ang, d.qvel[3:6], d.qpos[3:7])
        planner.set_state(d.qpos[:3], d.qpos[3:7], d.qvel[:3], ang, d.qpos[sim.qpos], d.qvel[sim.qvel])
        if student is not None and args.guide > 0 and teacher and k % plan_every == 0 and k > 0:
            # Policy-guided teacher: pull the MPC's warm start toward the student's current target so
            # the teacher refines the student's mode instead of switching to another one.
            guess = np.clip(q_nom + SCALE * student(_student_obs(student, d, sim, q_nom, t)), planner.lo, planner.hi)
            with planner.lock:
                planner.nominal = (1 - args.guide) * planner.nominal + args.guide * guess[None, :]
        if teacher and k % plan_every == 0:
            start = time.monotonic()
            transition = planner.plan(t, forces)
            plan_ms.append((time.monotonic() - start) * 1000)
        else:
            transition = planner._transition(t, np.asarray(forces, float))
        if transition == 'ready':
            outcome = 'ready'
            events.append((t, 'ready', planner.ready_why))
            if args.walk_after_ready:
                handoff = t
                outcome = 'ready+' + walk_after_handoff(sim, planner, robot, limits, t, forces, args.walk_s, frames)
                events.append((t, 'handoff', outcome))
            break
        if transition == 'straight' and args.walk_after_straight:
            events.append((t, 'straight', planner.ready_why))
            handoff = t
            outcome = 'straight+' + walk_after_handoff(sim, planner, robot, limits, t, forces, args.walk_s)
            events.append((t, 'handoff', outcome))
            break
        if transition in ('rise', 'straight'):
            events.append((t, transition, planner.ready_why))
            planner.reset(transition, t, d.qpos[sim.qpos])
            if teacher:
                planner.plan(t, forces)
        rise_ok = (float(planner.d.qpos[planner.free + 2]) > args.handoff_z and planner.upright() > args.handoff_up
                   and planner.feet_only())
        if args.early_handoff and rise_ok:
            handoff = t
            outcome = walk_after_handoff(sim, planner, robot, limits, t, forces, args.walk_s)
            events.append((t, 'handoff', outcome))
            break
        hist = hist[1:] + [frame(d, sim, q_nom)]
        obs = observation(hist, prev)
        label = (planner.target(t) - q_nom) / SCALE if teacher else zero
        a_student = student.step(hist[-1], t)[0] if student is not None else zero
        if student is not None and k % round(args.segment_s / CONTROL_DT) == 0:
            use_student = (not teacher) or rng.random() >= args.beta
        if args.mpc_straight and teacher:
            # Hybrid: the student rises, the MPC straightens (rise -> straight -> ready as in the ROS node).
            use_student = planner.phase == 'rise'
        action = a_student if use_student else label + args.act_noise * rng.standard_normal(len(label))
        for key, value in (('frame', hist[-1]), ('prev', prev), ('label', label), ('executed', action),
                           ('student', float(use_student)), ('t', t), ('phase', planner.phase == 'straight'),
                           ('rise_ok', rise_ok)):
            rec[key].append(value)
        target = np.clip(q_nom + SCALE * action, planner.lo, planner.hi)
        command = Command(target, zero, planner.kp, planner.kd, zero)
        for _ in range(ticks):
            sim.queue.append(command)
            if len(sim.queue) > sim.delay_steps:
                sim.command = sim.queue.pop(0)
            apply_mit(sim, planner, sim.command)
            mujoco.mj_step(m, d)
        if frames is not None:
            who = 'student' if use_student else 'MPC'
            frames.append((t, d.qpos.copy(), d.qvel.copy(), d.actuator_force.copy(), f'{planner.phase} ({who})'))
        prev = action
        if student is not None:
            student.prev = action  # the student sees the executed action, as at deployment
        k += 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    arrays = {key: np.asarray(value, np.float32) for key, value in rec.items()}
    np.savez_compressed(args.out, **arrays, q_nominal=q_nom, history=HISTORY, scale=SCALE,
                        control_dt=CONTROL_DT, joint_names=np.array(robot.joint_names))
    result = dict(case=args.start, seed=args.seed, outcome=outcome, ready_s=t if outcome == 'ready' else None,
                  handoff_s=handoff, rise_ok_ticks=int(np.sum(rec['rise_ok'])),
                  steps=k, student_share=float(np.mean(rec['student'])) if k else 0.0, beta=args.beta,
                  dr=dr, teacher=teacher, plan_ms_mean=float(np.mean(plan_ms)) if plan_ms else None,
                  wall_s=time.monotonic() - begin, events=events)
    args.out.with_suffix('.json').write_text(json.dumps(result, indent=2) + '\n')
    if frames:
        # Structured .npy (no pickle): ['t'], ['qpos'], ['qvel'], ['force'] (actuators), ['phase'] at 50 Hz.
        saved = np.empty(len(frames), dtype=[('t', float), ('qpos', float, (m.nq,)), ('qvel', float, (m.nv,)),
                                             ('force', float, (m.nu,)), ('phase', 'U32')])
        for k_, (tt, q, v, f, ph) in enumerate(frames):
            saved[k_] = (tt, q, v, f, ph)
        np.save(args.frames, saved)
    print(f'{args.start} seed={args.seed} {outcome} t={t:.2f}s ticks={k} student={result["student_share"]:.2f} '
          f'wall={result["wall_s"]:.0f}s', flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--desc', type=Path, default=ROOT / 'description' / 'humanoid_full_body_description')
    parser.add_argument('--start', choices=('prone', 'supine', 'random'), default='random')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--sim-s', type=float, default=20)
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--samples', type=int, default=96)
    parser.add_argument('--rollout-dt', type=float, default=0.01)
    parser.add_argument('--student', type=Path, help='npz policy from getup_bc.py')
    parser.add_argument('--beta', type=float, default=1.0, help='probability a segment is driven by the teacher')
    parser.add_argument('--segment-s', type=float, default=0.5, help='teacher/student switching interval')
    parser.add_argument('--act-noise', type=float, default=0.0, help='noise on executed teacher actions (DART)')
    parser.add_argument('--guide', type=float, default=0.0,
                        help='with --student: blend the MPC warm start toward the student target (0..1)')
    parser.add_argument('--no-teacher', action='store_true', help='evaluate the student only (no MPC)')
    parser.add_argument('--dr', action='store_true', help='randomise friction and link masses')
    parser.add_argument('--early-handoff', action='store_true',
                        help='hand over to the walking controller as soon as the robot stands on its feet '
                             '(outcome walk / fell / fault after --walk-s)')
    parser.add_argument('--walk-after-ready', action='store_true',
                        help='after ready() (the ROS node hand-over), run the walking controller for --walk-s')
    parser.add_argument('--walk-after-straight', action='store_true',
                        help='hand over to the walking controller when the rise check passes (feet only, z > 0.5, '
                             'torso up, held rise_hold_s) instead of after the straight phase')
    parser.add_argument('--mpc-straight', action='store_true',
                        help='with --student: the student drives the rise phase, the MPC the straight phase')
    parser.add_argument('--handoff-z', type=float, default=0.5)
    parser.add_argument('--handoff-up', type=float, default=0.85)
    parser.add_argument('--walk-s', type=float, default=5.0)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--frames', type=Path, help='save the trajectory (t, qpos, phase) at 50 Hz to this .npy')
    episode(parser.parse_args())


if __name__ == '__main__':
    main()
