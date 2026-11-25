"""Record one full-body SteppingController demonstration for velocity-conditioned BC."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_name] = '1'

import numpy as np  # noqa: E402


def torque_equivalent_action(c_kp, c_kd, c_q_ref, c_dq_ref, c_tau_ff,
                             q, dq, kp, kd, lo, hi, q_nom):
    """Convert joint-space teacher impedance to a bounded student position action.

    Equality is before actuator saturation and the nonlinear motor-side ankle law.
    """
    tau = c_kp * (c_q_ref - q) + c_kd * (c_dq_ref - dq) + c_tau_ff
    target = q + (tau + kd * dq) / kp
    return np.clip(target, lo, hi) - q_nom


PERIOD = 1.0
STANCE = 0.7


def gait_phase(gphase, stance_foot, next_stance, t, t_gphase, t_dur, left_foot):
    """Map the teacher's DS/SS progress to the RL cycle; None means no gait."""
    if gphase not in ('ds', 'ss'):
        return None
    tau = float(np.clip((t - t_gphase) / t_dur, 0, 1))
    if gphase == 'ds':
        return (0.0 if next_stance == left_foot else 0.5) + 0.2 * tau
    return (0.2 if stance_foot == left_foot else 0.7) + 0.3 * tau


def gait_command(velocity, phase, holding=False):
    """Append the real gait clock, suppressing it for zero commands or holds."""
    clock = np.zeros(2)
    if np.any(velocity) and not holding and phase is not None:
        clock = np.array([math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)])
    return np.concatenate([velocity, clock])


def episode(args):
    # Keep the pure conversion importable without the simulator dependencies.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from recovery_eval import apply_mit, description_paths, make_controller
    import mujoco
    from walking_controller.offline import Command, OfflineSim, load
    from walking_controller.getup_policy import CONTROL_DT, frame
    from walking_controller.recovery import GetUpPlanner, RecoveryParams
    from walking_controller.velocity import step_command
    from walking_controller.robot import FEET

    # An omitted option preserves the legacy recording and console format.
    diagnostic = getattr(args, "drive", None) is not None
    drive = getattr(args, "drive", None) or "teacher"
    begin = time.monotonic()
    rng = np.random.default_rng(args.seed)
    paths, robot, limits = load(description_paths(args.desc.resolve()))
    ctl = make_controller(robot, limits)
    ctl.p.t_ds, ctl.p.t_ss = 0.2, 0.3
    q_nom = ctl.q_nominal.copy()
    planner = GetUpPlanner(paths['robot_xml'], paths['mesh_dir'], paths['registry'],
                           robot.joint_names, q_nom, limits.torque, RecoveryParams(threads=1, samples=1))
    sim = OfflineSim(robot, limits, paths, delay_steps=2)
    m, d = sim.model, sim.data
    dr = {}
    if args.dr:
        m.geom('floor').friction[0] = rng.uniform(0.5, 1.2)
        scales = rng.uniform(0.9, 1.1, m.nbody)
        m.body_mass[1:] *= scales[1:]
        m.body_inertia[1:] *= scales[1:, None]
        dr = dict(friction=float(m.geom('floor').friction[0]), mass_scales=scales[1:].tolist())

    teacher_queue, teacher_command = [], None
    student_command = None
    zero = np.zeros(len(q_nom))
    ticks = round(CONTROL_DT / sim.dt)
    rec = {k: [] for k in ('frame', 'prev', 'label', 'command', 't', 'vx_meas', 'contacts', 'clock_contacts')}
    prev = np.zeros(len(q_nom))
    velocity = np.zeros(3)
    next_segment, segment_start = 2.0, 2.0
    errors, segments = [], []
    outcome, dropped = 'ok', 0
    for tick in range(round(args.sim_s / sim.dt)):
        t = sim.time
        # Integer tick comparisons avoid accumulating floating-point clock error.
        elapsed = tick * sim.dt
        if sim.base_held and elapsed >= 1.0:
            sim.release()
        if elapsed >= next_segment:
            segment_start = elapsed
            next_segment = elapsed + rng.uniform(*args.segment_s)
            if rng.random() < args.zero_frac:
                velocity = np.zeros(3)
                ctl.request_stop()
            else:
                velocity = np.array([rng.uniform(0, args.vx_max), 0.0,
                                     rng.uniform(-args.wz_max, args.wz_max)])
                ctl.resume()
                ctl.p.step_length_m, ctl.p.turn_rad = step_command(
                    velocity[0], velocity[2], ctl.p.t_ds + ctl.p.t_ss, 0.10, 0.15)
            segments.append(dict(t=t, command=velocity.tolist()))
        q, v = sim.state()
        command = ctl.step(t, q, v, sim.foot_forces(), sim.base_held)
        if command is None:
            outcome = f'fault: {ctl.fault}'
            break
        if not sim.base_held and sim.fallen():
            outcome = 'fell'
            break

        # Sample the pre-step state and the delayed command currently applied,
        # before this tick advances the delay queue or the physics.
        c = sim.command if drive == "teacher" else teacher_command
        if drive != "teacher" and c is not None and (drive == "label5" or tick % ticks == 0):
            label = torque_equivalent_action(
                c.kp, c.kd, c.q_ref, c.dq_ref, c.tau_ff,
                d.qpos[sim.qpos], d.qvel[sim.qvel], planner.kp, planner.kd,
                planner.lo, planner.hi, q_nom)
            student_command = Command(np.clip(q_nom + label, planner.lo, planner.hi),
                                      zero, planner.kp, planner.kd, zero)
        if elapsed >= 2.0 and tick % ticks == 0 and c is not None:
            joints, rates = d.qpos[sim.qpos], d.qvel[sim.qvel]
            label = torque_equivalent_action(c.kp, c.kd, c.q_ref, c.dq_ref, c.tau_ff,
                                             joints, rates, planner.kp, planner.kd,
                                             planner.lo, planner.hi, q_nom)
            if np.isfinite(label).all():
                w, x, y, z = d.qpos[3:7]
                yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                vx_meas = -math.sin(yaw) * d.qvel[0] + math.cos(yaw) * d.qvel[1]
                phase = gait_phase(getattr(ctl, 'gphase', None), getattr(ctl, 'stance', None),
                                   getattr(ctl, 'next_stance', None), t,
                                   getattr(ctl, 't_gphase', t), getattr(ctl, 't_dur', 1.0), FEET[0])
                conditioned = gait_command(velocity, phase, ctl.holding)
                if np.any(conditioned[3:]):
                    ph = phase % 1.0
                    expected = np.array([ph < STANCE, ph >= 0.5 or ph < 0.2])
                else:
                    expected = np.ones(2, dtype=bool)
                # Match TaskEnv.contacts: sole/floor collision pairs, not a force threshold.
                pairs = [set(c.geom) for c in d.contact[:d.ncon]]
                contacts = np.array([any(sim.floor in pair and foot in pair for pair in pairs)
                                     for foot in sim.foot_geoms])
                values = dict(frame=frame(d.qvel[3:6], d.qpos[3:7], joints, rates, q_nom),
                              prev=prev, label=label, command=conditioned, t=t, vx_meas=vx_meas,
                              contacts=contacts, clock_contacts=expected)
                for key, value in values.items():
                    rec[key].append(value)
                prev = label.copy()
                if np.any(velocity) and elapsed - segment_start >= 1.0:
                    errors.append(abs(vx_meas - velocity[0]))
            else:
                dropped += 1
        if drive != "teacher":
            # Advance only after sampling: this is the teacher command that
            # would be applied at the same pre-step point in the original loop.
            teacher_queue.append(command)
            if len(teacher_queue) > sim.delay_steps:
                teacher_command = teacher_queue.pop(0)
        sim.queue.append(command if drive == "teacher" else student_command)
        if len(sim.queue) > sim.delay_steps:
            sim.command = sim.queue.pop(0)
        apply_mit(sim, planner, sim.command)
        mujoco.mj_step(m, d)
        if not sim.base_held and sim.fallen():
            outcome = 'fell'
            break

    n = len(q_nom)
    widths = dict(frame=6 + 2 * n, prev=n, label=n, command=5, contacts=2, clock_contacts=2)
    arrays = {key: np.asarray(value, np.float32).reshape((-1, widths[key]) if key in widths else (-1,))
              for key, value in rec.items()}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, **arrays, q_nominal=q_nom.astype(np.float32), history=5,
                        scale=1.0, control_dt=CONTROL_DT, period=PERIOD, stance=STANCE, joint_names=np.array(robot.joint_names))
    count = len(rec['t'])
    for i, segment in enumerate(segments):
        end = segments[i + 1]['t'] if i + 1 < len(segments) else sim.time
        mask = (arrays['t'] >= segment['t'] + 1.0 - 1e-6) & (arrays['t'] < end - 1e-6)
        speeds = arrays['vx_meas'][mask]
        segment.update(end_t=end, measured_ticks=int(mask.sum()),
                       vx_meas_mean=float(speeds.mean()) if len(speeds) else None,
                       vx_abs_error_mean=float(np.abs(speeds - segment['command'][0]).mean())
                       if len(speeds) else None)
    matches = arrays['contacts'] == arrays['clock_contacts']
    agreement = float(matches.all(axis=1).mean()) if count else None
    result = dict(period=PERIOD, stance=STANCE, clock_contact_agreement=agreement,
                  clock_contact_agreement_per_foot=matches.mean(axis=0).tolist() if count else None,
                  seed=args.seed, outcome=outcome, recorded_ticks=count, short_episode=count < 50,
                  steps=int(getattr(ctl, 'steps', 0)), vx_error_mean=float(np.mean(errors)) if errors else None,
                  vx_error_ticks=len(errors), dropped_nonfinite_ticks=dropped, dr=dr, segments=segments,
                  wall_s=time.monotonic() - begin)
    if diagnostic:
        result.update(drive=drive, time_s=sim.time)
    args.out.with_suffix('.json').write_text(json.dumps(result, indent=2) + '\n')
    prefix = f'walk drive={drive}' if diagnostic else 'walk'
    print(f'{prefix} seed={args.seed} {outcome} t={sim.time:.2f}s ticks={count} steps={result["steps"]} '
          f'vx_error={result["vx_error_mean"]} wall={result["wall_s"]:.1f}s', flush=True)
    return result


def main():
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from recovery_eval import ROOT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--sim-s', type=float, default=30)
    parser.add_argument('--dr', action='store_true')
    parser.add_argument('--drive', choices=('teacher', 'label5', 'label20'), default=None,
                        help='actuation source (default: teacher); explicit use adds diagnostic metadata')
    parser.add_argument('--vx-max', type=float, default=0.16)
    parser.add_argument('--wz-max', type=float, default=0.2)
    parser.add_argument('--zero-frac', type=float, default=0.2)
    parser.add_argument('--segment-s', type=float, nargs=2, default=[3, 5])
    parser.add_argument('--desc', type=Path, default=ROOT / 'description' / 'humanoid_full_body_description')
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in [args.sim_s, args.vx_max, args.wz_max,
                                         args.zero_frac, *args.segment_s]):
        parser.error('numeric options must be finite')
    if args.sim_s <= 0 or min(args.vx_max, args.wz_max) < 0 or not 0 <= args.zero_frac <= 1:
        parser.error('require sim-s > 0, velocity maxima >= 0, and zero-frac in [0, 1]')
    if not 0 < args.segment_s[0] <= args.segment_s[1]:
        parser.error('require 0 < segment-s minimum <= maximum')
    episode(args)


if __name__ == '__main__':
    main()
