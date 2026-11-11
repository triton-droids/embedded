#!/usr/bin/env python3
"""Offline kick evaluation (no ROS): stand, then kick, optionally at a ball.

  kick_eval.py                       # air kick with the right foot
  kick_eval.py --ball                # ball placed in front of the kicking foot
  kick_eval.py --ball --set t_kick=0.8 kick_reach_m=0.22
  kick_eval.py --walk 0.04 --ball    # kick out of walking (step length 0.04)

Prints one JSON line: fall/fault, kick events, peak foot speed, ball speed
and direction, torso tilt during the kick.
"""
import argparse
import json
import math

import mujoco
import numpy as np

from walking_controller.offline import load, run
from walking_controller.robot import FEET, other
from walking_controller.stepping import StepParams, SteppingController
from walking_controller.wbc import WBCGains

GAINS = dict(com_kp_z=100.0, com_weight=10.0)
BALL_SCENE = {'add_head': True, 'scene_builder': 'humanoid_ball_chase.scene:build_scene'}


def value(text):
    try:
        return float(text)
    except ValueError:
        return text


def episode(foot='right_foot', ball=False, ball_ahead=None, ball_lateral=0.0, walk=0.0, kick_at=8.0,
            duration=14.0, overrides=None, timing=None, stop_walk=False, ball_base=None):
    """stop_walk: walk (step length `walk`) and stand 3 s before the kick, as the chase does (feet
    end up staggered). ball_base: (x, y) ball centre in base_link at the kick request instead of
    relative to the kicking foot."""
    paths, robot, limits = load()
    params = StepParams(step_length_m=walk, gains=WBCGains(**GAINS))
    for k, v in (overrides or {}).items():
        setattr(params, k, v)
    ctl = SteppingController(robot, limits.torque, params)
    st = dict(requested=False, stop=False, ball_set=False, foot_v=0.0, tilt=0.0, kick_t=None, kick_end=None,
              ball_v=0.0, ball_dir=None, ball_start=None, ball_hit_t=None, roll=0.0, pitch=0.0, qd=0.0)
    ball_q = ball_v = ball_body = None

    def on_step(t, sim, c):
        nonlocal ball_q, ball_v, ball_body
        if ball and ball_body is None:
            m = sim.model
            ball_body = m.body('chase_ball').id
            j = m.joint('chase_ball_free')
            ball_q, ball_v = m.jnt_qposadr[j.id], m.jnt_dofadr[j.id]
        if not hasattr(c, 'fwd_axis'):
            return False
        if walk == 0.0 and not st['stop']:
            c.request_stop()
            st['stop'] = True
        if stop_walk and t >= kick_at - 3.0 and not st['stop']:
            c.p.step_length_m = 0.0
            c.request_stop()
            st['stop'] = True
        if t >= kick_at - 2.0 and walk and not st['stop']:
            c.p.step_length_m = 0.0
        if t >= kick_at and not st['requested']:
            c.request_kick(foot)
            st['requested'] = True
        if ball and not st['ball_set'] and (c.kick_armed == foot or c.kicking):
            # In front of where the kicking foot will land, at strike height.
            stance = other(foot)
            spot = c._sole_center(foot, c.footholds[stance] * c.nominal_offset[foot])
            toe = c.half_len[foot][1]
            ahead = ball_ahead if ball_ahead is not None else 0.5 * (c.p.kick_reach_m - c.p.kick_windup_m) + toe + 0.06
            pos = spot + ahead * c.fwd_axis + ball_lateral * c.lat_axis
            d = sim.data
            if ball_base is not None:
                left3 = np.array([-c.fwd_axis[1], c.fwd_axis[0], 0.0])
                pos = np.array([*d.qpos[:2], 0.0]) + ball_base[0] * c.fwd_axis + ball_base[1] * left3
            d.qpos[ball_q:ball_q + 3] = [pos[0], pos[1], 0.10 + sim.model.geom('floor').pos[2]]
            d.qpos[ball_q + 3:ball_q + 7] = [1, 0, 0, 0]
            d.qvel[ball_v:ball_v + 6] = 0.0
            mujoco.mj_forward(sim.model, d)
            st['ball_set'] = True
            # Ball and kicking-foot sole relative to the base, in ROS base_link axes (x forward, y
            # left = forward rotated +90 deg). Not lat_axis: that points to left_foot, which in this
            # model is on the robot's right.
            base = d.qpos[:2].copy()
            fwd = c.fwd_axis[:2]
            left = np.array([-fwd[1], fwd[0]])
            st['ball_base'] = [round(float((pos[:2] - base) @ fwd), 3), round(float((pos[:2] - base) @ left), 3)]
            st['foot_base'] = [round(float((spot[:2] - base) @ fwd), 3), round(float((spot[:2] - base) @ left), 3)]
            st['ball_start'] = d.qpos[ball_q:ball_q + 2].copy()
            st['fwd'] = c.fwd_axis[:2].copy()
        if st['kick_t'] and t < (st['kick_end'] or t) + 1.0:
            # Torso tilt split into roll (about the walking direction) and pitch, degrees.
            z = c.rm.frame('torso').rotation[:, 2]
            st['roll'] = max(st['roll'], abs(math.degrees(math.asin(np.clip(z @ c.lat_axis, -1, 1)))))
            st['pitch'] = max(st['pitch'], abs(math.degrees(math.asin(np.clip(z @ c.fwd_axis, -1, 1)))))
            st['qd'] = max(st['qd'], float(np.abs(sim.data.qvel[sim.qvel]).max()))
        if c.kicking:
            st['kick_t'] = st['kick_t'] or t
            body = sim.model.body(foot).id
            vel = np.zeros(6)
            mujoco.mj_objectVelocity(sim.model, sim.data, mujoco.mjtObj.mjOBJ_BODY, body, vel, 0)
            st['foot_v'] = max(st['foot_v'], float(vel[3:5] @ c.fwd_axis[:2]))
            st['tilt'] = max(st['tilt'], c.status.get('tilt', 0.0))
        elif st['kick_t'] and st['kick_end'] is None:
            st['kick_end'] = t
        if ball and st['ball_set']:
            v = sim.data.qvel[ball_v:ball_v + 2]
            speed = float(np.linalg.norm(v))
            if speed > st['ball_v']:
                st['ball_v'] = speed
                st['ball_dir'] = math.degrees(math.atan2(*reversed(
                    [float(v @ st['fwd']), float(v @ np.array([-st['fwd'][1], st['fwd'][0]]))])))
                st['ball_hit_t'] = st['ball_hit_t'] or (t if speed > 0.05 else None)
        return False

    res = run(ctl, duration, paths=paths, robot=robot, limits=limits, on_step=on_step,
              world=BALL_SCENE if ball else None, timing=timing)
    out = dict(foot=foot, fallen=res['fallen'], fault=res['fault'], t_end=round(res['t_end'], 2),
               kicks=ctl.kicks, kick_start=st['kick_t'], kick_end=st['kick_end'],
               peak_foot_speed=round(st['foot_v'], 3), max_tilt=round(st['tilt'], 3),
               roll_deg=round(st['roll'], 1), pitch_deg=round(st['pitch'], 1), max_joint_speed=round(st['qd'], 2),
               events=[e for e in ctl.events if 'kick' in e['event'] or e.get('kick')])
    if ball:
        d = res['sim'].data
        out.update(ball_peak_speed=round(st['ball_v'], 3),
                   ball_dir_deg=None if st['ball_dir'] is None else round(st['ball_dir'], 1),
                   ball_hit_t=st['ball_hit_t'], ball_base=st.get('ball_base'), foot_base=st.get('foot_base'),
                   ball_moved_m=round(float(np.linalg.norm(d.qpos[ball_q:ball_q + 2] - st['ball_start'])), 3)
                   if st['ball_start'] is not None else None)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--foot', default='right_foot', choices=FEET)
    ap.add_argument('--ball', action='store_true')
    ap.add_argument('--ball-ahead', type=float)
    ap.add_argument('--ball-lateral', type=float, default=0.0)
    ap.add_argument('--walk', type=float, default=0.0, help='step length before the kick (0: stand)')
    ap.add_argument('--stop-walk', action='store_true', help='walk, then stand 3 s before the kick')
    ap.add_argument('--ball-base', type=float, nargs=2, help='ball centre x y in base_link at the kick')
    ap.add_argument('--duration', type=float, default=14.0)
    ap.add_argument('--chain', action='store_true', help='Jetson ROS-chain timing instead of a fixed delay')
    ap.add_argument('--seed', type=int, default=0, help='chain timing seed')
    ap.add_argument('--set', nargs='*', default=[], help='StepParams overrides, name=value')
    a = ap.parse_args()
    overrides = {k: value(v) for k, v in (s.split('=', 1) for s in a.set)}
    timing = None
    if a.chain:
        from walking_controller.offline import ChainTiming
        timing = ChainTiming(seed=a.seed)
    print(json.dumps(episode(a.foot, a.ball, a.ball_ahead, a.ball_lateral, a.walk, duration=a.duration,
                             overrides=overrides, timing=timing, stop_walk=a.stop_walk,
                             ball_base=a.ball_base), default=float))


if __name__ == '__main__':
    main()
