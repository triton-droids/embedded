"""Stepping with a LIPM MPC on top of the WBC (after the stand sequence).

Phase cycle: DS (shift towards the next stance foot) -> SS (other foot
swings) -> DS -> SS ... with fixed nominal durations; touchdown events
re-anchor the schedule.

- MPC: one LipmMPC per horizontal axis (lateral, forward) in the frame of the
  initial stance; CoP bounds per interval from the planned support (DS: both
  feet; SS: stance sole). Re-solved every cycle from the measured CoM state.
- WBC: CoM acceleration feedforward omega^2 (c - p0), CoP task p0.
  DS: both feet roll on parallel edges towards the side the CoM is on (the
  only way the CoM moves sideways in double support without ankle roll).
  The last part of DS ramps the next swing foot's load down.
  SS: stance foot flat (reference with its roll/pitch removed), swing foot
  follows a lift / move / land trajectory to its next foothold.
- Footholds: stepping in place, or a forward step length per step.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np
import pinocchio as pin

from .gait import ACTIVE, GaitController, GaitParams, min_jerk
from .lipm_mpc import LipmMPC, MPCWeights
from .robot import FEET, other
from .wbc import FLAT, INNER, OUTER, SWING

DS, SS = 'ds', 'ss'
HOLD = 'hold'   # planned double support with no further lift-off (standing still)


@dataclass
class StepParams(GaitParams):
    activity: str = 'step'
    t_ds: float = 0.2
    t_ss: float = 0.3
    t_first_ds: float = 0.4
    unload_s: float = 0.06
    lift_m: float = 0.025
    # The swing foot's horizontal motion finishes at this fraction of single
    # support (height still follows the full phase). Touchdown is accepted
    # from s > 0.6, so with 1.0 an early touchdown leaves the step short.
    swing_fwd_end: float = 0.85
    step_length_m: float = 0.0
    # Heading change per step (rad, + = left / counter-clockwise). Each swing
    # target is rotated by it about the stance foot, the MPC frame and the
    # torso yaw reference turn with it. 0: exactly the straight gait.
    turn_rad: float = 0.0
    # The swing foot's yaw reaches the turn at this fraction of SS. Each swing
    # has to rotate its foot by about twice the turn (the feet alternate), and
    # touchdown comes at s ~ 0.6.
    swing_yaw_end: float = 0.5
    first_stance: str = 'left_foot'
    lat_margin_m: float = 0.01
    fwd_margin_m: float = 0.02
    touchdown_force_n: float = 20.0
    touchdown_timeout_s: float = 0.15
    lean_deadband_m: float = 0.003
    max_steps: int = 0
    mpc_dt: float = 0.04
    mpc_horizon: int = 25
    mpc_weights: MPCWeights = field(default_factory=MPCWeights)


class SteppingController(GaitController):
    def __init__(self, robot, torque_limits, params: StepParams | None = None):
        super().__init__(robot, torque_limits, params or StepParams())
        self.events: list[dict] = []
        # Stand still (request_stop) / walk again (resume). Requests are flags
        # consumed by the control step, so any thread may call them.
        self.stop_requested = False
        self.holding = False
        self._resume_requested = False

    def request_stop(self):
        """Finish the current step, then stand in double support (no stepping in place)."""
        self._resume_requested = False
        self.stop_requested = True

    def resume(self):
        """Leave the stand: a full double-support phase, then the next lift-off."""
        if self.stop_requested:
            self._resume_requested = True

    # ------------------------------------------------------------ setup
    def _start_active(self, t, q):
        super()._start_active(t, q)
        p = self.p
        torso_y = self.rm.frame('torso').rotation[:, 1]
        fwd = np.array([torso_y[0], torso_y[1], 0.0])
        fwd -= (fwd @ self.lat_axis) * self.lat_axis
        self.fwd_axis = fwd / np.linalg.norm(fwd)
        self.mpc_lat = LipmMPC(self.omega, p.mpc_dt, p.mpc_horizon, p.mpc_weights)
        self.mpc_fwd = LipmMPC(self.omega, p.mpc_dt, p.mpc_horizon, p.mpc_weights)
        self.footholds = {f: self.rm.frame(f).copy() for f in FEET}
        # Nominal placement of each foot relative to the other one, from the
        # initial stance. Swing targets come from the stance foot plus this
        # offset, so slips and placement errors do not accumulate.
        self.nominal_offset = {f: self.footholds[other(f)].actInv(self.footholds[f]) for f in FEET}
        self.sole_offset = {f: self.rm.sole_center(f) - self.rm.frame(f).translation for f in FEET}
        self.half_len = {f: self._sole_extent(f) for f in FEET}
        self.lean = None
        self.steps = 0
        self.stance = None              # stance foot during SS; next stance during DS
        self.next_stance = p.first_stance
        self._begin(DS, t, p.t_first_ds)
        self.p_prev = np.zeros(2)
        self.heading = 0.0              # walking-frame heading relative to the start (rad)
        self.step_turn = 0.0            # commanded turn of the current swing
        self.frame_turn = 0.0           # frame rotation applied at this lift-off
        self.turning = False            # once turned, the frame follows the feet
        self.yaw0 = {f: self._yaw(self.footholds[f]) for f in FEET}
        self.swing_yaw = 0.0            # yaw the current swing foot turns

    def _sole_extent(self, foot):
        pts = self.rm.sole_points(foot)
        c = pts.mean(axis=1)
        rel = pts - c[:, None]
        return (float(np.max(np.abs(rel.T @ self.lat_axis))), float(np.max(rel.T @ self.fwd_axis)),
                float(np.min(rel.T @ self.fwd_axis)))

    def _begin(self, phase, t, duration):
        self.gphase, self.t_gphase, self.t_dur = phase, t, duration

    def _coords(self, point):
        d = point - self.mid
        return np.array([d @ self.lat_axis, d @ self.fwd_axis])

    def _foothold_center(self, foot):
        return self._sole_center(foot, self.footholds[foot])

    def _sole_center(self, foot, pose):
        """Sole centre of `foot` placed at `pose`. sole_offset is the world
        vector at the start; once turning it is rotated with the foot's yaw
        (straight walking keeps the original, bit-identical behaviour)."""
        offset = self.sole_offset[foot]
        if getattr(self, 'turning', False):
            dyaw = self._yaw(pose) - self.yaw0[foot]
            offset = pin.rpy.rpyToMatrix(0.0, 0.0, dyaw) @ offset
        return pose.translation + offset

    # -------------------------------------------------------- schedule
    def _plan(self, t):
        """Support sequence over the MPC horizon: list of ('ds'|foot) per interval."""
        p = self.p
        seq = []
        phase, start, dur = self.gphase, self.t_gphase, self.t_dur
        stance = self.stance if self.gphase == SS else self.next_stance
        nxt = stance
        if self.stop_requested and phase == DS:
            phase, dur = HOLD, math.inf
        for k in range(p.mpc_horizon):
            tk = t + (k + 0.5) * p.mpc_dt
            while tk > start + dur:
                start += dur
                if phase == DS:
                    phase, dur = SS, p.t_ss
                else:
                    phase, dur, nxt = DS, p.t_ds, other(nxt)
                    if self.stop_requested:          # the current swing is the last one
                        phase, dur = HOLD, math.inf
            seq.append((phase, nxt))
        return seq

    def _bounds(self, seq):
        """CoP bounds and reference per interval, with previewed footholds.

        Every planned touchdown puts the swing foot at the nominal offset from
        the stance foot plus one step length forward, so the forward MPC sees
        where the next supports will be.
        """
        p = self.p
        n = len(seq)
        lo, hi, ref = np.zeros((n, 2)), np.zeros((n, 2)), np.zeros((n, 2))
        centers = {f: self._coords(self._foothold_center(f)) for f in FEET}
        if self.gphase == SS:
            # The swing foot will land at its target.
            swing = other(self.stance)
            centers[swing] = self._coords(self._sole_center(swing, self.swing_target))
        width = {f: centers[f][0] for f in FEET}
        prev_phase = seq[0][0]
        for k, (phase, foot) in enumerate(seq):
            if k and prev_phase == 'ds' and phase == 'ss':
                # A planned lift-off of other(foot): it lands one step ahead.
                swing = other(foot)
                centers[swing] = np.array([width[swing], centers[foot][1] + p.step_length_m])
            prev_phase = phase
            hl, toe, heel = self.half_len[foot]
            if phase == 'ss':
                c = centers[foot]
                lo[k] = [c[0] - hl + p.lat_margin_m, c[1] + heel + p.fwd_margin_m]
                hi[k] = [c[0] + hl - p.lat_margin_m, c[1] + toe - p.fwd_margin_m]
                ref[k] = c
            else:
                cs = np.array([centers[f] for f in FEET])
                lo[k] = [cs[:, 0].min() - hl + p.lat_margin_m, cs[:, 1].min() + heel + p.fwd_margin_m]
                hi[k] = [cs[:, 0].max() + hl - p.lat_margin_m, cs[:, 1].max() + toe - p.fwd_margin_m]
                # Halfway between the feet midpoint and the next stance foot;
                # standing: the feet midpoint.
                ref[k] = cs.mean(axis=0) if phase == HOLD else 0.5 * (centers[foot] + cs.mean(axis=0))
        return lo, hi, ref

    # ---------------------------------------------------------- active
    def _active(self, t, q, v, joints, rates):
        if self.p.activity != 'step':
            return super()._active(t, q, v, joints, rates)
        p = self.p
        com, vcom = self.rm.com.copy(), self.rm.vcom.copy()
        c2, v2 = self._coords(com), np.array([vcom @ self.lat_axis, vcom @ self.fwd_axis])
        forces = self.forces

        # ---- stand / resume requests
        if self._resume_requested:
            self._resume_requested = self.stop_requested = False
            if self.holding:
                self.holding = False
                self.lean = None                     # re-choose contact modes from the CoM side
                self._begin(DS, t, p.t_ds)           # full DS (with unloading) before lifting off
                self.events.append({'t': t, 'event': 'resume'})
        if self.stop_requested and self.gphase == DS and not self.holding:
            self.holding = True
            self.events.append({'t': t, 'event': 'hold'})

        # ---- phase transitions
        elapsed = t - self.t_gphase
        done = (bool(p.max_steps) and self.steps >= p.max_steps) or self.holding
        if self.gphase == DS and elapsed >= self.t_dur and not done:
            self._lift_off(t, q)
        elif self.gphase == SS:
            swing = other(self.stance)
            s = elapsed / self.t_dur
            if (s > 0.6 and forces[FEET.index(swing)] > p.touchdown_force_n) or \
               (s >= 1.0 and self._swing_height(swing) < 0.003):
                self._touchdown(t, q)
            elif elapsed > self.t_dur + p.touchdown_timeout_s:
                return self._fail('Swing foot did not touch down')

        # ---- contact modes in DS: both feet roll towards the CoM side
        if self.holding:
            # Standing: both feet flat at full force, CoM held over their middle.
            if self.lean != HOLD:
                for foot in FEET:
                    self.wbc.set_mode(foot, FLAT, q)
                    self.wbc.set_max_force(foot, p.gains.f_max)
                self.lean = HOLD
            self.wbc.set_com_mask(np.ones(3))
        elif self.gphase == DS:
            # The legs tilt towards where the CoM is going: lean by the CoM
            # position 80 ms ahead; from rest, towards the next stance foot.
            ahead = c2[0] + 0.08 * v2[0]
            toward = 'left' if self._coords(self._foothold_center(self.next_stance))[0] > 0 else 'right'
            lean = ('left' if ahead > p.lean_deadband_m else 'right' if ahead < -p.lean_deadband_m
                    else toward)
            if lean != self.lean:
                modes = ({FEET[0]: OUTER, FEET[1]: INNER} if lean == 'left'
                         else {FEET[0]: INNER, FEET[1]: OUTER})
                for foot, mode in modes.items():
                    self.wbc.set_mode(foot, mode, q)
                self.lean = lean
            self.wbc.set_com_mask(np.ones(3) if self.lean else self._lateral_mask())
            remaining = self.t_dur - (t - self.t_gphase)
            if remaining < p.unload_s and not done:
                self.wbc.set_max_force(other(self.next_stance),
                                       max(0.0, remaining / p.unload_s) * p.gains.f_max)
        else:
            self.wbc.set_com_mask(np.ones(3))
            self._swing_reference(t)

        # ---- MPC
        seq = self._plan(t)
        lo, hi, ref = self._bounds(seq)
        p_lat = self.mpc_lat.solve([c2[0], v2[0]], ref[:, 0], lo[:, 0], hi[:, 0],
                                   c_ref=np.zeros(p.mpc_horizon), p_prev=self.p_prev[0])
        p_fwd = self.mpc_fwd.solve([c2[1], v2[1]], ref[:, 1], lo[:, 1], hi[:, 1],
                                   c_ref=ref[:, 1], p_prev=self.p_prev[1])
        p0 = np.array([p_lat[0], p_fwd[0]])
        self.p_prev = p0
        acc2 = self.omega**2 * (c2 - p0)
        acc = acc2[0] * self.lat_axis + acc2[1] * self.fwd_axis
        ref_c = com.copy()
        ref_c[2] = self.z0
        vel = vcom.copy()
        vel[2] = 0.0
        self.wbc.set_com(ref_c, vel, acc)
        cop = self.mid + p0[0] * self.lat_axis + p0[1] * self.fwd_axis
        cop[2] = self.ground_z
        self.wbc.set_cop(cop)
        try:
            result = self.wbc.solve(q, v, t)
        except RuntimeError as exc:
            return self._fail(str(exc))
        self.status.update({'gphase': self.gphase, 'holding': self.holding, 'stance': self.stance,
                            'next': self.next_stance,
                            'lat': float(c2[0]), 'fwd': float(c2[1]), 'v_lat': float(v2[0]),
                            'cop_lat': float(p0[0]), 'lean': self.lean, 'steps': self.steps,
                            'forces': result.forces, 'com_z': float(com[2])})
        return self._sample_command(result, joints, rates)

    # ------------------------------------------------------- events
    def _lift_off(self, t, q):
        p = self.p
        stance = self.next_stance
        swing = other(stance)
        self.wbc.set_mode(swing, SWING, q)
        # Stance foot flat: drop its roll/pitch from the contact reference.
        self.wbc.set_mode(stance, FLAT, q)
        flat = self.rm.data.oMf[self.rm.frame_id[stance]].copy()
        yaw = math.atan2(flat.rotation[1, 0], flat.rotation[0, 0])
        flat.rotation = pin.rpy.rpyToMatrix(0.0, 0.0, yaw) @ self._foot_rest_rotation(stance)
        self.wbc.contact_objects[self.wbc.contacts[stance][0]].setReference(flat)
        self.wbc.set_max_force(stance, p.gains.f_max)
        self.lean = None
        self.stance = stance
        self.swing_start = self.rm.data.oMf[self.rm.frame_id[swing]].copy()
        self.step_turn = float(p.turn_rad)
        self.frame_turn = 0.0
        self.turning = self.turning or bool(self.step_turn)
        if self.turning:
            # Closed loop on the measured stance foot: after this step the feet
            # are at its yaw and yaw + turn, so the frame takes the mean.
            cop = self.mid + self.p_prev[0] * self.lat_axis + self.p_prev[1] * self.fwd_axis
            measured = self._wrap(self._yaw(self.rm.frame(stance)) - self.yaw0[stance])
            self.frame_turn = self._wrap(measured + 0.5 * self.step_turn - self.heading)
            self._turn_frame(self.frame_turn, self._foothold_center(stance))
        # Target: the measured stance foot, made level, plus the nominal offset
        # and the commanded step length along the walking direction.
        stance_pose = self.rm.data.oMf[self.rm.frame_id[stance]].copy()
        yaw = math.atan2(stance_pose.rotation[1, 0], stance_pose.rotation[0, 0])
        level = pin.SE3(pin.rpy.rpyToMatrix(0.0, 0.0, yaw) @ self._foot_rest_rotation(stance),
                        stance_pose.translation.copy())
        level.translation[2] = self.footholds[stance].translation[2]
        target = level * self.nominal_offset[swing]
        if self.step_turn:
            # Rotate the nominal placement about the stance foot's vertical axis.
            Rz = pin.rpy.rpyToMatrix(0.0, 0.0, self.step_turn)
            pivot = level.translation
            target = pin.SE3(Rz, pivot - Rz @ pivot) * target
        target.translation = target.translation + p.step_length_m * self.fwd_axis
        self.swing_target = target
        if self.turning:
            self.p_prev = self._coords(cop)
        # Yaw the swing foot has to turn: from where it starts (feet splayed by
        # the last turn) to the target; 0 for straight walking.
        self.swing_yaw = (self._wrap(self._yaw(target) - self._yaw(self.swing_start))
                          if self.turning else 0.0)
        self.events.append({'t': t, 'event': 'lift', 'stance': stance})
        self._begin(SS, t, p.t_ss)

    def _turn_frame(self, angle, pivot):
        """Rotate the walking frame (MPC axes and origin) by `angle` about the
        vertical through `pivot`. The caller re-expresses the previous CoP."""
        Rz = pin.rpy.rpyToMatrix(0.0, 0.0, angle)
        piv = np.array([pivot[0], pivot[1], self.mid[2]])
        self.mid = piv + Rz @ (self.mid - piv)
        self.lat_axis, self.fwd_axis = Rz @ self.lat_axis, Rz @ self.fwd_axis
        self.half_len = {f: self._sole_extent(f) for f in FEET}
        self.heading += angle

    @staticmethod
    def _yaw(pose):
        return math.atan2(pose.rotation[1, 0], pose.rotation[0, 0])

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _foot_rest_rotation(self, foot):
        R = self.footholds[foot].rotation
        yaw = math.atan2(R[1, 0], R[0, 0])
        return pin.rpy.rpyToMatrix(0.0, 0.0, yaw).T @ R

    def _touchdown(self, t, q):
        swing = other(self.stance)
        landed = self.rm.data.oMf[self.rm.frame_id[swing]].copy()
        self.footholds[swing] = landed
        self.steps += 1
        self.events.append({'t': t, 'event': 'touchdown', 'foot': swing,
                            'height': self._swing_height(swing)})
        # Both feet flat until the lean is chosen again from the CoM side.
        self.wbc.set_mode(swing, FLAT, q)
        self.wbc.set_mode(self.stance, FLAT, q)
        self.lean = None
        self.next_stance = swing
        self._begin(DS, t, self.p.t_ds)

    def _swing_height(self, foot):
        return float(self.rm.sole_points(foot)[2].min() - self.ground_z)

    def _swing_reference(self, t):
        p = self.p
        swing = other(self.stance)
        T = self.t_dur
        s = min(max((t - self.t_gphase) / T, 0.0), 1.0)
        a, b = self.swing_start.translation, self.swing_target.translation
        # Horizontal: min-jerk over the first swing_fwd_end of the phase.
        Th = T * p.swing_fwd_end
        mh, dmh, ddmh = min_jerk(min(s / p.swing_fwd_end, 1.0))
        pos = a + mh * (b - a)
        vel = dmh / Th * (b - a)
        acc = ddmh / Th**2 * (b - a)
        m, dm, ddm = min_jerk(s)
        pos[2] = a[2] + m * (b[2] - a[2]) + p.lift_m * math.sin(math.pi * s)
        vel[2] = dm / T * (b[2] - a[2]) + p.lift_m * math.pi * math.cos(math.pi * s) / T
        acc[2] = ddm / T**2 * (b[2] - a[2]) - p.lift_m * math.pi**2 * math.sin(math.pi * s) / T**2
        if s >= 1.0:
            # Late: keep lowering slowly until contact.
            late = t - self.t_gphase - T
            pos[2] = b[2] - 0.05 * late
            vel[2] = -0.05
        # Yaw from the start to the target (only once turning, so a straight
        # step keeps the exact target orientation).
        turn = self.swing_yaw
        Ty = T * p.swing_yaw_end
        my, dmy, ddmy = min_jerk(min(s / p.swing_yaw_end, 1.0))
        rot = pin.rpy.rpyToMatrix(0.0, 0.0, -(1.0 - my) * turn) @ self.swing_target.rotation
        ang_vel = np.array([0.0, 0.0, dmy / Ty * turn])
        ang_acc = np.array([0.0, 0.0, ddmy / Ty**2 * turn])
        pose = pin.SE3(rot, pos)
        self.wbc.set_swing_reference(swing, pose, np.concatenate([vel, ang_vel]),
                                     np.concatenate([acc, ang_acc]))
        if self.turning:
            # Torso yaw from the previous frame heading to the new one with the swing.
            self.wbc.set_torso_yaw(self.yaw + self.heading - (1.0 - mh) * self.frame_turn)
