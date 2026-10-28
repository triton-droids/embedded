"""Thread-safe MuJoCo world shared by the simulated peripheral nodes.

One physics thread steps the model at real-time pace and wall-clock ROS time is
used throughout (no /clock, no use_sim_time). The deployed nodes measure input
age, deadlines and IMU timeouts against host time, so running the simulation in
real time lets them run unmodified. If stepping falls behind, simulated time
slows down rather than skipping physics; ``overruns`` counts these events.

Every access to ``data`` must hold ``lock``. Callers should copy what they need
and release it quickly; rendering happens outside the lock.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time

import mujoco
import numpy as np

from . import model_builder as mb
from .motor_model import MotorBank


@dataclass
class HeadServoConfig:
    """Hobby-servo model: rate-limited setpoint tracked by a stiff PD loop."""

    kp: float = 20.0
    kd: float = 0.3
    max_speed_rad_s: float = 6.0
    min_angle_rad: float = -1.0
    max_angle_rad: float = 1.0


@dataclass
class TargetMotion:
    """Side-to-side sweep of the mocap face target; amplitude 0 keeps it still."""

    amplitude_m: float = 0.4
    period_s: float = 8.0


@dataclass
class JointSnapshot:
    names: list[str]
    position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray


@dataclass
class ImuSnapshot:
    gyro: np.ndarray  # rad/s, sensor frame
    accel: np.ndarray  # m/s^2 specific force, sensor frame (+Z up at rest)
    quat_wxyz: np.ndarray | None  # sensor-to-world, None when unavailable


class SimWorld:
    """Owns MjModel/MjData, the motor bank and the physics thread."""

    def __init__(self, model: mujoco.MjModel, motors: MotorBank, *,
                 head: HeadServoConfig | None = None,
                 target_motion: TargetMotion | None = None,
                 base_held: bool = True, settle_s: float = 2.0) -> None:
        self.model = model
        self.data = mujoco.MjData(model)
        self.motors = motors
        self.lock = threading.RLock()
        self.overruns = 0
        self.steps = 0
        self.settle_s = float(settle_s)

        self.joint_names = list(motors.joint_names)
        self._qpos = np.array([model.joint(n).qposadr[0] for n in self.joint_names])
        self._qvel = np.array([model.joint(n).dofadr[0] for n in self.joint_names])
        self._act = np.array([model.actuator(mb.actuator_name(n)).id for n in self.joint_names])
        self._weld = model.equality(mb.BASE_WELD).id
        self._base_joint = model.body(mb.BASE_BODY).jntadr[0]

        self.head = head
        self._head_target: float | None = None
        self._head_ref = 0.0
        if head is not None:
            self._head_act = model.actuator(mb.HEAD_ACTUATOR).id
            self._head_qpos = model.joint(mb.HEAD_JOINT).qposadr[0]
            self._head_qvel = model.joint(mb.HEAD_JOINT).dofadr[0]
        self.target_motion = target_motion
        self._target_mocap = None
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, mb.TARGET_BODY) >= 0:
            self._target_mocap = model.body(mb.TARGET_BODY).mocapid[0]
            self._target_home = model.body(mb.TARGET_BODY).pos.copy()

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.reset(base_held=base_held)

    # ------------------------------------------------------------ lifecycle
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name='mujoco_physics', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        dt = self.model.opt.timestep
        next_step = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_step:
                time.sleep(next_step - now)
                continue
            self.step(now)
            next_step += dt
            if time.monotonic() - next_step > 0.1:
                # More than 100 ms behind: slow simulated time instead of bursting.
                self.overruns += 1
                next_step = time.monotonic()

    # -------------------------------------------------------------- physics
    def step(self, now_mono: float) -> None:
        model, data = self.model, self.data
        dt = model.opt.timestep
        with self.lock:
            self.motors.expire(now_mono)
            for i, joint in enumerate(self.joint_names):
                q = float(data.qpos[self._qpos[i]])
                self.motors.seed_reference(joint, q)
                sp = self.motors.setpoint(joint, q, dt)
                act = self._act[i]
                # force = ctrl + bias1*q + bias2*qdot = kp*(q_ref-q) + kd*(dq_ref-qdot) + tau_ff
                data.ctrl[act] = sp.kp * sp.q_ref + sp.kd * sp.dq_ref + sp.tau_ff
                model.actuator_biasprm[act, 1] = -sp.kp
                model.actuator_biasprm[act, 2] = -sp.kd
            if self.head is not None:
                self._apply_head(dt)
            if self._target_mocap is not None and self.target_motion is not None:
                self._move_target()
            mujoco.mj_step(model, data)
            self.steps += 1

    def _apply_head(self, dt: float) -> None:
        cfg, act = self.head, self._head_act
        if self._head_target is None:
            # Servo released (PWM off): limp, and restart ramps from the measured angle.
            self._head_ref = float(self.data.qpos[self._head_qpos])
            self.data.ctrl[act] = 0.0
            self.model.actuator_biasprm[act, 1:3] = 0.0
            return
        error = self._head_target - self._head_ref
        step = min(abs(error), cfg.max_speed_rad_s * dt)
        self._head_ref += math.copysign(step, error)
        self.data.ctrl[act] = cfg.kp * self._head_ref
        self.model.actuator_biasprm[act, 1] = -cfg.kp
        self.model.actuator_biasprm[act, 2] = -cfg.kd

    def _move_target(self) -> None:
        motion = self.target_motion
        offset = 0.0
        if motion.amplitude_m and motion.period_s > 0.0:
            offset = motion.amplitude_m * math.sin(2.0 * math.pi * self.data.time / motion.period_s)
        self.data.mocap_pos[self._target_mocap] = self._target_home + np.array([offset, 0.0, 0.0])

    # ------------------------------------------------------------- controls
    def reset(self, base_held: bool = True) -> None:
        """Return to the spawn pose with all motors disabled.

        The limp legs swing for about a second after spawning. Physics is
        pre-rolled for ``settle_s`` so the nodes first see a robot at rest,
        which the policy's gyro calibration requires.
        """
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            for joint in self.joint_names:
                self.motors.disable(joint)
            self._head_target = None
            self.data.eq_active[self._weld] = 1 if base_held else 0
            mujoco.mj_forward(self.model, self.data)
            start = time.monotonic()
            for _ in range(int(self.settle_s / self.model.opt.timestep)):
                self.step(start)

    def set_base_held(self, held: bool) -> None:
        """Engage or release the virtual gantry.

        Engaging first moves the base back to its spawn pose at rest so the weld
        does not yank the robot across the scene; joint angles are kept.
        """
        with self.lock:
            if held:
                adr = self.model.jnt_qposadr[self._base_joint]
                dof = self.model.jnt_dofadr[self._base_joint]
                self.data.qpos[adr:adr + 7] = self.model.qpos0[adr:adr + 7]
                self.data.qvel[dof:dof + 6] = 0.0
            self.data.eq_active[self._weld] = 1 if held else 0
            mujoco.mj_forward(self.model, self.data)

    def base_held(self) -> bool:
        with self.lock:
            return bool(self.data.eq_active[self._weld])

    def set_head_target(self, angle: float | None) -> None:
        """Command the head servo angle in rad, or None to release it."""
        with self.lock:
            if angle is not None and self.head is not None:
                angle = min(max(angle, self.head.min_angle_rad), self.head.max_angle_rad)
            self._head_target = angle

    # ------------------------------------------------------------ snapshots
    def joint_snapshot(self) -> JointSnapshot:
        with self.lock:
            return JointSnapshot(
                names=list(self.joint_names),
                position=self.data.qpos[self._qpos].copy(),
                velocity=self.data.qvel[self._qvel].copy(),
                effort=self.data.actuator_force[self._act].copy(),
            )

    def body_imu(self) -> ImuSnapshot:
        with self.lock:
            return ImuSnapshot(gyro=self.data.sensor(mb.IMU_GYRO).data.copy(),
                               accel=self.data.sensor(mb.IMU_ACCEL).data.copy(),
                               quat_wxyz=self.data.sensor(mb.IMU_QUAT).data.copy())

    def head_imu(self) -> ImuSnapshot:
        with self.lock:
            return ImuSnapshot(gyro=self.data.sensor(mb.HEAD_GYRO).data.copy(),
                               accel=self.data.sensor(mb.HEAD_ACCEL).data.copy(),
                               quat_wxyz=None)

    def head_angle(self) -> float:
        with self.lock:
            return float(self.data.qpos[self._head_qpos])

    def base_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Ground-truth base position, orientation (wxyz), linear and angular velocity.

        Velocities are in the world frame (MuJoCo free-joint linear velocity is
        world-frame; angular velocity is body-frame and rotated here).
        """
        with self.lock:
            adr = self.model.jnt_qposadr[self._base_joint]
            dof = self.model.jnt_dofadr[self._base_joint]
            pos = self.data.qpos[adr:adr + 3].copy()
            quat = self.data.qpos[adr + 3:adr + 7].copy()
            lin = self.data.qvel[dof:dof + 3].copy()
            ang_body = self.data.qvel[dof + 3:dof + 6].copy()
        ang = np.zeros(3)
        mujoco.mju_rotVecQuat(ang, ang_body, quat)
        return pos, quat, lin, ang
