"""ROS-independent simulated motor bank with the CAN backend's command rules.

The real backend (``motor_control_hybrid/python_can_node.py``) is the reference
for which commands are admitted, when they expire and how estop behaves. This
module reproduces those rules in joint coordinates and converts the active
command of each joint into an MIT-style torque law for MuJoCo:

    tau = kp * (q_ref - q) + kd * (dq_ref - dq) + tau_ff

The law is evaluated by MuJoCo at every physics step (see ``world.py``), which
approximates the motor driver's own high-rate PD loop. It is a behavioral model
of the command interface, not an identified actuator model: there is no current
loop, friction model, backlash, CAN latency or thermal derating.

All times are host monotonic seconds supplied by the caller, so the rules can be
unit tested without ROS or wall-clock sleeps.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

# MotorCommand.msg mode constants. Duplicated so this module needs no ROS import.
MODE_VELOCITY = 0
MODE_POSITION = 1
MODE_MOTION = 2
MODE_ENABLE = 3
MODE_DISABLE = 4

# Peak torques by registry model when max_torque_nm is null (RobStride datasheets).
DEFAULT_PEAK_TORQUE_NM = {'rs-02': 17.0, 'rs-03': 60.0, 'rs-04': 120.0}
FALLBACK_PEAK_TORQUE_NM = 60.0


@dataclass
class JointLimits:
    """Per-joint settings read from the motor registry, in joint coordinates."""

    lower: float
    upper: float
    kp: float
    kd: float
    max_torque: float
    max_velocity: float


@dataclass
class MotorSetpoint:
    """Torque-law inputs for one physics step. Zero gains mean a limp motor."""

    kp: float = 0.0
    kd: float = 0.0
    q_ref: float = 0.0
    dq_ref: float = 0.0
    tau_ff: float = 0.0


@dataclass
class _JointState:
    enabled: bool = False
    # Set by a command timeout; only an explicit ENABLE clears it.
    inhibited: bool = False
    mode: int = MODE_DISABLE
    deadline: float = math.inf
    kp: float = 0.0
    kd: float = 0.0
    q_target: float = 0.0
    dq_target: float = 0.0
    tau_ff: float = 0.0
    # POSITION mode moves this reference toward q_target at position_speed.
    q_ref: float = 0.0
    position_speed: float = 0.0
    events: list = field(default_factory=list)


def limits_from_registry(params: dict) -> dict[str, JointLimits]:
    """Build joint limits from the ``ros__parameters`` block of motors.yaml.

    Unlike the CAN backend this does not require ``hardware_verified``: no
    hardware is touched in simulation.
    """
    motors = params.get('motors') or {}
    if not motors:
        raise ValueError('Motor registry is empty')
    max_velocity = float(params.get('MAX_VEL_RAD_S', 4.5))
    limits = {}
    for name, cfg in motors.items():
        torque = cfg.get('max_torque_nm')
        if torque is None:
            torque = DEFAULT_PEAK_TORQUE_NM.get(str(cfg.get('model')), FALLBACK_PEAK_TORQUE_NM)
        limits[name] = JointLimits(
            lower=float(cfg.get('min_position', -math.inf)),
            upper=float(cfg.get('max_position', math.inf)),
            kp=float(cfg.get('kp', 0.0)),
            kd=float(cfg.get('kd', 0.0)),
            max_torque=float(torque),
            max_velocity=float(cfg.get('max_vel_rad_s', max_velocity)),
        )
    return limits


class MotorBank:
    """Command admission, expiry and estop for a set of simulated motors.

    Mirrors python_can_node:
      * Non-disable commands need a valid source stamp no older than
        ``command_timeout_s`` and, when set, an unexpired ``valid_until``.
      * Each admitted command sets a deadline; on expiry the joint is disabled
        and inhibited until a fresh explicit ENABLE.
      * Estop disables every joint and drops commands; clearing estop does not
        re-enable anything.
      * Motion commands to disabled joints are ignored.
    """

    def __init__(self, limits: dict[str, JointLimits], command_timeout_s: float = 0.5,
                 velocity_mode_kd: float = 2.0) -> None:
        if command_timeout_s <= 0.0:
            raise ValueError('command_timeout_s must be positive')
        self.limits = dict(limits)
        self.joint_names = list(limits)
        self.command_timeout_s = float(command_timeout_s)
        self.velocity_mode_kd = float(velocity_mode_kd)
        self.estop_active = False
        self._joints = {name: _JointState() for name in self.joint_names}

    # ------------------------------------------------------------------ state
    def is_enabled(self, joint: str) -> bool:
        return self._joints[joint].enabled

    def mode(self, joint: str) -> int:
        return self._joints[joint].mode

    def pop_events(self) -> list[str]:
        """Return and clear human-readable state changes for logging."""
        events = []
        for name in self.joint_names:
            events.extend(self._joints[name].events)
            self._joints[name].events.clear()
        return events

    # ---------------------------------------------------------------- inputs
    def set_estop(self, active: bool) -> None:
        self.estop_active = bool(active)
        if self.estop_active:
            for name in self.joint_names:
                self.disable(name)

    def submit(self, joint_names, modes, positions, velocities, torques, kps, kds,
               stamp_s: float, valid_until_s: float, now_ros_s: float,
               now_mono_s: float) -> list[str]:
        """Admit one MotorCommand. Returns joints rejected for a stale stamp or non-finite values.

        Args:
            joint_names: MotorCommand.joint_name.
            modes: One broadcast mode, one mode per joint, or empty (velocity).
            positions, velocities, torques, kps, kds: Per-joint arrays; missing
                kp/kd entries fall back to the registry value.
            stamp_s: header.stamp in ROS seconds.
            valid_until_s: valid_until in ROS seconds, 0 for none.
            now_ros_s: Current ROS time in seconds.
            now_mono_s: Current host monotonic time in seconds.
        """
        if self.estop_active or not joint_names:
            return []
        age = now_ros_s - stamp_s
        source_valid = stamp_s > 0 and -0.01 <= age < self.command_timeout_s
        deadline = now_mono_s + self.command_timeout_s - max(0.0, age)
        if valid_until_s != 0:
            source_valid = source_valid and valid_until_s > now_ros_s and valid_until_s > stamp_s
            deadline = min(deadline, now_mono_s + valid_until_s - now_ros_s)

        if len(modes) == 1:
            modes = [modes[0]] * len(joint_names)
        elif len(modes) == 0:
            modes = [MODE_VELOCITY] * len(joint_names)

        rejected = []
        for i, joint in enumerate(joint_names):
            if joint not in self._joints:
                continue
            mode = int(modes[i]) if i < len(modes) else MODE_VELOCITY
            if mode == MODE_DISABLE:
                self.disable(joint)
                continue
            if not source_valid:
                rejected.append(joint)
                continue
            values = [_value_at(array, i, 0.0) for array in (positions, velocities, torques, kps, kds)]
            if not all(math.isfinite(value) for value in values):
                rejected.append(joint)
                continue
            limits = self.limits[joint]
            state = self._joints[joint]
            if mode == MODE_ENABLE:
                if not state.enabled:
                    state.events.append(f'Enabled {joint}')
                state.enabled = True
                state.inhibited = False
                # The real driver enables in an idle, zero-gain state.
                state.mode = MODE_MOTION
                state.kp = state.kd = state.tau_ff = state.dq_target = 0.0
                state.deadline = deadline
                continue
            if not state.enabled or state.inhibited:
                continue

            state.mode = mode
            state.deadline = deadline
            position = _value_at(positions, i, state.q_target)
            velocity = _value_at(velocities, i, 0.0)
            state.kp = max(0.0, _value_at(kps, i, limits.kp))
            state.kd = max(0.0, _value_at(kds, i, limits.kd))
            if mode == MODE_VELOCITY:
                state.dq_target = _clip(velocity, -limits.max_velocity, limits.max_velocity)
                state.tau_ff = 0.0
            elif mode == MODE_POSITION:
                state.q_target = _clip(position, limits.lower, limits.upper)
                speed = abs(velocity) if velocity else 0.5
                state.position_speed = min(speed, limits.max_velocity)
                state.dq_target = 0.0
                state.tau_ff = 0.0
            elif mode == MODE_MOTION:
                state.q_target = _clip(position, limits.lower, limits.upper)
                state.q_ref = state.q_target
                state.dq_target = _clip(velocity, -limits.max_velocity, limits.max_velocity)
                state.tau_ff = _clip(_value_at(torques, i, 0.0),
                                     -limits.max_torque, limits.max_torque)
        return rejected

    # ----------------------------------------------------------------- update
    def expire(self, now_mono_s: float) -> None:
        """Disable and inhibit joints whose command deadline has passed."""
        for name in self.joint_names:
            state = self._joints[name]
            if state.enabled and now_mono_s >= state.deadline:
                self.disable(name)
                state.inhibited = True
                state.events.append(f'Command timeout for {name}; explicit enable required')

    def setpoint(self, joint: str, q: float, dt: float) -> MotorSetpoint:
        """Return the torque-law inputs for one physics step of length dt."""
        state = self._joints[joint]
        if not state.enabled:
            return MotorSetpoint()
        if state.mode == MODE_VELOCITY:
            kd = state.kd if state.kd > 0.0 else self.velocity_mode_kd
            return MotorSetpoint(kp=0.0, kd=kd, q_ref=q, dq_ref=state.dq_target)
        if state.mode == MODE_POSITION:
            error = state.q_target - state.q_ref
            step = min(abs(error), state.position_speed * dt)
            state.q_ref += math.copysign(step, error)
            return MotorSetpoint(kp=state.kp, kd=state.kd, q_ref=state.q_ref)
        return MotorSetpoint(kp=state.kp, kd=state.kd, q_ref=state.q_target,
                             dq_ref=state.dq_target, tau_ff=state.tau_ff)

    def seed_reference(self, joint: str, q: float) -> None:
        """Start POSITION-mode ramps from the measured position, not zero."""
        state = self._joints[joint]
        if state.mode != MODE_POSITION:
            state.q_ref = q

    def disable(self, joint: str) -> None:
        state = self._joints[joint]
        if state.enabled:
            state.events.append(f'Disabled {joint}')
        state.enabled = False
        state.mode = MODE_DISABLE
        state.deadline = math.inf
        state.kp = state.kd = state.tau_ff = state.dq_target = 0.0


def _value_at(values, index: int, default: float) -> float:
    if index < len(values):
        return float(values[index])
    return default


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
