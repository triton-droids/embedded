"""Validate hardware mappings and convert calibrated joint/motor coordinates."""
from dataclasses import dataclass
import math
from .ankle_mapping import AnkleMapper

# RobStride product pages (48 V), same numbers as humanoid_sim/motor_model.py:
# peak torque (N.m) and output no-load speed (rad/s).
MODEL_LIMITS = {
    'rs-02': (17.0, 410 * 2 * math.pi / 60),
    'rs-03': (60.0, 195 * 2 * math.pi / 60),
    'rs-04': (120.0, 200 * 2 * math.pi / 60),
}
# MotorCommand.msg modes whose MIT frame carries a position target.
POSITION_MODES = (1, 2)  # MODE_POSITION, MODE_MOTION
# A feedback step larger than speed * 1.5 * dt + this margin cannot be motion.
FEEDBACK_JUMP_MARGIN_RAD = 0.05
# Feedback samples further apart than this are not compared for jumps.
FEEDBACK_JUMP_WINDOW_S = 0.1
# Motor-side CAN watchdog (RobStride manual, parameter table and section
# "CAN communication failure protection"): canTimeout 0x7028, uint32,
# 20000 = 1 s, 0 = off; a motor that receives no CAN command for that long
# enters reset mode (unpowered). The setting copy (0x200C) is capped at 100000.
CAN_TIMEOUT_INDEX = 0x7028
CAN_TIMEOUT_TICKS_PER_S = 20000
CAN_TIMEOUT_MAX_S = 5.0
# limit_torque (0x700B) and limit_cur (0x7018) are deliberately not written:
# the manual documents limit_cur for velocity/CSP modes only, does not say that
# limit_torque bounds the operation-control (MIT) torque, and asks users not to
# change the torque limit.


def firmware_writes(cfg, defaults):
    """Validated ((index, kind, name, raw_value),) written and read back at ENABLE.

    firmware_can_timeout_s (per motor, else FIRMWARE_CAN_TIMEOUT_S; null = do not
    touch the motor's own value).
    """
    seconds = cfg.get('firmware_can_timeout_s', defaults.get('FIRMWARE_CAN_TIMEOUT_S'))
    if seconds is None:
        return ()
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
        raise ValueError('firmware_can_timeout_s must be a number of seconds')
    seconds = float(seconds)
    if not (math.isfinite(seconds) and 0 < seconds <= CAN_TIMEOUT_MAX_S):
        raise ValueError(f'firmware_can_timeout_s must be in (0, {CAN_TIMEOUT_MAX_S}]')
    return ((CAN_TIMEOUT_INDEX, 'u32', 'canTimeout', round(seconds * CAN_TIMEOUT_TICKS_PER_S)),)


def parameter_payload(kind, value):
    """Bytes 4..7 of a type-17/18 parameter frame for this value."""
    import struct
    return struct.pack('<L', int(value)) if kind == 'u32' else struct.pack('<f', float(value))


@dataclass(frozen=True)
class MotorSettings:
    direction: int
    offset: float
    lower: float
    upper: float
    kp: float
    kd: float
    max_velocity: float
    max_torque: float | None

    ankle: object = None
    # Gain ceilings; requests above them are reduced, never rejected.
    max_kp: float = math.inf
    max_kd: float = math.inf
    # Largest |target - measured| in motor coordinates a position frame may ask for.
    max_position_error: float = math.inf
    # Fastest physically possible motor output speed, for feedback-jump detection.
    max_motor_speed: float = math.inf
    # Parameter writes applied and verified at ENABLE, see firmware_writes().
    firmware_writes: tuple = ()

    @classmethod
    def from_config(cls, cfg, defaults):
        direction = cfg.get('direction', 1)
        if isinstance(direction, bool) or direction not in (-1, 1):
            raise ValueError('direction must be +1 or -1')
        peak, no_load = MODEL_LIMITS.get(cfg.get('model'), (None, None))
        kp = float(cfg.get('kp', defaults.get('KP', 10.0)))
        kd = float(cfg.get('kd', defaults.get('KD', 0.2)))
        max_torque = float(cfg['max_torque_nm']) if cfg.get('max_torque_nm') is not None else None
        settings = cls(
            int(direction), float(cfg.get('encoder_offset_rad', 0.0)),
            float(cfg.get('min_position', -math.inf)), float(cfg.get('max_position', math.inf)),
            kp, kd,
            float(cfg.get('max_vel_rad_s', defaults.get('MAX_VEL_RAD_S', 4.5))),
            max_torque,
            AnkleCalibration(cfg['ankle_mapping']) if cfg.get('ankle_mapping') else None,
            float(cfg.get('max_kp', defaults.get('MAX_KP', kp))),
            float(cfg.get('max_kd', defaults.get('MAX_KD', kd))),
            float(cfg.get('max_position_error_rad',
                          defaults.get('MAX_POSITION_ERROR_RAD', math.inf))),
            float(cfg.get('max_motor_speed_rad_s', no_load if no_load is not None else math.inf)),
            firmware_writes(cfg, defaults),
        )
        if (settings.direction not in (-1, 1) or not math.isfinite(settings.offset)
                or math.isnan(settings.lower) or math.isnan(settings.upper)
                or settings.lower >= settings.upper
                or not all(math.isfinite(x) and x >= 0 for x in
                           (settings.kp, settings.kd, settings.max_velocity))
                or (settings.max_torque is not None and
                    (not math.isfinite(settings.max_torque) or settings.max_torque <= 0))
                or not all(x > 0 and not math.isnan(x) for x in
                           (settings.max_kp, settings.max_kd, settings.max_position_error,
                            settings.max_motor_speed))):
            raise ValueError('Invalid motor calibration, limits or gains')
        hard_lo, hard_hi = cfg.get('hard_min_position'), cfg.get('hard_max_position')
        if ((hard_lo is not None and settings.lower < float(hard_lo))
                or (hard_hi is not None and settings.upper > float(hard_hi))):
            raise ValueError(f'min/max_position [{settings.lower}, {settings.upper}] leave the '
                             f'mechanical range [{hard_lo}, {hard_hi}]')
        if peak is not None and settings.max_torque is not None and settings.max_torque > peak:
            raise ValueError(f'max_torque_nm {settings.max_torque} exceeds the '
                             f'{cfg.get("model")} peak {peak}')
        return settings

    def check_feedback_step(self, previous, current):
        """Raise when raw motor position moved faster than the motor can.

        previous/current are (raw_position_rad, host_monotonic_s). A jump means
        the coordinate system changed under us (external mechanical-zero write,
        encoder fault, wrong motor answering), not motion.
        """
        (p0, t0), (p1, t1) = previous, current
        dt = t1 - t0
        if not 0.0 <= dt <= FEEDBACK_JUMP_WINDOW_S or not math.isfinite(self.max_motor_speed):
            return
        limit = self.max_motor_speed * 1.5 * dt + FEEDBACK_JUMP_MARGIN_RAD
        if abs(p1 - p0) > limit:
            raise ValueError(f'feedback position jumped {p1 - p0:+.3f} rad in {dt * 1000:.1f} ms '
                             f'(limit {limit:.3f}); coordinate change or encoder fault')

    def feedback(self, position, velocity, torque):
        if not all(math.isfinite(x) for x in (position, velocity, torque)):
            raise ValueError('Non-finite motor feedback')
        if self.ankle is not None:
            return self.ankle.feedback(self.direction * (position - self.offset), self.direction * velocity, self.direction * torque)
        return (self.direction * (position - self.offset),
                self.direction * velocity, self.direction * torque)

    def command(self, command, measured_motor=None):
        """Joint target -> clamped motor target.

        measured_motor is the latest fresh raw motor position; a position-mode
        target is kept within max_position_error of it, a coarse guard against
        a stale or re-zeroed coordinate. It must stay well above the errors the
        policy uses deliberately (the ankles hold 0.3-1.2 rad in simulation),
        and it is not a torque budget: clipping to (max_torque - |ff|) / kp
        made the simulated robot fall.
        Without a fresh measurement the frame becomes a zero-gain, zero-torque
        probe: RobStride reports status only in reply to a command, so refusing
        to send would never produce the measurement.
        """
        result = dict(command)
        values = [result[key] for key in ('position', 'velocity', 'acceleration', 'torque', 'kp', 'kd')]
        if not all(math.isfinite(x) for x in values) or result['kp'] < 0 or result['kd'] < 0:
            raise ValueError('Non-finite command or negative gains')
        position = min(max(result['position'], self.lower), self.upper)
        velocity = min(max(result['velocity'], -self.max_velocity), self.max_velocity)
        torque = result['torque']
        if self.max_torque is not None:
            torque = min(max(torque, -self.max_torque), self.max_torque)
        if self.ankle is not None:
            position = self.ankle.command(position)
        kp = min(result['kp'], self.max_kp)
        kd = min(result['kd'], self.max_kd)
        motor_position = self.offset + self.direction * position
        if result.get('mode') in POSITION_MODES:
            if measured_motor is None or not math.isfinite(measured_motor):
                result.update(position=motor_position, kp=0.0, kd=0.0, velocity=0.0,
                              torque=0.0, probe=True)
                return result
            error = self.max_position_error
            motor_position = min(max(motor_position, measured_motor - error),
                                 measured_motor + error)
        result.update(position=motor_position, kp=kp, kd=kd,
                      velocity=self.direction * velocity, torque=self.direction * torque)
        return result


def validate_registry(params):
    """Validate the entire registry before opening any CAN bus."""
    if params.get('hardware_verified') is False:
        raise ValueError('Motor registry is not hardware verified: verify CAN IDs, '
                         'models, directions and offsets before enabling hardware_verified')
    motors = params.get('motors', {})
    if not motors:
        raise ValueError('Motor registry is empty')
    expected = params.get('model_contract', {}).get('joint_order')
    if expected is not None and list(motors) != expected:
        raise ValueError('Motor registry order does not match the tracking contract')
    seen = set()
    masters = {}
    settings = {}
    for name, cfg in motors.items():
        motor_id = cfg.get('motor_id')
        interface = cfg.get('can_interface', params.get('default_can_interface', 'can1'))
        if (isinstance(motor_id, bool) or not isinstance(motor_id, int)
                or not 1 <= motor_id < 255 or not interface or not cfg.get('model')):
            raise ValueError(f'{name}: valid motor_id, can_interface and model are required')
        key = (interface, motor_id)
        if key in seen:
            raise ValueError(f'Duplicate CAN motor mapping: {key}')
        seen.add(key)
        master = cfg.get('master_id', params.get('default_master_id', 255))
        if (isinstance(master, bool) or not isinstance(master, int)
                or not 1 <= master <= 255 or master == motor_id):
            raise ValueError(f'{name}: invalid master_id')
        if interface in masters and masters[interface] != master:
            raise ValueError(f'{interface}: all motors must share a master_id')
        masters[interface] = master
        try:
            settings[name] = MotorSettings.from_config(cfg, params)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{name}: invalid motor configuration: {exc}') from exc
    return settings


class AnkleCalibration:
    """Keep solver guesses continuous, matching the source branch linkage convention."""
    def __init__(self, config):
        self.mapper = AnkleMapper(config)
        self.motor = 0.0
        self.joint = 0.0
        self.last_time = None

    def feedback(self, position, velocity, torque):
        import time
        now = time.monotonic()
        joint = self.mapper.motor_logical_rad_to_ankle_rad(position, self.joint)
        dt = 0 if self.last_time is None else now - self.last_time
        joint_velocity = (joint - self.joint) / dt if dt > 1e-4 else 0.0
        if not math.isfinite(joint):
            raise ValueError('Invalid ankle conversion')
        self.motor, self.joint, self.last_time = position, joint, now
        return joint, joint_velocity, torque

    def command(self, position):
        target = self.mapper.ankle_rad_to_motor_logical_rad(position, self.motor)
        if not math.isfinite(target):
            raise ValueError('Invalid ankle conversion')
        return target
