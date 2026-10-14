"""Validate hardware mappings and convert calibrated joint/motor coordinates."""
from dataclasses import dataclass
import math
from .ankle_mapping import AnkleMapper


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

    @classmethod
    def from_config(cls, cfg, defaults):
        direction = cfg.get('direction', 1)
        if isinstance(direction, bool) or direction not in (-1, 1):
            raise ValueError('direction must be +1 or -1')
        settings = cls(
            int(direction), float(cfg.get('encoder_offset_rad', 0.0)),
            float(cfg.get('min_position', -math.inf)), float(cfg.get('max_position', math.inf)),
            float(cfg.get('kp', defaults.get('KP', 10.0))),
            float(cfg.get('kd', defaults.get('KD', 0.2))),
            float(cfg.get('max_vel_rad_s', defaults.get('MAX_VEL_RAD_S', 4.5))),
            float(cfg['max_torque_nm']) if cfg.get('max_torque_nm') is not None else None,
            AnkleCalibration(cfg['ankle_mapping']) if cfg.get('ankle_mapping') else None,
        )
        if (settings.direction not in (-1, 1) or not math.isfinite(settings.offset)
                or math.isnan(settings.lower) or math.isnan(settings.upper)
                or settings.lower >= settings.upper
                or not all(math.isfinite(x) and x >= 0 for x in
                           (settings.kp, settings.kd, settings.max_velocity))
                or (settings.max_torque is not None and
                    (not math.isfinite(settings.max_torque) or settings.max_torque <= 0))):
            raise ValueError('Invalid motor calibration, limits or gains')
        return settings

    def feedback(self, position, velocity, torque):
        if not all(math.isfinite(x) for x in (position, velocity, torque)):
            raise ValueError('Non-finite motor feedback')
        if self.ankle is not None:
            return self.ankle.feedback(self.direction * (position - self.offset), self.direction * velocity, self.direction * torque)
        return (self.direction * (position - self.offset),
                self.direction * velocity, self.direction * torque)

    def command(self, command):
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
        result.update(position=self.offset + self.direction * position,
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
        interface = cfg.get('can_interface', params.get('default_can_interface', 'can0'))
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
