"""One snapshot of everything the mode machine decides on, and parsers for the JSON status topics.

The ROS node fills an Observation from its subscriptions every tick; the machine only reads it.
None means "not received yet" wherever a field is optional.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from typing import Mapping, Optional


@dataclass(frozen=True)
class ControllerStatus:
    """The parts of /walk/status or /policy/status the mode machine uses."""
    state: str = ''                  # walk: gait phase or 'fault'; policy: 'running', 'fault', ...
    fault: str = ''                  # Empty when healthy.
    recovery: Optional[str] = None   # walk with fall_recovery: 'walk', 'rise' or 'straight'; else None.
    kick: Optional[str] = None       # walk: 'requested', 'armed' or 'kicking'; None when idle.


@dataclass(frozen=True)
class Observation:
    now: float                                  # Seconds, the node's clock.
    estop: Optional[bool] = None                # /safety/estop
    supervisor_ok: Optional[bool] = None        # /safety/status supervisor not at ERROR (latched).
    supervisor_warn: bool = False               # At WARN: a check initializing or unhealthy, not latched.
    supervisor_reason: str = ''
    inputs_fresh: bool = False                  # Joints, IMU and the controller status are fresh.
    stale_inputs: tuple = ()                    # Names of the stale inputs, for the fault reason.
    controller: Optional[ControllerStatus] = None
    base_held: Optional[bool] = None            # /sim/status 'base_held'; None on hardware.
    tilt_rad: Optional[float] = None            # Torso tilt from the body IMU orientation.
    angular_speed: Optional[float] = None       # |gyro|, rad/s.
    fall_warning: bool = False                  # /safety/fall_warning (learned predictor, advisory).
    cmd_vel_active: bool = False                # A fresh, non-zero task-layer velocity command.
    command_stages: Mapping[str, str] = field(default_factory=dict)  # control_core request_id -> stage.
    localization: Optional[Mapping] = None      # Parsed /localization/status.


def _load(text: str) -> Optional[dict]:
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def parse_controller_status(text: str) -> Optional[ControllerStatus]:
    """/walk/status or /policy/status JSON (both have 'state' and 'fault')."""
    data = _load(text)
    if data is None:
        return None
    recovery = data.get('recovery')
    kick = data.get('kick')
    return ControllerStatus(
        state=str(data.get('state') or ''),
        fault=str(data.get('fault') or ''),
        recovery=recovery if isinstance(recovery, str) and recovery != 'off' else None,
        kick=kick if isinstance(kick, str) and kick else None)


def parse_base_held(text: str) -> Optional[bool]:
    """/sim/status JSON 'base_held'."""
    data = _load(text)
    held = None if data is None else data.get('base_held')
    return held if isinstance(held, bool) else None


def parse_localization_status(text: str) -> Optional[dict]:
    return _load(text)


def tilt_from_quaternion(x: float, y: float, z: float, w: float) -> Optional[float]:
    """Angle between the body z axis and world up (rad); None for an invalid quaternion."""
    norm = x * x + y * y + z * z + w * w
    if not math.isfinite(norm) or norm < 1e-6:
        return None
    # World z component of the body z axis: R[2, 2] of the normalized quaternion.
    r22 = 1.0 - 2.0 * (x * x + y * y) / norm
    return math.acos(max(-1.0, min(1.0, r22)))
