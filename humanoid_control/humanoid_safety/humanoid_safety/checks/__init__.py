"""Pure safety checks and the latched supervisor state machine.

One module per sensor area. New checks implement SafetyCheck and register
with SafetySupervisor without changing the shared latch/reset behavior.
"""

from .base import (
    CanObservation,
    CheckResult,
    FreshnessCheck,
    ImuObservation,
    JointObservation,
    MotorObservation,
    PowerObservation,
    SafetyCheck,
    Vector3,
)
from .comms import CanLinkCheck
from .imu import (
    FallDetectionCheck,
    ImuFreshnessCheck,
    ImuStuckCheck,
    quaternion_to_roll_pitch,
)
from .joints import JointPositionLimitCheck, JointVelocityLimitCheck
from .motors import (
    MotorFaultCheck,
    MotorFeedbackCheck,
    MotorTemperatureCheck,
    MotorTorqueCheck,
)
from .power import BatteryVoltageCheck
from .supervisor import SafetySupervisor

__all__ = [
    "BatteryVoltageCheck",
    "CanLinkCheck",
    "CanObservation",
    "CheckResult",
    "FallDetectionCheck",
    "FreshnessCheck",
    "ImuFreshnessCheck",
    "ImuObservation",
    "ImuStuckCheck",
    "JointObservation",
    "JointPositionLimitCheck",
    "JointVelocityLimitCheck",
    "MotorFaultCheck",
    "MotorFeedbackCheck",
    "MotorObservation",
    "MotorTemperatureCheck",
    "MotorTorqueCheck",
    "PowerObservation",
    "SafetyCheck",
    "SafetySupervisor",
    "Vector3",
    "quaternion_to_roll_pitch",
]

