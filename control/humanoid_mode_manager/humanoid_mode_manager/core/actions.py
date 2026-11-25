"""Side effects the mode machine asks for. The ROS node executes them; the machine never does I/O."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SubmitCommand:
    """One-shot control_core command (motor_control_interfaces/AbstractCommand)."""
    operation: str      # 'enable', 'disable', 'home', 'cancel' or 'reset'
    request_id: str
    joint_name: str = '*'


@dataclass(frozen=True)
class SelectSource:
    """Which controller cpp_control_node forwards: 'policy', 'walk', 'getup' or 'none'.

    Placeholder: cpp_control_node has no runtime source selection yet (the source is a launch remap).
    """
    source: str


@dataclass(frozen=True)
class SwitchModel:
    """Switch tracking_policy_node to a preloaded model (/policy/switch_model, feat/policy-switching)."""
    model_name: str
    restart_reference: bool = True


@dataclass(frozen=True)
class Damp:
    """Soft stop: damping-only command on every joint. Placeholder: no damping command path yet."""
    duration_s: float


@dataclass(frozen=True)
class Kick:
    """Kick request for the walking controller (/walk/kick, std_msgs/String)."""
    foot: str           # 'left_foot' or 'right_foot'


@dataclass(frozen=True)
class ResetController:
    """Clear the active controller's latched fault. Placeholder: walking_controller has no reset service."""
    source: str


@dataclass(frozen=True)
class ResetSafety:
    """/safety/reset (std_srvs/Trigger); the estop only clears when the supervisor agrees."""


@dataclass(frozen=True)
class ResetLocalization:
    """Re-initialize localization after a fall. Placeholder: vio_localizer has no reset service."""


Action = (SubmitCommand | SelectSource | SwitchModel | Damp | Kick
          | ResetController | ResetSafety | ResetLocalization)
