"""Robot modes of the mode manager, grouped into the safety, lifecycle and motion layers.

The safety layer is entered from any mode. The estop itself belongs to humanoid_safety;
the mode manager only follows it (ESTOP) or stops on its own faults (DAMPING -> FAULT).
"""
from __future__ import annotations

from enum import Enum


class Mode(str, Enum):
    # Safety layer.
    ESTOP = 'estop'          # /safety/estop is true; humanoid_safety has disabled the motors.
    DAMPING = 'damping'      # Soft stop: damping only, then DISABLED (requested) or FAULT (fault).
    FAULT = 'fault'          # Latched mode-manager fault; an operator reset returns to DISABLED.
    # Lifecycle layer.
    BOOT = 'boot'            # Waiting for every input to arrive and be fresh.
    DISABLED = 'disabled'    # Motors disabled; the default safe mode.
    ENABLING = 'enabling'    # ENABLE submitted to control_core, waiting for COMPLETED.
    HOMING = 'homing'        # HOME submitted to control_core, waiting for COMPLETED.
    GANTRY = 'gantry'        # Controller selected, the base is still held.
    # Motion layer.
    STAND = 'stand'
    WALK = 'walk'
    KICK = 'kick'            # Placeholder: forwarded to the walking controller, ends on its status.
    SQUAT = 'squat'          # Placeholder: no runtime squat switch exists yet.
    FALLING = 'falling'      # Tilting past the fall threshold, not yet at rest.
    FALLEN = 'fallen'        # Lying at rest.
    GETUP = 'getup'


SAFETY_MODES = frozenset({Mode.ESTOP, Mode.DAMPING, Mode.FAULT})
LIFECYCLE_MODES = frozenset({Mode.BOOT, Mode.DISABLED, Mode.ENABLING, Mode.HOMING, Mode.GANTRY})
MOTION_MODES = frozenset({Mode.STAND, Mode.WALK, Mode.KICK, Mode.SQUAT,
                          Mode.FALLING, Mode.FALLEN, Mode.GETUP})
# Modes in which a controller owns the motors (stale inputs or an unhealthy supervisor fault).
ACTIVE_MODES = frozenset({Mode.HOMING, Mode.GANTRY}) | MOTION_MODES
# Modes in which task-layer velocity commands are passed on.
MOTION_ALLOWED = frozenset({Mode.STAND, Mode.WALK})


class NavMode(str, Enum):
    """WALK sub-mode from localization health (second stage; published only, no actions yet)."""
    NAVIGATE = 'navigate'      # Localization healthy: global path, shooting.
    LOCAL_ONLY = 'local_only'  # Degraded: relative (vision) chase only, no shooting.
    HOLD = 'hold'              # Lost for too long: stand, look for features to relocalize.
    UNKNOWN = 'unknown'        # No localization status received.
