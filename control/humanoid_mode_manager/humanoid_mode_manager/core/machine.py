"""The mode machine. Pure Python: it knows nothing about ROS, only Observations, requests and Actions.

Each tick: step(observation) -> list of Actions for the node to execute. Operator and task requests
are queued with request() and consumed on the next step. Priority on every step:

  1. /safety/estop             -> ESTOP (from any mode)
  2. own faults while active   -> DAMPING -> FAULT (stale inputs, unhealthy supervisor,
                                  controller fault that is not a fall, timeouts)
  3. the current mode's transitions

The machine never clears an estop by itself and never commands motion; it only selects who does.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import itertools
from typing import List, Optional

from .actions import (Action, Damp, Kick, ResetController, ResetLocalization, ResetSafety,
                      SelectSource, SubmitCommand, SwitchModel)
from .observation import Observation
from .states import ACTIVE_MODES, MOTION_ALLOWED, Mode, NavMode

REQUESTS = frozenset({'enable', 'disable', 'reset', 'release', 'kick_left', 'kick_right', 'squat'})
DONE_STAGES = frozenset({'completed'})
FAILED_STAGES = frozenset({'failed', 'cancelled', 'rejected'})


@dataclass
class MachineParams:
    controller: str = 'walk'            # Command source in the motion modes: 'walk' or 'policy'.
    home_on_enable: bool = True         # ENABLING -> HOMING; False goes straight to GANTRY.
    gantry: str = 'sim'                 # 'sim': leave GANTRY when /sim/status base_held is false;
                                        # 'operator': leave it on a 'release' request.
    getup: str = 'walking_controller'   # 'walking_controller' (its internal rise/straight),
                                        # 'policy' (switch to getup_model) or 'none'.
    getup_model: str = ''               # getup == 'policy': model name for /policy/switch_model.
    walk_model: str = ''                # getup == 'policy': model to switch back to after GETUP.
    max_getup_attempts: int = 3
    boot_timeout_s: float = 30.0
    command_timeout_s: float = 5.0      # ENABLE / DISABLE acknowledgement.
    home_timeout_s: float = 15.0
    damping_s: float = 1.0
    fall_tilt_rad: float = 0.45         # Tilt that counts as falling; below walking_controller's
                                        # max_tilt_rad (0.5) and the safety trip (40 deg = 0.70).
    upright_tilt_rad: float = 0.25      # Tilt that counts as upright again.
    rest_angular_speed: float = 0.3     # |gyro| below this counts as at rest.
    rest_s: float = 0.5                 # FALLING -> FALLEN after this long at rest.
    upright_s: float = 1.0              # GETUP -> STAND after this long upright (and handed over).
    getup_timeout_s: float = 30.0
    getup_reset_s: float = 20.0         # Standing/walking this long after a get-up clears the attempts.
    stand_after_s: float = 1.0          # WALK -> STAND after this long without a velocity command.
    kick_timeout_s: float = 5.0
    squat_timeout_s: float = 5.0
    use_fall_warning: bool = False      # Enter FALLING on /safety/fall_warning as well.
    external_enable: bool = False       # Someone else enables the motors (sim_autostart, an operator):
                                        # DISABLED follows a gantry release straight to STAND.
    vo_lost_s: float = 0.5              # Localization: VO older than this is degraded.
    hold_after_s: float = 3.0           # Localization degraded this long -> NavMode.HOLD.


class ModeMachine:
    def __init__(self, params: MachineParams, request_prefix: str = 'mode_manager') -> None:
        if params.controller not in ('walk', 'policy'):
            raise ValueError("controller must be 'walk' or 'policy'")
        if params.gantry not in ('sim', 'operator'):
            raise ValueError("gantry must be 'sim' or 'operator'")
        if params.getup not in ('walking_controller', 'policy', 'none'):
            raise ValueError("getup must be 'walking_controller', 'policy' or 'none'")
        if params.getup == 'policy' and not params.getup_model:
            raise ValueError("getup 'policy' needs getup_model")
        self.p = params
        self.mode = Mode.BOOT
        self.previous: Optional[Mode] = None
        self.reason = 'starting'
        self.entered_at: Optional[float] = None
        self.fault_reason = ''
        self.getup_attempts = 0
        self.nav = NavMode.UNKNOWN
        self.history = deque(maxlen=50)   # (time, from, to, reason)
        self._requests: List[str] = []
        self._ids = (f'{request_prefix}_{n}' for n in itertools.count(1))
        self._pending_id: Optional[str] = None   # control_core request in ENABLING / HOMING.
        self._after_damping = Mode.FAULT
        self._since: dict = {}                    # Condition name -> time it became true.
        self._last_cmd_vel_at: Optional[float] = None
        self._nav_degraded_at: Optional[float] = None
        self._got_up_at: Optional[float] = None

    # ------------------------------------------------------------------ public
    def request(self, name: str) -> bool:
        """Queue an operator / task request; returns False for an unknown name."""
        if name not in REQUESTS:
            return False
        self._requests.append(name)
        return True

    @property
    def motion_allowed(self) -> bool:
        """Whether task-layer velocity commands may reach the controller now."""
        return self.mode in MOTION_ALLOWED

    def status(self) -> dict:
        return {'mode': self.mode.value, 'previous': None if self.previous is None else self.previous.value,
                'reason': self.reason, 'entered_at': self.entered_at, 'fault': self.fault_reason,
                'getup_attempts': self.getup_attempts, 'nav': self.nav.value,
                'motion_allowed': self.motion_allowed, 'controller': self.p.controller,
                'history': [list(h) for h in self.history]}

    def step(self, obs: Observation) -> List[Action]:
        if self.entered_at is None:
            self.entered_at = obs.now
        requests, self._requests = self._requests, []
        actions: List[Action] = []
        if obs.cmd_vel_active:
            self._last_cmd_vel_at = obs.now
        self._update_nav(obs)

        # 1. Safety estop, from any mode.
        if obs.estop and self.mode != Mode.ESTOP:
            self._go(Mode.ESTOP, obs, 'safety estop', actions)
            actions.append(SelectSource('none'))
            return actions
        # 2. Own faults while a controller owns the motors.
        if self.mode in ACTIVE_MODES:
            fault = self._active_fault(obs)
            if fault:
                self._start_damping(obs, fault, Mode.FAULT, actions)
                return actions
            if 'disable' in requests:
                self._start_damping(obs, 'disable requested', Mode.DISABLED, actions)
                return actions
        # 3. The current mode.
        handler = getattr(self, '_on_' + self.mode.value)
        handler(obs, requests, actions)
        return actions

    # ------------------------------------------------------------------ helpers
    def _go(self, mode: Mode, obs: Observation, why: str, actions: List[Action]) -> None:
        self.history.append((round(obs.now, 3), self.mode.value, mode.value, why))
        self.previous, self.mode, self.reason, self.entered_at = self.mode, mode, why, obs.now
        self._since.clear()
        self._on_enter(mode, obs, actions)

    def _elapsed(self, obs: Observation) -> float:
        return obs.now - (self.entered_at if self.entered_at is not None else obs.now)

    def _held_for(self, name: str, condition: bool, obs: Observation) -> float:
        """How long `condition` has been continuously true (0 when false)."""
        if not condition:
            self._since.pop(name, None)
            return 0.0
        return obs.now - self._since.setdefault(name, obs.now)

    def _submit(self, operation: str, actions: List[Action]) -> str:
        request_id = next(self._ids)
        actions.append(SubmitCommand(operation, request_id))
        return request_id

    def _start_damping(self, obs, why, then: Mode, actions) -> None:
        if then == Mode.FAULT:
            self.fault_reason = why
        self._after_damping = then
        self._go(Mode.DAMPING, obs, why, actions)

    def _active_fault(self, obs: Observation) -> str:
        if not obs.inputs_fresh:
            return 'stale inputs: ' + ', '.join(obs.stale_inputs or ('unknown',))
        if obs.supervisor_ok is False:
            return 'supervisor unhealthy: ' + obs.supervisor_reason
        c = obs.controller
        # A controller fault during a fall belongs to the fall handling, not to FAULT.
        if c is not None and c.fault and self.mode not in (Mode.FALLING, Mode.FALLEN, Mode.GETUP) \
                and not self._falling(obs):
            return 'controller fault: ' + c.fault
        return ''

    def _falling(self, obs: Observation) -> bool:
        if obs.tilt_rad is not None and obs.tilt_rad > self.p.fall_tilt_rad:
            return True
        if self.p.use_fall_warning and obs.fall_warning:
            return True
        c = obs.controller
        # walking_controller with fall_recovery has already started its get-up.
        return c is not None and c.recovery in ('rise', 'straight')

    def _upright(self, obs: Observation) -> bool:
        return obs.tilt_rad is not None and obs.tilt_rad < self.p.upright_tilt_rad

    def _at_rest(self, obs: Observation) -> bool:
        return obs.angular_speed is not None and obs.angular_speed < self.p.rest_angular_speed

    def _update_nav(self, obs: Observation) -> None:
        loc = obs.localization
        if loc is None:
            self.nav, self._nav_degraded_at = NavMode.UNKNOWN, None
            return
        vo_age = loc.get('vo_age_s')
        healthy = bool(loc.get('initialized')) and isinstance(vo_age, (int, float)) \
            and vo_age <= self.p.vo_lost_s
        if healthy:
            self.nav, self._nav_degraded_at = NavMode.NAVIGATE, None
            return
        if self._nav_degraded_at is None:
            self._nav_degraded_at = obs.now
        self.nav = (NavMode.HOLD if obs.now - self._nav_degraded_at >= self.p.hold_after_s
                    else NavMode.LOCAL_ONLY)

    def _command_stage(self, obs: Observation) -> Optional[str]:
        return None if self._pending_id is None else obs.command_stages.get(self._pending_id)

    # ------------------------------------------------------------------ entry actions
    def _on_enter(self, mode: Mode, obs: Observation, actions: List[Action]) -> None:
        if mode == Mode.DISABLED:
            actions.append(SelectSource('none'))
            self._submit('disable', actions)
        elif mode == Mode.DAMPING:
            actions.append(Damp(self.p.damping_s))
        elif mode == Mode.FAULT:
            actions.append(SelectSource('none'))
            self._submit('disable', actions)
        elif mode == Mode.ENABLING:
            self._pending_id = self._submit('enable', actions)
        elif mode == Mode.HOMING:
            self._pending_id = self._submit('home', actions)
        elif mode == Mode.GANTRY:
            self.getup_attempts = 0
            actions.append(SelectSource(self.p.controller))
        elif mode == Mode.FALLEN:
            actions.append(ResetLocalization())
        elif mode == Mode.GETUP:
            self.getup_attempts += 1
            if self.p.getup == 'policy':
                actions.append(SwitchModel(self.p.getup_model))
                actions.append(SelectSource('policy'))
        elif mode == Mode.STAND and self.previous == Mode.GETUP and self.p.getup == 'policy' \
                and self.p.walk_model:
            actions.append(SwitchModel(self.p.walk_model))

    # ------------------------------------------------------------------ per-mode transitions
    def _on_estop(self, obs, requests, actions) -> None:
        if 'reset' in requests:
            actions.append(ResetSafety())
        if obs.estop is False:
            # Released by humanoid_safety (after its own reset): back to the safe default.
            self._go(Mode.DISABLED, obs, 'estop released', actions)

    def _on_damping(self, obs, requests, actions) -> None:
        if self._elapsed(obs) >= self.p.damping_s:
            self._go(self._after_damping, obs, self.reason, actions)

    def _on_fault(self, obs, requests, actions) -> None:
        if 'reset' in requests:
            actions.append(ResetController(self.p.controller))
            self.fault_reason = ''
            self._go(Mode.DISABLED, obs, 'operator reset', actions)

    def _on_boot(self, obs, requests, actions) -> None:
        if obs.inputs_fresh and obs.estop is not None and obs.controller is not None:
            self._go(Mode.DISABLED, obs, 'inputs ready', actions)
        elif self._elapsed(obs) > self.p.boot_timeout_s:
            self.fault_reason = 'boot timeout: ' + ', '.join(obs.stale_inputs or ('missing inputs',))
            self._go(Mode.FAULT, obs, self.fault_reason, actions)

    def _on_disabled(self, obs, requests, actions) -> None:
        if self.p.external_enable and obs.base_held is False and obs.estop is False \
                and obs.inputs_fresh and self.previous == Mode.BOOT:
            # Only from the first DISABLED: after a fault or estop the motors are off again.
            self._go(Mode.GANTRY, obs, 'enabled externally', actions)
            return
        if 'enable' in requests:
            # Same admission as cpp_control_node: no enable while a safety check is not clear.
            if not obs.inputs_fresh or obs.estop is not False or obs.supervisor_warn:
                self.reason = 'enable refused: inputs stale, estop or supervisor warning'
                return
            self._go(Mode.ENABLING, obs, 'enable requested', actions)

    def _on_enabling(self, obs, requests, actions) -> None:
        stage = self._command_stage(obs)
        if stage in DONE_STAGES:
            self._go(Mode.HOMING if self.p.home_on_enable else Mode.GANTRY, obs, 'enabled', actions)
        elif stage in FAILED_STAGES or self._elapsed(obs) > self.p.command_timeout_s:
            self._start_damping(obs, f'enable {stage or "timed out"}', Mode.FAULT, actions)

    def _on_homing(self, obs, requests, actions) -> None:
        stage = self._command_stage(obs)
        if stage in DONE_STAGES:
            self._go(Mode.GANTRY, obs, 'homed', actions)
        elif stage in FAILED_STAGES or self._elapsed(obs) > self.p.home_timeout_s:
            self._start_damping(obs, f'home {stage or "timed out"}', Mode.FAULT, actions)

    def _on_gantry(self, obs, requests, actions) -> None:
        released = (obs.base_held is False if self.p.gantry == 'sim' else 'release' in requests)
        if released:
            self._go(Mode.STAND, obs, 'gantry released', actions)

    def _clear_attempts_after_getup(self, obs) -> None:
        if self._got_up_at is not None and obs.now - self._got_up_at >= self.p.getup_reset_s:
            self.getup_attempts, self._got_up_at = 0, None

    def _on_stand(self, obs, requests, actions) -> None:
        self._clear_attempts_after_getup(obs)
        if self._falling(obs):
            self._got_up_at = None  # A fall before getup_reset_s keeps counting attempts.
            self._go(Mode.FALLING, obs, 'fall detected', actions)
        elif 'kick_left' in requests or 'kick_right' in requests:
            self._start_kick(obs, requests, actions)
        elif 'squat' in requests:
            self._go(Mode.SQUAT, obs, 'squat requested', actions)
        elif obs.cmd_vel_active:
            self._go(Mode.WALK, obs, 'velocity command', actions)

    def _on_walk(self, obs, requests, actions) -> None:
        self._clear_attempts_after_getup(obs)
        idle = (self._last_cmd_vel_at is None or obs.now - self._last_cmd_vel_at >= self.p.stand_after_s)
        if self._falling(obs):
            self._got_up_at = None  # A fall before getup_reset_s keeps counting attempts.
            self._go(Mode.FALLING, obs, 'fall detected', actions)
        elif 'kick_left' in requests or 'kick_right' in requests:
            self._start_kick(obs, requests, actions)
        elif idle:
            self._go(Mode.STAND, obs, 'no velocity command', actions)

    def _start_kick(self, obs, requests, actions) -> None:
        if self.p.controller != 'walk':
            self.reason = 'kick refused: only the walking controller kicks'
            return
        foot = 'left_foot' if 'kick_left' in requests else 'right_foot'
        actions.append(Kick(foot))
        self._go(Mode.KICK, obs, f'kick {foot}', actions)

    def _on_kick(self, obs, requests, actions) -> None:
        c = obs.controller
        if self._falling(obs):
            self._go(Mode.FALLING, obs, 'fall during kick', actions)
        # The status reports the kick after a 0.5 s period at the latest; wait for it to clear.
        elif self._held_for('kick_idle', c is not None and c.kick is None, obs) >= 1.0:
            self._go(Mode.WALK if obs.cmd_vel_active else Mode.STAND, obs, 'kick done', actions)
        elif self._elapsed(obs) > self.p.kick_timeout_s:
            self._go(Mode.STAND, obs, 'kick timed out', actions)

    def _on_squat(self, obs, requests, actions) -> None:
        # Placeholder: walking_controller's squat is a launch-time activity, not a runtime request.
        if self._falling(obs):
            self._go(Mode.FALLING, obs, 'fall during squat', actions)
        elif self._elapsed(obs) > self.p.squat_timeout_s:
            self._go(Mode.STAND, obs, 'squat placeholder done', actions)

    def _on_falling(self, obs, requests, actions) -> None:
        c = obs.controller
        if self.p.getup == 'walking_controller' and c is not None and c.recovery in ('rise', 'straight'):
            # The walking controller plans its own get-up; FALLEN would only lose time.
            self._go(Mode.GETUP, obs, f'controller recovery: {c.recovery}', actions)
        elif self._held_for('rest', self._at_rest(obs), obs) >= self.p.rest_s:
            self._go(Mode.FALLEN, obs, 'at rest', actions)
        elif self._held_for('upright', self._upright(obs), obs) >= self.p.upright_s:
            self._go(Mode.STAND, obs, 'caught itself', actions)

    def _on_fallen(self, obs, requests, actions) -> None:
        if self.p.getup == 'none':
            self._start_damping(obs, 'fallen, no get-up configured', Mode.DISABLED, actions)
        elif self.getup_attempts >= self.p.max_getup_attempts:
            self._start_damping(obs, f'fallen after {self.getup_attempts} get-up attempts',
                                Mode.FAULT, actions)
        else:
            self._go(Mode.GETUP, obs, f'get-up attempt {self.getup_attempts + 1}', actions)

    def _on_getup(self, obs, requests, actions) -> None:
        c = obs.controller
        handed_over = (self.p.getup != 'walking_controller'
                       or (c is not None and c.recovery in (None, 'walk')))
        if self._held_for('up', self._upright(obs) and handed_over, obs) >= self.p.upright_s:
            self._got_up_at = obs.now
            self._go(Mode.STAND, obs, 'got up', actions)
        elif self._elapsed(obs) > self.p.getup_timeout_s:
            self._start_damping(obs, 'get-up timed out', Mode.FAULT, actions)
        elif self._elapsed(obs) > 2.0 and self.p.getup != 'walking_controller' \
                and self._held_for('down', self._at_rest(obs) and not self._upright(obs), obs) >= 2.0:
            # A get-up policy gave up lying down: count it as a new fall.
            self._go(Mode.FALLEN, obs, 'get-up failed', actions)
