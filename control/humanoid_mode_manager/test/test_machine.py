"""ModeMachine without ROS: lifecycle, motion, falls and faults."""
from dataclasses import replace

import pytest

from humanoid_mode_manager.core.actions import (Damp, Kick, ResetController, ResetLocalization, ResetSafety,
                                           SelectSource, SubmitCommand, SwitchModel)
from humanoid_mode_manager.core.machine import MachineParams, ModeMachine
from humanoid_mode_manager.core.observation import (ControllerStatus, Observation, parse_controller_status,
                                               tilt_from_quaternion)
from humanoid_mode_manager.core.states import Mode, NavMode

HEALTHY = Observation(now=0.0, estop=False, supervisor_ok=True, inputs_fresh=True,
                      controller=ControllerStatus(state='ss'), base_held=True, tilt_rad=0.05,
                      angular_speed=0.0)


class Run:
    """Steps a machine through time with an editable observation."""

    def __init__(self, **params):
        self.m = ModeMachine(MachineParams(**params), request_prefix='t')
        self.obs = HEALTHY
        self.actions = []

    def step(self, dt=0.02, **changes):
        self.obs = replace(self.obs, now=self.obs.now + dt, **changes)
        self.actions = self.m.step(self.obs)
        return self.m.mode

    def hold(self, seconds, **changes):
        """Step for up to `seconds`, stopping at the first mode change; actions accumulate."""
        start, collected = self.m.mode, []
        for _ in range(int(round(seconds / 0.02))):
            self.step(**changes)
            changes = {}
            collected += self.actions
            if self.m.mode != start:
                break
        self.actions = collected
        return self.m.mode

    def submitted(self, operation):
        return [a for a in self.actions if isinstance(a, SubmitCommand) and a.operation == operation]

    def complete_pending(self):
        stages = dict(self.obs.command_stages)
        stages[self.m._pending_id] = 'completed'
        return self.step(command_stages=stages)


def standing(**params):
    """A machine taken BOOT -> DISABLED -> ENABLING -> HOMING -> GANTRY -> STAND."""
    r = Run(**params)
    assert r.step() == Mode.DISABLED
    r.m.request('enable')
    assert r.step() == Mode.ENABLING
    assert r.submitted('enable')
    assert r.complete_pending() == Mode.HOMING
    assert r.submitted('home')
    assert r.complete_pending() == Mode.GANTRY
    assert SelectSource(r.m.p.controller) in r.actions
    assert r.step(base_held=False) == Mode.STAND
    return r


def test_boot_waits_for_inputs_then_times_out():
    r = Run(boot_timeout_s=1.0)
    r.obs = replace(HEALTHY, inputs_fresh=False, stale_inputs=('imu',), controller=None)
    assert r.step() == Mode.BOOT
    assert r.hold(1.1) == Mode.FAULT
    assert 'imu' in r.m.fault_reason


def test_lifecycle_to_stand():
    standing()


def test_enable_refused_while_estop_unknown():
    r = Run()
    r.step()
    r.m.request('enable')
    assert r.step(estop=None) == Mode.DISABLED
    assert 'refused' in r.m.reason


def test_enable_failure_damps_then_faults():
    r = Run()
    r.step()
    r.m.request('enable')
    r.step()
    stages = {r.m._pending_id: 'rejected'}
    assert r.step(command_stages=stages) == Mode.DAMPING
    assert any(isinstance(a, Damp) for a in r.actions)
    assert r.hold(1.1) == Mode.FAULT
    assert r.submitted('disable')


def test_without_homing_and_with_operator_gantry():
    r = Run(home_on_enable=False, gantry='operator')
    r.step()
    r.m.request('enable')
    r.step()
    assert r.complete_pending() == Mode.GANTRY
    assert r.step(base_held=None) == Mode.GANTRY
    r.m.request('release')
    assert r.step() == Mode.STAND


def test_walk_and_back_to_stand():
    r = standing(stand_after_s=0.5)
    assert r.step(cmd_vel_active=True) == Mode.WALK
    assert r.m.motion_allowed
    assert r.hold(0.3, cmd_vel_active=False) == Mode.WALK
    assert r.hold(0.3) == Mode.STAND


def test_estop_from_any_mode_and_release():
    r = standing()
    r.step(cmd_vel_active=True)
    assert r.step(estop=True) == Mode.ESTOP
    assert SelectSource('none') in r.actions
    assert not r.m.motion_allowed
    r.m.request('reset')
    r.step()
    assert ResetSafety() in r.actions
    assert r.m.mode == Mode.ESTOP          # Still latched until humanoid_safety releases it.
    assert r.step(estop=False) == Mode.DISABLED


def test_stale_inputs_while_active_fault():
    r = standing()
    assert r.step(inputs_fresh=False, stale_inputs=('joint_states',)) == Mode.DAMPING
    assert r.hold(1.1) == Mode.FAULT
    assert 'joint_states' in r.m.fault_reason
    r.m.request('reset')
    assert r.step(inputs_fresh=True) == Mode.DISABLED
    assert ResetController('walk') in r.actions


def test_unhealthy_supervisor_faults():
    r = standing()
    assert r.step(supervisor_ok=False, supervisor_reason='imu stale') == Mode.DAMPING


def test_controller_fault_without_fall_faults():
    r = standing()
    assert r.step(controller=ControllerStatus(state='fault', fault='Cycle took 30 ms')) == Mode.DAMPING
    assert 'Cycle' in r.m.fault_reason


def test_disable_request_soft_stops_to_disabled():
    r = standing()
    r.m.request('disable')
    assert r.step() == Mode.DAMPING
    assert r.hold(1.1) == Mode.DISABLED
    assert r.m.fault_reason == ''


def test_fall_with_walking_controller_recovery():
    r = standing()
    r.step(cmd_vel_active=True)
    # The controller faults on tilt and starts its own get-up: no FAULT, straight to GETUP.
    rising = ControllerStatus(state='ss', recovery='rise')
    assert r.step(tilt_rad=1.2, angular_speed=2.0, controller=rising, cmd_vel_active=False) == Mode.FALLING
    assert r.step() == Mode.GETUP
    assert r.m.getup_attempts == 1
    assert r.hold(5.0, tilt_rad=0.1, controller=ControllerStatus(state='ss', recovery='straight')) == Mode.GETUP
    assert r.hold(1.1, controller=ControllerStatus(state='ss', recovery='walk')) == Mode.STAND


def test_fall_to_rest_then_getup_and_attempt_limit():
    r = standing(getup='policy', getup_model='getup', walk_model='walk', max_getup_attempts=2)
    assert r.step(tilt_rad=1.5, angular_speed=3.0) == Mode.FALLING
    assert r.hold(0.6, angular_speed=0.0) == Mode.FALLEN
    assert ResetLocalization() in r.actions
    for attempt in (1, 2):
        assert r.step() == Mode.GETUP
        assert SwitchModel('getup') in r.actions
        assert r.m.getup_attempts == attempt
        # The get-up policy gives up lying down: a new fall.
        assert r.hold(4.5) == Mode.FALLEN
    assert r.step() == Mode.DAMPING
    assert r.hold(1.1) == Mode.FAULT
    assert 'attempts' in r.m.fault_reason


def test_getup_success_switches_back_and_clears_attempts():
    r = standing(getup='policy', getup_model='getup', walk_model='walk', getup_reset_s=2.0)
    r.step(tilt_rad=1.5, angular_speed=3.0)
    r.hold(0.6, angular_speed=0.0)
    r.step()
    assert r.m.mode == Mode.GETUP
    assert r.hold(1.1, tilt_rad=0.1) == Mode.STAND
    assert SwitchModel('walk') in r.actions
    assert r.m.getup_attempts == 1
    r.hold(2.1)
    assert r.m.getup_attempts == 0


def test_no_getup_disables_after_fall():
    r = standing(getup='none')
    r.step(tilt_rad=1.5, angular_speed=3.0)
    assert r.hold(0.6, angular_speed=0.0) == Mode.FALLEN
    assert r.step() == Mode.DAMPING
    assert r.hold(1.1) == Mode.DISABLED


def test_caught_itself_returns_to_stand():
    r = standing()
    r.step(tilt_rad=0.7, angular_speed=1.0)
    assert r.m.mode == Mode.FALLING
    assert r.hold(1.1, tilt_rad=0.1, angular_speed=1.0) == Mode.STAND


def test_kick_cycle():
    r = standing()
    r.m.request('kick_left')
    assert r.step() == Mode.KICK
    assert Kick('left_foot') in r.actions
    assert r.hold(0.5, controller=ControllerStatus(state='ss', kick='kicking')) == Mode.KICK
    assert r.hold(1.1, controller=ControllerStatus(state='ss')) == Mode.STAND


def test_kick_refused_for_policy():
    r = standing(controller='policy')
    r.m.request('kick_right')
    assert r.step() == Mode.STAND
    assert 'refused' in r.m.reason


def test_nav_submode_from_localization():
    r = standing(hold_after_s=1.0)
    assert r.m.nav == NavMode.UNKNOWN
    r.step(localization={'initialized': True, 'vo_age_s': 0.1})
    assert r.m.nav == NavMode.NAVIGATE
    r.step(localization={'initialized': True, 'vo_age_s': 2.0})
    assert r.m.nav == NavMode.LOCAL_ONLY
    r.hold(1.1)
    assert r.m.nav == NavMode.HOLD


def test_unknown_request_and_bad_params():
    m = ModeMachine(MachineParams())
    assert not m.request('fly')
    with pytest.raises(ValueError):
        ModeMachine(MachineParams(controller='mpc'))
    with pytest.raises(ValueError):
        ModeMachine(MachineParams(getup='policy'))


def test_status_json_fields():
    r = standing()
    status = r.m.status()
    assert status['mode'] == 'stand'
    assert [h[1:3] for h in status['history']][:2] == [['boot', 'disabled'], ['disabled', 'enabling']]


def test_parsers():
    walk = parse_controller_status('{"state": "fault", "fault": "tilt", "recovery": "off", "kick": null}')
    assert walk == ControllerStatus(state='fault', fault='tilt')
    assert parse_controller_status('{"state": "ss", "fault": null, "recovery": "rise"}').recovery == 'rise'
    assert parse_controller_status('not json') is None
    assert tilt_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    assert tilt_from_quaternion(0.7071068, 0.0, 0.0, 0.7071068) == pytest.approx(1.5707963, abs=1e-6)
    assert tilt_from_quaternion(0.0, 0.0, 0.0, 0.0) is None


def test_external_enable_follows_gantry_release_once():
    r = Run(external_enable=True)
    assert r.step() == Mode.DISABLED
    assert r.step() == Mode.DISABLED          # Still on the gantry.
    assert r.step(base_held=False) == Mode.GANTRY
    assert r.step() == Mode.STAND
    r.step(estop=True)
    assert r.step(estop=False) == Mode.DISABLED
    assert r.hold(0.5) == Mode.DISABLED       # Not again after an estop.


def test_supervisor_warning_blocks_enable_but_not_motion():
    r = Run()
    r.step(supervisor_warn=True)
    r.m.request('enable')
    assert r.step() == Mode.DISABLED
    assert 'supervisor' in r.m.reason
    r = standing()
    assert r.hold(1.0, supervisor_warn=True) == Mode.STAND
