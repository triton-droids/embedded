"""Synthetic zero and measured-velocity regressions; no ROS participant or CAN."""
import time
from types import SimpleNamespace
from unittest.mock import Mock
from motor_control_interfaces.msg import MotorCommand
from motor_control_interfaces.srv import ExecuteMotorOperation
from motor_control_hybrid.fake_motor_node import FakeMotorNode


def fake_state():
    return SimpleNamespace(joint_names=['a', 'b'], _zero_requests=set(), _estop_active=False,
        _position={'a': .3, 'b': -.2}, _target_position={'a': .3, 'b': -.2},
        _velocity={'a': 0., 'b': 0.}, _enabled={'a': False, 'b': False},
        _mode={'a': MotorCommand.MODE_DISABLE, 'b': MotorCommand.MODE_DISABLE},
        _effort={'a': 0., 'b': 0.}, _temperature={'a': 28., 'b': 28.},
        _position_speed={'a': .1, 'b': .1})


def zero(state, request_id='zero1'):
    request = ExecuteMotorOperation.Request(request_id=request_id, joint_name='a',
        operation=ExecuteMotorOperation.Request.SET_MECHANICAL_ZERO)
    return FakeMotorNode._set_zero(state, request, ExecuteMotorOperation.Response())


def test_simulated_zero_changes_only_selected_coordinate_and_never_enables():
    state = fake_state()
    assert zero(state).accepted
    assert state._position == {'a': 0., 'b': -.2}
    assert state._target_position['a'] == 0.
    assert not any(state._enabled.values())
    assert not zero(state).accepted  # Same operation cannot execute twice.


def test_simulated_zero_rejects_motion_or_estop_without_coordinate_change():
    state = fake_state()
    state._velocity['b'] = .1
    assert not zero(state).accepted
    state._velocity['b'] = 0.
    state._estop_active = True
    assert not zero(state).accepted
    assert state._position['a'] == .3


def test_position_speed_limit_is_not_reported_as_measured_velocity():
    state = fake_state()
    state._enabled['a'] = True
    state._mode['a'] = MotorCommand.MODE_POSITION
    state._velocity['a'] = .1
    state._last_update = time.monotonic() - .02
    state._publish_joint_states = Mock()
    state._publish_motor_status = Mock()
    FakeMotorNode._tick(state)
    assert state._velocity['a'] == 0.  # Already at target despite a .1 speed limit.
    state._target_position['a'] = .2
    state._last_update = time.monotonic() - .02
    FakeMotorNode._tick(state)
    assert -.101 <= state._velocity['a'] < 0


def test_zero_gain_idle_command_does_not_move_nonzero_fake_joint():
    state = fake_state()
    state._enabled['a'] = True
    state._mode['a'] = MotorCommand.MODE_MOTION
    state._kp = {'a':0.,'b':0.};state._kd = {'a':0.,'b':0.}
    state._target_position['a'] = 0.
    state._last_update = time.monotonic() - .02
    state._publish_joint_states = Mock();state._publish_motor_status = Mock()
    FakeMotorNode._tick(state)
    assert state._position['a'] == .3 and state._velocity['a'] == 0.
