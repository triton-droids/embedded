"""The selected transport mode must reach the entire simulation stack."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
import pytest


@pytest.mark.parametrize('selected', [None, 'SYNCHRONOUS'])
def test_all_simulation_children_inherit_publication_mode(monkeypatch, tmp_path, selected):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    path = Path(__file__).resolve().parents[1] / 'launch/sim.launch.py'
    spec = importlib.util.spec_from_file_location('sim_transport_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(ros_domain_id='187', localhost_only='true')
    if selected:
        context.launch_configurations['dds_publication_mode'] = selected
    # Execute only the environment setup, never spawn nodes during this test.
    for action in module.generate_launch_description().entities:
        if isinstance(action, (DeclareLaunchArgument, SetEnvironmentVariable)):
            action.execute(context)
    assert context.environment['RMW_FASTRTPS_PUBLICATION_MODE'] == (selected or 'ASYNCHRONOUS')
