"""Shared fixtures. Tests that need MuJoCo, TSID or the installed model files
are skipped when the workspace is not built and sourced."""
from pathlib import Path
import os
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

GAINS = dict(com_kp_z=100.0, com_weight=10.0)   # as node.py and walk_eval.py


def pytest_collection_modifyitems(config, items):
    if os.environ.get('WALK_SLOW_TESTS') == '1':
        return
    skip = pytest.mark.skip(reason='slow; set WALK_SLOW_TESTS=1')
    for item in items:
        if 'slow' in item.keywords:
            item.add_marker(skip)


def pytest_configure(config):
    config.addinivalue_line('markers', 'slow: long offline episodes (WALK_SLOW_TESTS=1)')


@pytest.fixture(scope='session')
def model():
    """(paths, robot, limits) from the installed packages."""
    pytest.importorskip('mujoco')
    pytest.importorskip('tsid')
    pytest.importorskip('humanoid_sim')
    from ament_index_python.packages import PackageNotFoundError
    from walking_controller.offline import load
    try:
        return load()
    except PackageNotFoundError as exc:
        pytest.skip(f'workspace not sourced: {exc}')


@pytest.fixture(scope='session')
def standing(model):
    """Robot state after the offline stand sequence (gantry, landing, squat)."""
    from walking_controller.gait import ACTIVE, GaitController, GaitParams
    from walking_controller.offline import run
    from walking_controller.wbc import WBCGains
    paths, robot, limits = model
    ctl = GaitController(robot, limits.torque, GaitParams(gains=WBCGains(**GAINS)))
    res = run(ctl, 4.0, paths=paths, robot=robot, limits=limits)
    assert ctl.phase == ACTIVE and not res['fault'] and not res['fallen']
    q, v = res['sim'].state()
    return {'q': q, 'v': v, 'controller': ctl, 'result': res}
