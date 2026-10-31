"""GUI stalls must not prevent physics steps or IMU reads."""
from contextlib import nullcontext
import threading
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from humanoid_sim.viewer import ViewerMirror, ViewerProcess  # noqa: E402


def _blocked_render_worker(model, snapshots, forces, ready, closed, stop):
    sys.setswitchinterval(10.0)
    ready.set()
    end = time.monotonic() + .8
    while time.monotonic() < end:
        pass
    closed.set()


def test_render_process_holding_gil_does_not_stall_parent_physics(world):
    viewer = ViewerProcess(world, worker=_blocked_render_worker)
    try:
        begin = time.monotonic()
        for _ in range(50):
            with world.lock:
                mujoco.mj_step(world.model, world.data)
            viewer.update()
        assert time.monotonic() - begin < .3
        assert viewer.is_running()
    finally:
        viewer.close()


@pytest.fixture
def world():
    model = mujoco.MjModel.from_xml_string('''
      <mujoco><worldbody><body pos="0 0 1"><freejoint/>
        <geom type="sphere" size="0.1" mass="1"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return SimpleNamespace(model=model, data=data, lock=threading.RLock())


@pytest.mark.parametrize('stall_at', ['lock', 'sync'])
def test_gui_stall_does_not_block_physics(world, stall_at):
    mirror = ViewerMirror(world)
    entered = threading.Event()
    release = threading.Event()

    class StalledViewer:
        def lock(self):
            if stall_at == 'lock':
                entered.set()
                release.wait(2.0)
            return nullcontext()

        def sync(self):
            if stall_at == 'sync':
                entered.set()
                release.wait(2.0)

    thread = threading.Thread(target=mirror.sync, args=(StalledViewer(),))
    thread.start()
    try:
        assert entered.wait(1.0)
        # This is the same lock used by SimWorld.step() and body_imu().
        acquired = world.lock.acquire(timeout=0.1)
        assert acquired, 'GUI stall blocked physics/IMU access'
        try:
            mujoco.mj_step(world.model, world.data)
        finally:
            world.lock.release()
    finally:
        release.set()
        thread.join(timeout=2.0)
    assert not thread.is_alive()


def test_mirror_copies_state_and_keeps_gui_edits_separate(world):
    mirror = ViewerMirror(world)
    with world.lock:
        world.data.qpos[0] = 0.3
        world.data.qvel[0] = 0.2
        world.data.time = 1.5
    mirror.update()
    np.testing.assert_allclose(mirror.data.qpos, world.data.qpos)
    np.testing.assert_allclose(mirror.data.qvel, world.data.qvel)
    assert mirror.data.time == world.data.time
    mirror.data.qpos[0] = 9.0
    mirror.model.opt.gravity[2] = 0.0
    assert world.data.qpos[0] == 0.3
    assert world.model.opt.gravity[2] == -9.81


def test_mouse_perturbation_force_reaches_physics(world):
    mirror = ViewerMirror(world)
    mirror.data.xfrc_applied[1, 0] = 2.5
    mirror.update()
    assert world.data.xfrc_applied[1, 0] == 2.5
    mirror.data.xfrc_applied[:] = 0.0
    mirror.update()
    assert not world.data.xfrc_applied.any()
