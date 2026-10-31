"""A viewer mirror whose GUI synchronization never holds the physics lock."""
from __future__ import annotations

import copy
import multiprocessing
from queue import Empty, Full
import time

import mujoco
import numpy as np

from .world import SimWorld


def _latest(queue, value):
    try:
        queue.put_nowait(value)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        try:
            queue.put_nowait(value)
        except Full:
            pass


def _viewer_worker(model, snapshots, forces, ready, closed, stop):
    import mujoco.viewer

    data = mujoco.MjData(model)
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    viewer = None
    try:
        viewer = mujoco.viewer.launch_passive(model, data)
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = model.body('floating_base').id
            viewer.cam.distance = 2.5
        ready.set()
        while not stop.is_set() and viewer.is_running():
            state = None
            try:
                while True:
                    state = snapshots.get_nowait()
            except Empty:
                pass
            with viewer.lock():
                perturbation = data.xfrc_applied.copy()
                if state is not None:
                    mujoco.mj_setState(model, data, state, spec)
                    mujoco.mj_forward(model, data)
                data.xfrc_applied[:] = perturbation
            viewer.sync()
            _latest(forces, data.xfrc_applied.copy())
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        closed.set()


class ViewerProcess:
    def __init__(self, world, worker=None):
        self.world = world
        self._spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self._state = np.empty(mujoco.mj_stateSize(world.model, self._spec))
        context = multiprocessing.get_context('spawn')
        self._snapshots = context.Queue(maxsize=1)
        self._forces = context.Queue(maxsize=1)
        self._ready = context.Event()
        self._closed = context.Event()
        self._stop = context.Event()
        self._process = context.Process(
            target=worker or _viewer_worker,
            args=(world.model, self._snapshots, self._forces,
                  self._ready, self._closed, self._stop),
            name='mujoco_viewer', daemon=True,
        )
        self._process.start()
        if not self._ready.wait(10.0) or not self._process.is_alive():
            self.close()
            raise RuntimeError('Viewer process failed to initialize')
        self.update()

    def update(self):
        perturbation = None
        try:
            while True:
                perturbation = self._forces.get_nowait()
        except Empty:
            pass
        with self.world.lock:
            if perturbation is not None and np.isfinite(perturbation).all():
                self.world.data.xfrc_applied[:] = perturbation
            mujoco.mj_getState(self.world.model, self.world.data, self._state, self._spec)
        _latest(self._snapshots, self._state.copy())

    def is_running(self):
        return self._process.is_alive() and not self._closed.is_set()

    def close(self):
        self._stop.set()
        self._process.join(timeout=1.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
        for queue in (self._snapshots, self._forces):
            queue.cancel_join_thread()
            queue.close()


class ViewerMirror:
    """Display a state snapshot and return mouse perturbation forces to physics.

    MuJoCo's passive viewer may wait on its render thread in sync(). Keeping
    independent model/data prevents that wait, or GUI model edits, from blocking
    the simulation's sensors and physics.
    """

    def __init__(self, world: SimWorld) -> None:
        self.world = world
        with world.lock:
            self.model = copy.copy(world.model)
        self.data = mujoco.MjData(self.model)
        self._spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self._state = np.empty(mujoco.mj_stateSize(self.model, self._spec))
        self.update()

    def update(self) -> None:
        with self.world.lock:
            # sync() writes mouse perturbations into the mirror, not live data.
            self.world.data.xfrc_applied[:] = self.data.xfrc_applied
            mujoco.mj_getState(self.world.model, self.world.data, self._state, self._spec)
        mujoco.mj_setState(self.model, self.data, self._state, self._spec)
        mujoco.mj_forward(self.model, self.data)

    def sync(self, viewer) -> None:
        # Waiting for the GUI lock also happens outside the physics lock.
        with viewer.lock():
            self.update()
        viewer.sync()
