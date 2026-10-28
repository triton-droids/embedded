"""A viewer mirror whose GUI synchronization never holds the physics lock."""
from __future__ import annotations

import copy

import mujoco
import numpy as np

from .world import SimWorld


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
