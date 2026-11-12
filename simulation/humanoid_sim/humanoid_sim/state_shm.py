"""Kinematic state shared from the physics process to an out-of-process renderer.

The physics thread writes qpos and the mocap poses after every step; the
renderer (``sim_stereo_publisher``) maps the same file, rebuilds the poses with
``mj_kinematics`` on its own copy of the model, and renders. Rendering and image
handling then never hold this process's GIL or world lock.

The model travels as a MuJoCo binary (``*.mjb``) written next to the state file
at startup, so both processes use exactly the same compiled model. Both file
names carry ROS_DOMAIN_ID, so simulations on different domains stay apart.

Same sequence-counter scheme as :mod:`humanoid_sim.imu_shm`: an odd counter
marks a write in progress and the reader retries. Layout is little-endian:

  magic[4] version:u32 seq:u64 stamp_ns:i64 nq:u32 nmocap:u32 | float64 qpos[nq],
  mocap_pos[3*nmocap], mocap_quat[4*nmocap]
"""
from __future__ import annotations

import mmap
import os
import struct
import tempfile
import time

import numpy as np

MAGIC = b'HQST'
VERSION = 1
_HEADER = '<4sIQqII'
_HEADER_SIZE = struct.calcsize(_HEADER)  # 32, so the float64 block is aligned.
_SEQ_OFFSET = 8
_STAMP_OFFSET = 16



def _names() -> tuple[str, str]:
    # Per ROS domain: two simulations on one host must not share a segment.
    domain = os.environ.get('ROS_DOMAIN_ID', '0') or '0'
    return f'humanoid_stereo_state_{domain}.bin', f'humanoid_stereo_model_{domain}.mjb'


def candidate_dirs(directory: str | None = None) -> list:
    if directory:
        return [directory]
    dirs = []
    for base in (os.environ.get('HUMANOID_IMU_SNAPSHOT_DIR'),
                 os.environ.get('XDG_RUNTIME_DIR'), tempfile.gettempdir()):
        if base and base not in dirs:
            dirs.append(base)
    return dirs


def _size(nq: int, nmocap: int) -> int:
    needed = _HEADER_SIZE + 8 * (nq + 7 * nmocap)
    return max(mmap.PAGESIZE, -(-needed // mmap.PAGESIZE) * mmap.PAGESIZE)


class StateWriter:
    """Own the state file and the model file; write one record per physics step."""

    def __init__(self, model, directory: str | None = None) -> None:
        import mujoco
        self.nq, self.nmocap = int(model.nq), int(model.nmocap)
        size = _size(self.nq, self.nmocap)
        last_error = None
        for candidate in candidate_dirs(directory):
            try:
                os.makedirs(candidate, exist_ok=True)
                state_name, model_name = _names()
                state_path = os.path.join(candidate, state_name)
                model_path = os.path.join(candidate, model_name)
                tmp = model_path + f'.{os.getpid()}.tmp'
                mujoco.mj_saveModel(model, tmp, None)
                os.replace(tmp, model_path)  # Readers never see a partial model.
                fd = os.open(state_path, os.O_RDWR | os.O_CREAT, 0o600)
                try:
                    os.ftruncate(fd, size)
                    self._map = mmap.mmap(fd, size)
                finally:
                    os.close(fd)
                self.path, self.model_path = state_path, model_path
                break
            except OSError as error:
                last_error = error
        else:
            raise OSError(f'cannot create stereo state segment: {last_error}')
        n = self.nq + 7 * self.nmocap
        self._values = np.frombuffer(self._map, dtype='<f8', count=n, offset=_HEADER_SIZE)
        self._seq = 0
        self._closed = False
        struct.pack_into(_HEADER, self._map, 0, MAGIC, VERSION, 0, 0, self.nq, self.nmocap)

    def write(self, stamp_ns: int, qpos, mocap_pos, mocap_quat) -> None:
        if self._closed:
            return
        self._seq += 1
        struct.pack_into('<Q', self._map, _SEQ_OFFSET, self._seq)
        nq, nm = self.nq, self.nmocap
        self._values[:nq] = qpos
        if nm:
            self._values[nq:nq + 3 * nm] = np.ravel(mocap_pos)
            self._values[nq + 3 * nm:] = np.ravel(mocap_quat)
        struct.pack_into('<q', self._map, _STAMP_OFFSET, int(stamp_ns))
        self._seq += 1
        struct.pack_into('<Q', self._map, _SEQ_OFFSET, self._seq)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        del self._values
        self._map.close()


class StateReader:
    """Map the state written by :class:`StateWriter`; wait for it to appear."""

    def __init__(self, directory: str | None = None, timeout_s: float = 30.0,
                 should_stop=None) -> None:
        deadline = time.monotonic() + timeout_s
        while True:
            if should_stop is not None and should_stop():
                raise FileNotFoundError('stereo state wait cancelled')
            for candidate in candidate_dirs(directory):
                if self._open(candidate):
                    return
            if time.monotonic() > deadline:
                raise FileNotFoundError(f'stereo state not available in {candidate_dirs(directory)}')
            time.sleep(0.05)

    def _open(self, directory: str) -> bool:
        state_name, model_name = _names()
        state_path = os.path.join(directory, state_name)
        model_path = os.path.join(directory, model_name)
        try:
            fd = os.open(state_path, os.O_RDONLY)
        except OSError:
            return False
        try:
            size = os.fstat(fd).st_size
            if size < _HEADER_SIZE or not os.path.exists(model_path):
                return False
            mapping = mmap.mmap(fd, size, prot=mmap.PROT_READ)
        finally:
            os.close(fd)
        magic, version, _seq, _stamp, nq, nmocap = struct.unpack_from(_HEADER, mapping, 0)
        if magic != MAGIC or version != VERSION or size < _size(nq, nmocap):
            mapping.close()
            return False
        self._map, self.nq, self.nmocap = mapping, nq, nmocap
        self.path, self.model_path = state_path, model_path
        self._values = np.frombuffer(mapping, dtype='<f8', count=nq + 7 * nmocap, offset=_HEADER_SIZE)
        return True

    def read(self):
        """Newest (stamp_ns, qpos, mocap_pos, mocap_quat) copy, or None before the first step."""
        for _ in range(64):
            before = struct.unpack_from('<Q', self._map, _SEQ_OFFSET)[0]
            if before & 1 or before == 0:
                continue
            values = self._values.copy()
            stamp_ns = struct.unpack_from('<q', self._map, _STAMP_OFFSET)[0]
            if struct.unpack_from('<Q', self._map, _SEQ_OFFSET)[0] != before:
                continue
            nq, nm = self.nq, self.nmocap
            return (stamp_ns, values[:nq], values[nq:nq + 3 * nm].reshape(nm, 3),
                    values[nq + 3 * nm:].reshape(nm, 4))
        return None

    def close(self) -> None:
        if getattr(self, '_map', None) is not None:
            del self._values
            self._map.close()
            self._map = None
