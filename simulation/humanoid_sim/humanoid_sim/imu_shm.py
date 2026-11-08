"""A lock-free IMU snapshot shared between the physics process and a publisher.

The physics thread writes one fixed-size record per step; a separate process
reads the latest record and publishes it.  A sequence counter makes the copy
atomic without taking the world lock, so the reader never blocks the simulation
and never observes a half-written sample.

Layout is fixed and little-endian so the two processes only share a byte range,
not a Python object.  Bytes on disk survive a crashed writer; a new run rewrites
the header before publishing, and the reader rejects records whose magic or
version do not match.
"""
from __future__ import annotations

import mmap
import os
import struct
import tempfile
import time

MAGIC = b'HIMU'
VERSION = 1
SIZE = 4096  # one page: the record is ~120 bytes, the rest is slack

# magic, version, seq, stamp_ns, mono, gyro[3], accel[3], quat[4], flags
_LAYOUT = '<4sIQ q d 3d 3d 4d I'
_RECORD = struct.calcsize(_LAYOUT)
assert _RECORD <= SIZE

FLAG_CONTROL_FAULT = 1 << 0
FLAG_VALID = 1 << 1


def candidate_paths(path: str | None = None) -> list:
    """Preferred segment locations, in order.

    XDG_RUNTIME_DIR is the right home for runtime state but can be missing or
    unwritable (containers, sandboxes), and a permission probe can disagree with
    the actual write policy. Callers therefore try each candidate instead of
    trusting os.access.
    """
    if path:
        return [path]
    candidates = []
    for base in (os.environ.get('HUMANOID_IMU_SNAPSHOT_DIR'),
                 os.environ.get('XDG_RUNTIME_DIR'), tempfile.gettempdir()):
        if base:
            candidate = os.path.join(base, 'humanoid_imu_snapshot.bin')
            if candidate not in candidates:
                candidates.append(candidate)
    return candidates


def default_path() -> str:
    """The preferred location; the writer falls back when it is unusable."""
    return candidate_paths()[0]


class ImuSnapshotWriter:
    """Own the shared file and write one record per physics step."""

    def __init__(self, path: str | None = None) -> None:
        last_error = None
        for candidate in candidate_paths(path):
            try:
                self._map, self.path = self._create(candidate)
                break
            except OSError as error:
                last_error = error
        else:
            raise OSError(f'cannot create IMU snapshot segment: {last_error}')
        self._seq = 0
        self._closed = False
        # Header first, then an even (quiescent) sequence number.
        struct.pack_into('<4sII', self._map, 0, MAGIC, VERSION, 0)

    @staticmethod
    def _create(path):
        directory = os.path.dirname(path) or '.'
        os.makedirs(directory, exist_ok=True)
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            if os.fstat(fd).st_size < SIZE:
                os.ftruncate(fd, SIZE)
            return mmap.mmap(fd, SIZE), path
        finally:
            os.close(fd)

    def write(self, stamp_ns: int, gyro, accel, quat, control_fault: bool) -> None:
        if self._closed:
            return
        # Odd sequence marks "write in progress"; readers retry instead of
        # returning a torn mix of two steps.
        self._seq += 1
        struct.pack_into('<Q', self._map, 8, self._seq)
        struct.pack_into(
            _LAYOUT, self._map, 0, MAGIC, VERSION, self._seq, int(stamp_ns),
            time.monotonic(), float(gyro[0]), float(gyro[1]), float(gyro[2]),
            float(accel[0]), float(accel[1]), float(accel[2]),
            float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]),
            (FLAG_CONTROL_FAULT if control_fault else 0) | FLAG_VALID)
        self._seq += 1
        struct.pack_into('<Q', self._map, 8, self._seq)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._map.flush()
        self._map.close()


class ImuSnapshotReader:
    """Read the newest record; retry on a torn read instead of blocking."""

    def __init__(self, path: str | None = None, timeout_s: float = 5.0,
                 should_stop=None) -> None:
        self.path = path
        self._candidates = candidate_paths(path)
        deadline = time.monotonic() + timeout_s
        self._map = None
        while time.monotonic() < deadline:
            # A blocked waiter must still honour shutdown: rclpy replaces SIGINT
            # with a context shutdown instead of raising, so poll for it.
            if should_stop is not None and should_stop():
                raise FileNotFoundError('IMU snapshot wait cancelled')
            for candidate in self._candidates:
                self._map = self._open(candidate)
                if self._map is not None:
                    self.path = candidate
                    return
            time.sleep(0.01)
        raise FileNotFoundError(f'IMU snapshot not available in {self._candidates}')

    @staticmethod
    def _open(path):
        try:
            fd = os.open(path, os.O_RDONLY)
        except OSError:
            return None
        try:
            if os.fstat(fd).st_size < SIZE:
                return None
            mapping = mmap.mmap(fd, SIZE, prot=mmap.PROT_READ)
            magic, version = struct.unpack_from('<4sI', mapping, 0)
            if magic != MAGIC or version != VERSION:
                mapping.close()
                return None
            return mapping
        finally:
            os.close(fd)

    def read(self):
        """Return the newest record as a dict, or None if none is available."""
        for _ in range(64):
            seq_before = struct.unpack_from('<Q', self._map, 8)[0]
            if seq_before & 1 or seq_before == 0:
                continue
            fields = struct.unpack_from(_LAYOUT, self._map, 0)
            seq_after = struct.unpack_from('<Q', self._map, 8)[0]
            if seq_before != seq_after:
                continue
            magic, version, _seq, stamp_ns, mono, *rest = fields
            if magic != MAGIC or version != VERSION:
                return None
            flags = rest[-1]
            return dict(
                stamp_ns=stamp_ns, written_mono=mono,
                gyro=rest[0:3], accel=rest[3:6], quat=rest[6:10],
                control_fault=bool(flags & FLAG_CONTROL_FAULT),
                valid=bool(flags & FLAG_VALID),
            )
        return None

    def close(self) -> None:
        if self._map is not None:
            self._map.close()
            self._map = None
