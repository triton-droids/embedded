"""Unit tests for the shared-segment IMU snapshot."""
import struct

import pytest

from humanoid_sim.imu_shm import (
    FLAG_CONTROL_FAULT,
    MAGIC,
    VERSION,
    ImuSnapshotReader,
    ImuSnapshotWriter,
)


def _sample(writer, stamp=123, gyro=(0.1, 0.2, 0.3), fault=False):
    writer.write(stamp, gyro, (0.0, 0.0, 9.81), (1.0, 0.0, 0.0, 0.0), fault)


def test_round_trip_preserves_every_field(tmp_path):
    path = str(tmp_path / 'imu.bin')
    writer = ImuSnapshotWriter(path)
    reader = ImuSnapshotReader(path, timeout_s=1.0)
    try:
        _sample(writer, stamp=987654321, gyro=(1.5, -2.5, 3.5))
        record = reader.read()
        assert record['stamp_ns'] == 987654321
        assert record['gyro'] == pytest.approx((1.5, -2.5, 3.5))
        assert record['accel'] == pytest.approx((0.0, 0.0, 9.81))
        assert record['quat'] == pytest.approx((1.0, 0.0, 0.0, 0.0))
        assert record['valid'] is True
        assert record['control_fault'] is False
    finally:
        reader.close()
        writer.close()


def test_control_fault_flag_is_reported(tmp_path):
    path = str(tmp_path / 'imu.bin')
    writer = ImuSnapshotWriter(path)
    reader = ImuSnapshotReader(path, timeout_s=1.0)
    try:
        _sample(writer, fault=True)
        assert reader.read()['control_fault'] is True
    finally:
        reader.close()
        writer.close()


def test_reader_reports_nothing_before_the_first_write(tmp_path):
    path = str(tmp_path / 'imu.bin')
    writer = ImuSnapshotWriter(path)
    reader = ImuSnapshotReader(path, timeout_s=1.0)
    try:
        assert reader.read() is None
    finally:
        reader.close()
        writer.close()


def test_torn_write_is_retried_not_returned(tmp_path):
    """A reader that catches an odd sequence must not mix two steps."""
    path = str(tmp_path / 'imu.bin')
    writer = ImuSnapshotWriter(path)
    reader = ImuSnapshotReader(path, timeout_s=1.0)
    try:
        _sample(writer, stamp=1)
        # Simulate a writer caught mid-update: odd sequence, half-written bodies.
        struct.pack_into('<Q', writer._map, 8, 3)
        struct.pack_into('<q', writer._map, 16, 999999)
        assert reader.read() is None
        # Once the writer completes, the next record is visible again.
        _sample(writer, stamp=2)
        assert reader.read()['stamp_ns'] == 2
    finally:
        reader.close()
        writer.close()


def test_header_identifies_the_segment(tmp_path):
    path = str(tmp_path / 'imu.bin')
    writer = ImuSnapshotWriter(path)
    reader = ImuSnapshotReader(path, timeout_s=1.0)
    try:
        magic, version = struct.unpack_from('<4sI', reader._map, 0)
        assert magic == MAGIC
        assert version == VERSION
        # flags sit after magic+version+seq+stamp+mono+gyro+accel+quat
        flags = struct.unpack_from('<I', reader._map, struct.calcsize('<4sIQ q d 3d 3d 4d'))[0]
        assert flags == 0
    finally:
        reader.close()
        writer.close()


def test_missing_segment_raises_instead_of_hanging(tmp_path):
    with pytest.raises(FileNotFoundError):
        ImuSnapshotReader(str(tmp_path / 'absent.bin'), timeout_s=0.2)
