import math

import numpy as np

from attitude_sensing_pkg import imu_read


def test_parse_bno085_csv_converts_to_ros_units():
    sample = imu_read.parse_imu_line(
        "1234,0.0,0.5,1.0,180.0,-90.0,0.0,10.0,-20.0",
        input_format="bno085_csv",
    )

    assert sample is not None
    assert sample["source"] == "bno085_csv"
    assert sample["sensor_time_s"] == 1.234
    np.testing.assert_allclose(sample["acc_m_s2"], [0.0, 4.903325, 9.80665])
    np.testing.assert_allclose(sample["gyro_rad_s"], [math.pi, -math.pi / 2.0, 0.0])
    np.testing.assert_allclose(sample["sensor_rpy_deg"], [10.0, -20.0, 0.0])
    assert math.isclose(np.linalg.norm(sample["sensor_quat_wb"]), 1.0)
    assert sample["sensor_orientation_partial"] is True


def test_auto_detects_existing_json_format():
    sample = imu_read.parse_imu_line(
        '{"acc":[0,0,1],"gyro":[0,0,180]}',
        input_format="auto",
        acc_units="g",
        gyro_units="deg/s",
    )

    assert sample is not None
    assert sample["source"] == "json"
    np.testing.assert_allclose(sample["acc_m_s2"], [0.0, 0.0, 9.80665])
    np.testing.assert_allclose(sample["gyro_rad_s"], [0.0, 0.0, math.pi])


def test_bno085_status_and_malformed_lines_are_ignored():
    assert imu_read.parse_imu_line("serial_ok", input_format="auto") is None
    assert imu_read.parse_imu_line("BNO085 not found", input_format="auto") is None
    assert imu_read.parse_imu_line("1,2,3", input_format="bno085_csv") is None


def test_serial_generator_uses_bno085_board_timestamps(monkeypatch):
    class FakeSerial:
        def __init__(self):
            self.lines = iter([
                b"1000,0,0,1,0,0,0,0,0\n",
                b"1005,0,0,1,0,0,0,0,0\n",
            ])
            self.closed = False

        def readline(self):
            return next(self.lines)

        def close(self):
            self.closed = True

    fake_serial = FakeSerial()
    monkeypatch.setattr(
        imu_read.serial,
        "Serial",
        lambda *_args, **_kwargs: fake_serial,
    )
    samples = imu_read.iter_imu_samples(
        input_format="bno085_csv",
        include_all=False,
    )

    assert next(samples)["dt"] == 0.0
    assert math.isclose(next(samples)["dt"], 0.005)
    samples.close()
    assert fake_serial.closed is True
