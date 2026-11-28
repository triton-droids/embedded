"""scripts/robstride_watchdog_commission.py against a simulated motor (python-can virtual bus).

The fake motor stores type-18 writes, answers type-17 reads, and implements the
manual's canTimeout rule: in Motor mode, no command for canTimeout/20000 s puts
it in reset mode. No frame it receives carries torque (kp = kd = 0 is checked).
"""
import importlib.util
import struct
import threading
import time
import uuid
from pathlib import Path

import can
import pytest
from robstride_dynamics import Motor

from motor_control_hybrid.can_transport import NonblockingRobstrideBus

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / 'robstride_watchdog_commission.py'
spec = importlib.util.spec_from_file_location('commission', SCRIPT)
commission = importlib.util.module_from_spec(spec)
spec.loader.exec_module(commission)


class WatchdogMotor:
    def __init__(self, channel, device=4, host=255, supports_reads=True, honour_timeout=True):
        self.bus = can.Bus(interface='virtual', channel=channel)
        self.device, self.host = device, host
        self.supports_reads, self.honour_timeout = supports_reads, honour_timeout
        self.params = {0x7028: (0).to_bytes(4, 'little')}
        self.mode, self.last_command = 0, time.monotonic()
        self.torque_frames = 0
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def status(self):
        extra = (self.mode << 14) | self.device
        self.bus.send(can.Message(arbitration_id=(2 << 24) | (extra << 8) | self.host,
                                  is_extended_id=True, data=struct.pack('>HHHH', 32767, 32767, 32767, 300)))

    def loop(self):
        while self.running:
            msg = self.bus.recv(timeout=0.01)
            now = time.monotonic()
            timeout = int.from_bytes(self.params[0x7028], 'little') / 20000
            if self.honour_timeout and self.mode == 2 and timeout and now - self.last_command > timeout:
                self.mode = 0
            if msg is None or (msg.arbitration_id & 255) != self.device:
                continue
            kind = (msg.arbitration_id >> 24) & 31
            index = struct.unpack('<H', bytes(msg.data[:2]))[0]
            if self.mode == 2 and timeout and now - self.last_command > timeout and self.honour_timeout:
                self.mode = 0
            self.last_command = now
            if kind == 18:
                self.params[index] = bytes(msg.data[4:8])
            elif kind == 17 and self.supports_reads:
                self.bus.send(can.Message(arbitration_id=(17 << 24) | (self.device << 8) | self.host,
                                          is_extended_id=True,
                                          data=bytes(msg.data[:4]) + self.params.get(index, bytes(4))))
            elif kind == 3:
                self.mode = 2
                self.status()
            elif kind == 4:
                self.mode = 0
                self.status()
            elif kind == 1:
                _, _, kp, kd = struct.unpack('>HHHH', msg.data)
                self.torque_frames += int(kp != 0 or kd != 0 or ((msg.arbitration_id >> 8) & 0xFFFF) != 32767)
                self.status()

    def stop(self):
        self.running = False
        self.thread.join(1)
        self.bus.shutdown()


@pytest.fixture
def setup(request):
    channel = f'commission-{uuid.uuid4()}'
    motor = WatchdogMotor(channel, **getattr(request, 'param', {}))
    bus = NonblockingRobstrideBus('virtual', {'motor_4': Motor(4, 'rs-04')}, {})
    bus._motor_by_id = {4: 'motor_4'}
    bus.host_id = 255
    bus.channel_handler = can.Bus(interface='virtual', channel=channel)
    yield motor, commission.Commission(bus, 'motor_4', log=lambda *_: None)
    bus.channel_handler.shutdown()
    motor.stop()


def test_watchdog_fires_after_silence_and_settings_are_restored(setup):
    motor, run = setup
    result = run.run(timeout_s=0.3, quiet_s=0.1)
    assert result['pass'], result
    assert result['readback'] == 6000 and result['restored'] == 0
    assert result['mode_after_short_silence'] == 2 and result['mode_after_long_silence'] == 0
    assert motor.torque_frames == 0  # never a frame that could produce torque
    assert motor.mode == 0  # left disabled


@pytest.mark.parametrize('setup', [dict(honour_timeout=False)], indirect=True)
def test_reports_fail_when_the_firmware_ignores_can_timeout(setup):
    _, run = setup
    result = run.run(timeout_s=0.3, quiet_s=0.1)
    assert not result['pass'] and result['mode_after_long_silence'] == 2


@pytest.mark.parametrize('setup', [dict(supports_reads=False)], indirect=True)
def test_stops_before_enabling_when_parameter_reads_are_unsupported(setup):
    motor, run = setup
    with pytest.raises(SystemExit, match='type-17'):
        run.run(timeout_s=0.3, quiet_s=0.1)
    assert motor.mode == 0
