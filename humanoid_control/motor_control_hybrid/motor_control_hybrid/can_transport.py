"""Non-waiting RobStride TX and independently decoded CAN feedback.

The adapter owns audited MIT encoding for the deployed RS02/03/04 profiles. This
adapter changes I/O ownership: one TX worker and one RX worker per interface.
An enable/disable return means submitted to the host CAN stack, not acknowledged
by the physical motor. No calibration, protocol change or zeroing is issued.
"""
from dataclasses import dataclass
import struct
import math
import threading
import queue

import can
from robstride_dynamics import RobstrideBus, CommunicationType
# CAN2.0 private protocol, pinned official manuals; see docs/can_protocol_audit.md.
# (position magnitude, velocity magnitude, torque magnitude, kp max, kd max)
PROTOCOL_PROFILES = {
    'rs-02': (4 * math.pi, 44.0, 17.0, 500.0, 5.0),  # 2026-07-13
    'rs-03': (4 * math.pi, 20.0, 60.0, 5000.0, 100.0),  # 2026-10-13
    'rs-04': (4 * math.pi, 15.0, 120.0, 5000.0, 100.0),  # 2026-07-13
}



@dataclass(frozen=True)
class MotorFeedback:
    motor_name: str
    position: float
    velocity: float
    torque: float
    temperature: float
    mode: int
    fault_bits: int
    timestamp: float


class NonblockingRobstrideBus(RobstrideBus):
    """Audited private-protocol encoding, zero-wait TX and one RX owner."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if any(motor.model not in PROTOCOL_PROFILES for motor in self.motors.values()):
            raise ValueError('CAN model lacks an audited protocol profile')
        self.tx_frames = 0
        self.tx_errors = 0

    def connect(self, handshake=True):
        if self.channel_handler is not None:
            raise RuntimeError('CAN interface already open')
        self.channel_handler = can.ThreadSafeBus(
            interface='socketcan', channel=self.channel, bitrate=self.bitrate,
            receive_own_messages=False,
            can_filters=[{'can_id': (kind << 24) | self.host_id,
                          'can_mask': (31 << 24) | 255, 'extended': True}
                         for kind in (CommunicationType.OPERATION_STATUS, CommunicationType.FAULT_REPORT)])
        self.channel_handler.socket.setblocking(False)
        self._motor_by_id = {motor.id: name for name, motor in self.motors.items()}

    def transmit(self, communication_type, extra_data, device_id, data=bytes(8)):
        if (any(type(value) is not int for value in (communication_type, extra_data, device_id))
                or not 0 <= communication_type <= 31 or not 0 <= extra_data <= 65535
                or not 1 <= device_id <= 254 or len(data) != 8):
            raise ValueError('Invalid RobStride frame')
        frame = can.Message(
            arbitration_id=(communication_type << 24) | (extra_data << 8) | device_id,
            is_extended_id=True, data=data)
        # Backpressure raises immediately; it must never wait behind RX or turn
        # an old setpoint into a delayed retransmission inside the transport.
        try:
            self.channel_handler.send(frame, timeout=0.0)
            self.tx_frames += 1
        except can.CanError:
            self.tx_errors += 1
            raise

    def write_operation_frame(self, motor, position, kp, kd, velocity=0, torque=0):
        model = self.motors[motor].model
        if model not in PROTOCOL_PROFILES:
            raise ValueError(f'No audited CAN profile for {model}')
        if not all(math.isfinite(value) for value in (position, velocity, torque, kp, kd)):
            raise ValueError('MIT values must be finite')
        if self.calibration:
            calibration = self.calibration[motor]
            position = position * calibration['direction'] + calibration['homing_offset']
            velocity *= calibration['direction']
            torque *= calibration['direction']
        pmax, vmax, tmax, kpmax, kdmax = PROTOCOL_PROFILES[model]
        def encode(value, low, high):
            value = max(low, min(high, value))
            return int((value - low) * 65535.0 / (high - low))
        data = struct.pack('>HHHH', encode(position, -pmax, pmax), encode(velocity, -vmax, vmax),
                           encode(kp, 0, kpmax), encode(kd, 0, kdmax))
        self.transmit(CommunicationType.OPERATION_CONTROL, encode(torque, -tmax, tmax), self.motors[motor].id, data)

    def enable(self, motor):
        self.transmit(CommunicationType.ENABLE, self.host_id, self.motors[motor].id)

    def disable(self, motor):
        self.transmit(CommunicationType.DISABLE, self.host_id, self.motors[motor].id)

    def disconnect(self, disable_torque=True):
        if self.channel_handler is None:
            return
        try:
            if disable_torque:
                for name in self.motors:
                    self.disable(name)
        finally:
            self.channel_handler.shutdown()
            self.channel_handler = None

    def decode_feedback(self, frame):
        """Route by reported motor ID before applying that motor's SI scales.

        Reject unrelated hosts, unknown devices, malformed frames and parameter
        responses. Position/velocity/torque are in raw motor rad, rad/s and Nm;
        polarity and linkage conversion remain in the node's hardware adapter.
        """
        if frame.is_error_frame:
            raise RuntimeError('SocketCAN error frame')
        if (not frame.is_extended_id or frame.is_remote_frame or frame.is_fd
                or not 0 <= frame.arbitration_id <= 0x1FFFFFFF or frame.dlc != 8 or len(frame.data) != 8):
            return None
        kind = (frame.arbitration_id >> 24) & 31
        extra = (frame.arbitration_id >> 8) & 65535
        if frame.arbitration_id & 255 != self.host_id:
            return None
        name = self._motor_by_id.get(extra & 255)
        if name is None:
            return None
        if kind == CommunicationType.FAULT_REPORT:
            raise RuntimeError(f'{name}: motor fault report {bytes(frame.data).hex()}')
        if kind != CommunicationType.OPERATION_STATUS:
            return None
        # A firmware-query reply also uses type 2. Its management payload must
        # never be scaled into plausible joint angles or satisfy an enable ack.
        if bytes(frame.data[:3]) == b'\x00\xc4\x56':
            return None
        model = self.motors[name].model
        if model not in PROTOCOL_PROFILES:
            raise ValueError(f'No audited CAN profile for {model}')
        pmax, vmax, tmax, _, _ = PROTOCOL_PROFILES[model]
        p, v, t, temperature = struct.unpack('>HHHH', frame.data)
        return MotorFeedback(
            name, (p / 65535.0 * 2 - 1) * pmax,
            (v / 65535.0 * 2 - 1) * vmax,
            (t / 65535.0 * 2 - 1) * tmax, temperature * 0.1,
            (extra >> 14) & 3, (extra >> 8) & 63, frame.timestamp)


class LatestCommandMailbox:
    """Bounded latest targets, retaining the latest lifecycle request before motion.

    At most two entries per configured joint: enable/disable and latest target.
    Disable discards pending motion; motion after a pending disable is ignored
    until an explicit enable. The worker drains a bounded batch each pass.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = {}

    def put_nowait(self, command):
        commands = command if isinstance(command, list) else [command]
        with self._lock:
            # Install one ROS message atomically, including all ten joints.
            for command in commands:
                joint, mode = command['joint'], command['mode']
                lifecycle, motion = self._pending.get(joint, (None, None))
                if mode in (3, 4):
                    lifecycle, motion = command, None
                elif lifecycle is None or lifecycle['mode'] != 4:
                    motion = command
                self._pending[joint] = (lifecycle, motion)

    def get_nowait(self):
        with self._lock:
            if not self._pending:
                raise queue.Empty
            joint = next(iter(self._pending))
            lifecycle, motion = self._pending[joint]
            if lifecycle is not None:
                if motion is None:
                    del self._pending[joint]
                else:
                    self._pending[joint] = (None, motion)
                return lifecycle
            del self._pending[joint]
            return motion

    def clear(self):
        with self._lock:
            self._pending.clear()

    def empty(self):
        return self.qsize() == 0

    def qsize(self):
        with self._lock:
            return sum((life is not None) + (motion is not None)
                       for life, motion in self._pending.values())
