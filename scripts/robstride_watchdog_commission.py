#!/usr/bin/env python3
"""Commission the RobStride motor-side CAN watchdog (canTimeout, 0x7028) on ONE motor.

The motor never produces torque: every operation frame has kp = kd = 0, zero
velocity and zero feedforward torque, so it only elicits a status reply.

  1. Listen for --quiet-s: refuse if anything else is talking on the bus.
  2. DISABLE, read the current canTimeout (type 17).
  3. Write canTimeout = --timeout-s * 20000 (type 18) and read it back.
  4. ENABLE, then send zero-gain frames for 1 s: status must report Motor mode.
  5. Stay silent for a fraction of the timeout, then one zero-gain frame:
     the motor must still be in Motor mode (control).
  6. Stay silent for longer than the timeout, then one zero-gain frame:
     the motor must report reset mode (the watchdog fired).
  7. DISABLE and restore the original canTimeout (runtime value; nothing is
     saved with type 22).

Run with the robot's controller stopped, the motor mechanically free or
supported, and a hand on the power cut-off:

  python3 scripts/robstride_watchdog_commission.py --channel can1 --motor-id 4 \\
      --model rs-04 --physical

Without --physical only vcan interfaces are accepted.
"""
import argparse
import json
import struct
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'humanoid_control' / 'motor_control_hybrid'))

CAN_TIMEOUT = 0x7028
TICKS_PER_S = 20000


class Commission:
    def __init__(self, bus, motor_name, log=print):
        self.bus, self.motor, self.log = bus, motor_name, log
        self.id = bus.motors[motor_name].id

    def _recv_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.bus.channel_handler.recv(timeout=max(0.0, deadline - time.monotonic()))
            if frame is None:
                continue
            result = predicate(frame)
            if result is not None:
                return result
        return None

    def drain(self):
        """Drop queued frames so the next reply read belongs to the next request."""
        while self.bus.channel_handler.recv(timeout=0.0) is not None:
            pass

    def status(self, timeout=0.2):
        """Next type-2 status of this motor (MotorFeedback) or None."""
        def match(frame):
            try:
                sample = self.bus.decode_feedback(frame)
            except RuntimeError as exc:  # fault report
                raise SystemExit(f'motor fault: {exc}')
            return sample if sample is not None and sample.motor_name == self.motor else None
        return self._recv_until(match, timeout)

    def read(self, index, timeout=0.2):
        self.drain()
        self.bus.read_parameter(self.motor, index)

        def match(frame):
            reply = self.bus.decode_parameter_reply(frame)
            if reply is not None and reply.motor_name == self.motor and reply.index == index:
                return int.from_bytes(reply.value, 'little')
            return None
        return self._recv_until(match, timeout)

    def write_u32(self, index, value):
        self.bus.transmit(18, self.bus.host_id, self.id, struct.pack('<HHL', index, 0, value))

    def probe(self):
        """One zero-gain, zero-torque operation frame; returns the status reply."""
        position = self.last_position if self.last_position is not None else 0.0
        self.drain()
        self.bus.write_operation_frame(self.motor, position, 0.0, 0.0, 0.0, 0.0)
        return self.status()

    last_position = None

    def run(self, timeout_s, quiet_s):
        frame = self.bus.channel_handler.recv(timeout=quiet_s)
        if frame is not None:
            raise SystemExit(f'bus is not quiet (saw id 0x{frame.arbitration_id:08x}); '
                             'stop every other CAN user first')
        self.drain()
        self.bus.disable(self.motor)
        first = self.status()
        if first is None:
            raise SystemExit('no status reply to DISABLE: wrong motor id, host id or wiring')
        self.last_position = first.position
        self.log(f'DISABLE ok: status mode {first.mode}, position {first.position:+.3f} rad, '
                 f'{first.temperature:.1f} C')
        original = self.read(CAN_TIMEOUT)
        if original is None:
            raise SystemExit('no type-17 reply for canTimeout: firmware may not support parameter '
                             'reads; the verified ENABLE would never enable this motor')
        self.log(f'canTimeout before: {original} ({original / TICKS_PER_S:.3f} s)')
        ticks = round(timeout_s * TICKS_PER_S)
        result = dict(original=original, written=ticks)
        try:
            self.write_u32(CAN_TIMEOUT, ticks)
            readback = self.read(CAN_TIMEOUT)
            result['readback'] = readback
            self.log(f'canTimeout written {ticks}, read back {readback}')
            if readback != ticks:
                raise SystemExit('readback mismatch: the driver would refuse to enable this motor')
            self.bus.enable(self.motor)
            modes, positions = [], []
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                sample = self.probe()
                if sample is not None:
                    modes.append(sample.mode)
                    positions.append(sample.position)
                    self.last_position = sample.position
                time.sleep(0.02)
            if not modes or modes[-1] != 2:
                raise SystemExit(f'motor did not reach Motor mode after ENABLE (modes {sorted(set(modes))})')
            drift = max(positions) - min(positions)
            result['enabled_drift_rad'] = drift
            self.log(f'ENABLE ok: Motor mode, position drift {drift:.4f} rad over 1 s at zero gain')
            time.sleep(0.5 * timeout_s)
            control = self.probe()
            result['mode_after_short_silence'] = None if control is None else control.mode
            self.log(f'after {0.5 * timeout_s:.2f} s silence: mode '
                     f'{result["mode_after_short_silence"]} (expected 2: still running)')
            time.sleep(1.5 * timeout_s)
            fired = self.probe()
            result['mode_after_long_silence'] = None if fired is None else fired.mode
            self.log(f'after {1.5 * timeout_s:.2f} s silence: mode '
                     f'{result["mode_after_long_silence"]} (expected 0: reset by the watchdog)')
        finally:
            self.bus.disable(self.motor)
            self.status()
            self.write_u32(CAN_TIMEOUT, original)
            restored = self.read(CAN_TIMEOUT)
            result['restored'] = restored
            self.log(f'DISABLE sent; canTimeout restored to {restored}')
        result['pass'] = (result.get('mode_after_short_silence') == 2
                          and result.get('mode_after_long_silence') == 0
                          and result.get('restored') == original)
        return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--channel', required=True)
    ap.add_argument('--motor-id', type=int, required=True)
    ap.add_argument('--model', required=True, choices=['rs-02', 'rs-03', 'rs-04'])
    ap.add_argument('--host-id', type=int, default=255, help='master_id in motors.yaml')
    ap.add_argument('--timeout-s', type=float, default=1.0)
    ap.add_argument('--quiet-s', type=float, default=1.0)
    ap.add_argument('--physical', action='store_true', help='allow a real (non-vcan) interface')
    args = ap.parse_args()
    if not 0 < args.timeout_s <= 5:
        sys.exit('--timeout-s must be in (0, 5]')
    info = json.loads(subprocess.check_output(['ip', '-details', '-json', 'link', 'show', args.channel]))
    kind = info[0].get('linkinfo', {}).get('info_kind')
    if kind != 'vcan' and not args.physical:
        sys.exit(f'{args.channel} is {kind}, not vcan: pass --physical for a real motor')
    from robstride_dynamics import Motor
    from motor_control_hybrid.can_transport import NonblockingRobstrideBus
    name = f'motor_{args.motor_id}'
    bus = NonblockingRobstrideBus(args.channel, {name: Motor(args.motor_id, args.model)}, {})
    bus.host_id = args.host_id
    bus.connect(handshake=False)
    try:
        result = Commission(bus, name).run(args.timeout_s, args.quiet_s)
    finally:
        bus.disconnect(disable_torque=True)
    print(json.dumps(result))
    print('PASS' if result['pass'] else 'FAIL')
    sys.exit(0 if result['pass'] else 1)


if __name__ == '__main__':
    main()
