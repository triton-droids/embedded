#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sinusoidal knee sweep for RobStride motors 4 and 9 (MIT mode 0)

Goal:
- Motor 4 and 9 move sinusoidally between joint limits:
    0 rad  <->  -1.57 rad  (~ -90 deg)
- They are opposite ends (phase-shifted by pi):
    when motor4 = 0, motor9 = -1.57, and vice versa
- Slow motion (default period = 50s)

Run:
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set up can0
  python3 sine_knees_4_9_opposed.py
"""

import os
import sys
import time
import math
import struct
import signal

# ---- RobStride SDK imports (same pattern as your tuner) ----
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
except ImportError:
    from bus import RobstrideBus, Motor
    from protocol import ParameterType, CommunicationType


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# --------- CONFIG ---------
CHANNEL = "can0"
BITRATE = 1_000_000

# Motors
M4 = 4
M9 = 9

# Models (both are RS04 in your list)
MOTOR_MODEL_BY_ID = {
    M4: "rs-04",
    M9: "rs-04",
}

# Direction (from your inversion array: IDs 1..5 are +1, 6..10 are -1)
DIR_BY_ID = {
    M4: +1,
    M9: -1,
}

# Logical joint limits (radians)
LIM_LO = -1.57
LIM_HI = 0.0

# Sine parameters (slow)
FREQ_HZ = 0.02  # 0.02 Hz => 50s period (slow). Try 0.01 for even slower.
CENTER = 0.5 * (LIM_LO + LIM_HI)         # -0.785 rad
AMP = 0.5 * (LIM_HI - LIM_LO)            # 0.785 rad  => range [-1.57, 0]

# Control loop rate
CTRL_HZ = 50.0
DT = 1.0 / CTRL_HZ

# MIT gains (keep modest; adjust as needed)
KP = 10.0
KD = 0.2

# Optional extra safety: limit how fast we change commanded position (logical)
# (keeps motion gentle even if you accidentally increase FREQ_HZ)
MAX_VEL_RAD_S = 0.15  # ~8.6 deg/s
MAX_STEP = MAX_VEL_RAD_S * DT


def set_mode_raw(bus: RobstrideBus, motor_name: str, mode: int):
    """Force MODE parameter (int8) to mode=0 for MIT."""
    device_id = bus.motors[motor_name].id
    param_id, _, _ = ParameterType.MODE
    value_buffer = struct.pack("<bBH", int(mode), 0, 0)
    data = struct.pack("<HH", param_id, 0x00) + value_buffer
    bus.transmit(CommunicationType.WRITE_PARAMETER, bus.host_id, device_id, data)
    time.sleep(0.05)


def main():
    motors = {
        f"motor_{M4}": Motor(id=M4, model=MOTOR_MODEL_BY_ID[M4]),
        f"motor_{M9}": Motor(id=M9, model=MOTOR_MODEL_BY_ID[M9]),
    }
    calibration = {
        f"motor_{M4}": {"direction": 1, "homing_offset": 0.0},
        f"motor_{M9}": {"direction": 1, "homing_offset": 0.0},
    }

    bus = None
    running = True

    def shutdown(_sig=None, _frame=None):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        try:
            bus = RobstrideBus(CHANNEL, motors, calibration, bitrate=BITRATE)
        except TypeError:
            bus = RobstrideBus(CHANNEL, motors, calibration)

        print(f"Connecting to {CHANNEL} (bitrate={BITRATE}) ...")
        bus.connect(handshake=True)

        # Enable + MIT mode
        for mid in (M4, M9):
            name = f"motor_{mid}"
            print(f"Enabling motor {mid} ...")
            bus.enable(name)
            time.sleep(0.2)
            print(f"Setting motor {mid} to MIT mode (MODE=0) ...")
            set_mode_raw(bus, name, 0)

        # Prime reads + start from current pose (no jump)
        state = {}
        for mid in (M4, M9):
            name = f"motor_{mid}"
            pos, vel, tq, temp = bus.read_operation_frame(name)
            # Convert physical -> logical using your direction convention
            logical_pos = pos / float(DIR_BY_ID[mid])
            state[mid] = {
                "pos_phys": pos,
                "cmd_logical": logical_pos,  # we will ramp from here to sine target
            }
            # Hold at current pose
            bus.write_operation_frame(name, pos, KP, KD, 0.0, 0.0)
            time.sleep(0.05)

        print("\nRunning sine sweep:")
        print(f"  Range: [{LIM_LO:.3f}, {LIM_HI:.3f}] rad  ([-90deg, 0deg])")
        print(f"  Freq : {FREQ_HZ:.3f} Hz  (period {1.0/FREQ_HZ:.1f} s)")
        print("Press Ctrl+C to stop.\n")

        t0 = time.time()
        next_t = time.time()

        while running:
            now = time.time()
            if now < next_t:
                time.sleep(max(0.0, next_t - now))
                continue
            next_t += DT

            t = now - t0
            s = math.sin(2.0 * math.pi * FREQ_HZ * t)

            # Opposed endpoints:
            # motor4 goes CENTER + AMP*s  in [-1.57, 0]
            # motor9 goes CENTER - AMP*s  in [-1.57, 0]
            target4 = clamp(CENTER + AMP * s, LIM_LO, LIM_HI)
            target9 = clamp(CENTER - AMP * s, LIM_LO, LIM_HI)

            # Gentle ramp in logical space
            for mid, target in ((M4, target4), (M9, target9)):
                cmd = state[mid]["cmd_logical"]
                delta = target - cmd
                if abs(delta) <= MAX_STEP:
                    cmd = target
                else:
                    cmd = cmd + math.copysign(MAX_STEP, delta)
                # clamp again
                cmd = clamp(cmd, LIM_LO, LIM_HI)
                state[mid]["cmd_logical"] = cmd

            # Send to motors (logical -> physical using direction)
            cmd4_phys = state[M4]["cmd_logical"] * float(DIR_BY_ID[M4])
            cmd9_phys = state[M9]["cmd_logical"] * float(DIR_BY_ID[M9])

            bus.write_operation_frame(f"motor_{M4}", cmd4_phys, KP, KD, 0.0, 0.0)
            bus.write_operation_frame(f"motor_{M9}", cmd9_phys, KP, KD, 0.0, 0.0)

            # Optional: read occasionally (not every tick) to reduce bus load
            # (uncomment if you want telemetry)
            # if int(t * CTRL_HZ) % 10 == 0:
            #     for mid in (M4, M9):
            #         pos, vel, tq, temp = bus.read_operation_frame(f"motor_{mid}")
            #         state[mid]["pos_phys"] = pos

    finally:
        print("\nStopping...")
        try:
            if bus is not None:
                # Hold where they are briefly, then disable
                for mid in (M4, M9):
                    name = f"motor_{mid}"
                    try:
                        pos, vel, tq, temp = bus.read_operation_frame(name)
                        bus.write_operation_frame(name, pos, KP, KD, 0.0, 0.0)
                    except Exception:
                        pass
                time.sleep(0.2)
                for mid in (M4, M9):
                    try:
                        bus.disable(f"motor_{mid}")
                    except Exception:
                        pass
                try:
                    bus.disconnect()
                except Exception:
                    pass
        except Exception:
            pass
        print("Done.")


if __name__ == "__main__":
    main()
