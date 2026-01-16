#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sinusoidal knee sweep for RobStride motors 4/9 + coupled ankle sweep for 5/10 (MIT mode 0)

Goal:
- Knee motors 4 and 9 move sinusoidally between:
    0 rad  <->  -1.57 rad  (~ -90 deg)
- They are opposite ends (phase-shifted by pi):
    when motor4 = 0, motor9 = -1.57, and vice versa
- Ankle motors 5 and 10 are *coupled* to their respective knee:
    If knee (4 or 9) = 0 deg   -> ankle (5 or 10) = -30 deg
    If knee (4 or 9) = -90 deg -> ankle (5 or 10) = 0 deg
  i.e., ankle range is [-30, 0] deg.

Config requested:
- FREQ_HZ = 0.3
- MAX_VEL_RAD_S = 1.8

Run:
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set up can0
  python3 sine_knees_4_9_ankles_5_10_coupled.py
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
M4, M5, M9, M10 = 4, 5, 9, 10
MOTORS = (M4, M5, M9, M10)

# Models (from your list)
MOTOR_MODEL_BY_ID = {
    M4: "rs-04",
    M5: "rs-02",
    M9: "rs-04",
    M10: "rs-02",
}

# Direction (from your inversion array: IDs 1..5 are +1, 6..10 are -1)
DIR_BY_ID = {
    M4: +1,
    M5: +1,
    M9: -1,
    M10: -1,
}

# Knee logical limits (radians) for this test
KNEE_LO = -1.57
KNEE_HI = 0.0

# Ankle logical limits (radians): [-30deg, 0deg]
ANKLE_LO = -math.radians(30.0)   # -0.523599...
ANKLE_HI = 0.0

# Sine parameters
FREQ_HZ = 0.3
KNEE_CENTER = 0.5 * (KNEE_LO + KNEE_HI)   # -0.785 rad
KNEE_AMP = 0.5 * (KNEE_HI - KNEE_LO)      # 0.785 rad => range [-1.57, 0]

# Control loop rate
CTRL_HZ = 50.0
DT = 1.0 / CTRL_HZ

# MIT gains (adjust as needed)
KP = 10.0
KD = 0.2

# Limit how fast we change commanded position (logical), requested:
MAX_VEL_RAD_S = 1.8
MAX_STEP = MAX_VEL_RAD_S * DT


def set_mode_raw(bus: RobstrideBus, motor_name: str, mode: int):
    """Force MODE parameter (int8) to mode=0 for MIT."""
    device_id = bus.motors[motor_name].id
    param_id, _, _ = ParameterType.MODE
    value_buffer = struct.pack("<bBH", int(mode), 0, 0)
    data = struct.pack("<HH", param_id, 0x00) + value_buffer
    bus.transmit(CommunicationType.WRITE_PARAMETER, bus.host_id, device_id, data)
    time.sleep(0.05)


def knee_to_ankle(knee_rad: float) -> float:
    """
    Map knee in [0, -1.57] to ankle in [-30deg, 0] such that:
      knee = 0      -> ankle = -30deg
      knee = -1.57  -> ankle = 0
    """
    knee_rad = clamp(knee_rad, KNEE_LO, KNEE_HI)
    # alpha=0 at knee=0, alpha=1 at knee=-1.57
    alpha = (KNEE_HI - knee_rad) / (KNEE_HI - KNEE_LO)
    ankle = ANKLE_LO * (1.0 - alpha) + ANKLE_HI * alpha  # ANKLE_HI=0, but keep explicit
    return clamp(ankle, ANKLE_LO, ANKLE_HI)


def ramp_toward(cmd: float, target: float, max_step: float, lo: float, hi: float) -> float:
    delta = target - cmd
    if abs(delta) <= max_step:
        cmd = target
    else:
        cmd = cmd + math.copysign(max_step, delta)
    return clamp(cmd, lo, hi)


def main():
    motors = {f"motor_{mid}": Motor(id=mid, model=MOTOR_MODEL_BY_ID[mid]) for mid in MOTORS}
    calibration = {f"motor_{mid}": {"direction": 1, "homing_offset": 0.0} for mid in MOTORS}

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
        for mid in MOTORS:
            name = f"motor_{mid}"
            print(f"Enabling motor {mid} (model={MOTOR_MODEL_BY_ID[mid]}) ...")
            bus.enable(name)
            time.sleep(0.2)
            print(f"Setting motor {mid} to MIT mode (MODE=0) ...")
            set_mode_raw(bus, name, 0)

        # Prime reads + start from current pose (no jump)
        state = {}
        for mid in MOTORS:
            name = f"motor_{mid}"
            pos, vel, tq, temp = bus.read_operation_frame(name)
            logical_pos = pos / float(DIR_BY_ID[mid])  # physical -> logical
            state[mid] = {
                "cmd_logical": logical_pos,  # we ramp from current to target
            }
            # Hold at current pose
            bus.write_operation_frame(name, pos, KP, KD, 0.0, 0.0)
            time.sleep(0.03)

        print("\nRunning coupled sine sweep:")
        print(f"  Knee range : [{KNEE_LO:.3f}, {KNEE_HI:.3f}] rad  ([-90deg, 0deg]) for motors 4/9")
        print(f"  Ankle range: [{ANKLE_LO:.3f}, {ANKLE_HI:.3f}] rad  ([-30deg, 0deg]) for motors 5/10")
        print(f"  Freq       : {FREQ_HZ:.3f} Hz  (period {1.0/FREQ_HZ:.2f} s)")
        print(f"  Max vel cap : {MAX_VEL_RAD_S:.2f} rad/s")
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

            # Knees opposed
            target4 = clamp(KNEE_CENTER + KNEE_AMP * s, KNEE_LO, KNEE_HI)
            target9 = clamp(KNEE_CENTER - KNEE_AMP * s, KNEE_LO, KNEE_HI)

            # Ankles coupled to their knee
            target5 = knee_to_ankle(target4)   # motor5 follows motor4
            target10 = knee_to_ankle(target9)  # motor10 follows motor9

            # Ramp each motor in its own logical limits
            state[M4]["cmd_logical"] = ramp_toward(state[M4]["cmd_logical"], target4, MAX_STEP, KNEE_LO, KNEE_HI)
            state[M9]["cmd_logical"] = ramp_toward(state[M9]["cmd_logical"], target9, MAX_STEP, KNEE_LO, KNEE_HI)
            state[M5]["cmd_logical"] = ramp_toward(state[M5]["cmd_logical"], target5, MAX_STEP, ANKLE_LO, ANKLE_HI)
            state[M10]["cmd_logical"] = ramp_toward(state[M10]["cmd_logical"], target10, MAX_STEP, ANKLE_LO, ANKLE_HI)

            # Send (logical -> physical using direction)
            cmd4_phys = state[M4]["cmd_logical"] * float(DIR_BY_ID[M4])
            cmd9_phys = state[M9]["cmd_logical"] * float(DIR_BY_ID[M9])
            cmd5_phys = state[M5]["cmd_logical"] * float(DIR_BY_ID[M5])
            cmd10_phys = state[M10]["cmd_logical"] * float(DIR_BY_ID[M10])

            bus.write_operation_frame(f"motor_{M4}", cmd4_phys, KP, KD, 0.0, 0.0)
            bus.write_operation_frame(f"motor_{M5}", cmd5_phys, KP, KD, 0.0, 0.0)
            bus.write_operation_frame(f"motor_{M9}", cmd9_phys, KP, KD, 0.0, 0.0)
            bus.write_operation_frame(f"motor_{M10}", cmd10_phys, KP, KD, 0.0, 0.0)

    finally:
        print("\nStopping...")
        try:
            if bus is not None:
                # Hold where they are briefly, then disable
                try:
                    for mid in MOTORS:
                        name = f"motor_{mid}"
                        pos, vel, tq, temp = bus.read_operation_frame(name)
                        # bus.write_operation_frame(name, pos, KP, KD, 0.0, 0.0)
                except Exception:
                    pass
                time.sleep(0.2)

                for mid in MOTORS:
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
