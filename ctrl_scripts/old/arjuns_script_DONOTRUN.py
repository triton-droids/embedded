#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sinusoidal leg swing for RobStride motors:
- Hip swing:   motors 1 / 6  (rs-04)  [NEW]
- Knee sweep:  motors 4 / 9  (rs-04)
- Ankle sweep: motors 5 / 10 (rs-02) coupled to knee

Goal (existing):
- Knee motors 4 and 9 move sinusoidally between:
    0 rad  <->  -1.57 rad  (~ -90 deg)
- They are opposite ends (phase-shifted by pi):
    when motor4 = 0, motor9 = -1.57, and vice versa
- Ankle motors 5 and 10 are *coupled* to their respective knee:
    If knee (4 or 9) = 0 deg   -> ankle (5 or 10) = -30 deg
    If knee (4 or 9) = -90 deg -> ankle (5 or 10) = 0 deg
  i.e., ankle range is [-30, 0] deg.

NEW:
- Hip swing motors 1 and 6 also move sinusoidally, opposite phase (pi apart).
  Default logical hip range here is conservative; adjust HIP_LO/HIP_HI to your robot.

Config requested:
- FREQ_HZ = 0.3
- MAX_VEL_RAD_S = 1.8

Per-motor MIT gains (as requested):
- motor 1 & 6:  kp=300 kd=20
- motor 4 & 9:  kp=270 kd=20
- motor 5 & 10: kp=40  kd=2

Run:
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set up can0
  python3 sine_leg_swing_1_6_4_9_5_10.py
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
M1, M2, M3, M4, M5, M6, M7, M8, M9, M10 = 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
MOTORS = (M1, M6, M2, M3, M7, M8, M4, M5, M9, M10)

# Models
MOTOR_MODEL_BY_ID = {
    M1: "rs-04",
    M6: "rs-04",
    M2: "rs-03",   # NEW: hold-zero motor
    M3: "rs-03",
    M7: "rs-03",   # NEW: hold-zero motor
    M8: "rs-03",
    M4: "rs-04",
    M5: "rs-02",
    M9: "rs-04",
    M10: "rs-02",
}

# Direction (from your inversion array: IDs 1..5 are +1, 6..10 are -1)
DIR_BY_ID = {
    M1: -1,
    M4: +1,
    M5: +1,
    M6: 1,
    M9: -1,
    M10: -1,
    M2: +1,   # NEW
    M3: +1,
    M7: -1,   # NEW
    M8: -1,
}

# Per-motor MIT gains (kp/kd) as requested
GAINS_BY_ID = {
    M1: (300.0, 20.0),
    M6: (300.0, 20.0),
    M4: (270.0, 20.0),
    M9: (270.0, 20.0),
    M5: (40.0, 2.0),
    M10: (40.0, 2.0),
    M2: (120.0, 8.0),   # NEW: hold-zero motor
    M3: (120.0, 8.0),
    M7: (120.0, 8.0),   # NEW: hold-zero motor
    M8: (120.0, 8.0),
}

# -------------------- LOGICAL LIMITS --------------------
# Hip swing logical limits (radians) [NEW]
# NOTE: adjust these to your mechanical joint range for motors 1/6.

# Hip swing logical limits (radians) for motors 1/6: [-50deg, +50deg]
HIP_LO = -math.radians(30.0)
HIP_HI =  +math.radians(45.0)

HIP_CENTER = 0.5 * (HIP_LO + HIP_HI)   # 0.0
HIP_AMP    = 0.5 * (HIP_HI - HIP_LO)   # 50deg in rad


# Knee logical limits (radians) for this test
KNEE_LO = -1.0
KNEE_HI = 0.0

# Ankle logical limits (radians): [-30deg, 0deg]
ANKLE_LO = -math.radians(30.0)   # -0.523599...
ANKLE_HI = 0.0

# Hold-zero logical limits (radians) for motors 3/8
HOLD_LO = -math.pi
HOLD_HI = math.pi

# -------------------- SINE PARAMETERS --------------------
FREQ_HZ = 0.5

KNEE_CENTER = 0.5 * (KNEE_LO + KNEE_HI)   # -0.785 rad
KNEE_AMP = 0.5 * (KNEE_HI - KNEE_LO)      # 0.785 rad => range [-1.57, 0]

# Control loop rate
CTRL_HZ = 60.0
DT = 1.0 / CTRL_HZ

# Limit how fast we change commanded position (logical)
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
    ankle = ANKLE_LO * (1.0 - alpha) + ANKLE_HI * alpha
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
            state[mid] = {"cmd_logical": logical_pos}
            kp, kd = GAINS_BY_ID[mid]
            # Hold at current pose (no jump)
            bus.write_operation_frame(name, pos, kp, kd, 0.0, 0.0)
            time.sleep(0.03)

        print("\nRunning coupled sine sweep:")
        print(f"  Hip  range : [{HIP_LO:.3f}, {HIP_HI:.3f}] rad  for motors 1/6 (opposed)")
        print(f"  Knee range : [{KNEE_LO:.3f}, {KNEE_HI:.3f}] rad  ([-90deg, 0deg]) for motors 4/9 (opposed)")
        print(f"  Ankle range: [{ANKLE_LO:.3f}, {ANKLE_HI:.3f}] rad  ([-30deg, 0deg]) for motors 5/10 (coupled)")
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

            # --- Hips (NEW): opposed ---
            target1 = clamp(HIP_CENTER + HIP_AMP * s, HIP_LO, HIP_HI)
            target6 = clamp(HIP_CENTER - HIP_AMP * s, HIP_LO, HIP_HI)

            # --- Hold-zero motors (NEW): fixed at 0 ---
            target2 = 0.0
            target3 = 0.0
            target7 = 0.0
            target8 = 0.0

            # --- Knees: opposed ---
            target4 = clamp(KNEE_CENTER + KNEE_AMP * s, KNEE_LO, KNEE_HI)
            target9 = clamp(KNEE_CENTER - KNEE_AMP * s, KNEE_LO, KNEE_HI)

            # --- Ankles: coupled to their knee ---
            target5 = knee_to_ankle(target4)    # motor5 follows motor4
            target10 = knee_to_ankle(target9)   # motor10 follows motor9

            # Ramp each motor in its own logical limits
            state[M1]["cmd_logical"] = ramp_toward(state[M1]["cmd_logical"], target1, MAX_STEP, HIP_LO, HIP_HI)
            state[M6]["cmd_logical"] = ramp_toward(state[M6]["cmd_logical"], target6, MAX_STEP, HIP_LO, HIP_HI)

            state[M2]["cmd_logical"] = ramp_toward(state[M2]["cmd_logical"], target2, MAX_STEP, HOLD_LO, HOLD_HI)
            state[M3]["cmd_logical"] = ramp_toward(state[M3]["cmd_logical"], target3, MAX_STEP, HOLD_LO, HOLD_HI)
            state[M7]["cmd_logical"] = ramp_toward(state[M7]["cmd_logical"], target7, MAX_STEP, HOLD_LO, HOLD_HI)
            state[M8]["cmd_logical"] = ramp_toward(state[M8]["cmd_logical"], target8, MAX_STEP, HOLD_LO, HOLD_HI)

            state[M4]["cmd_logical"] = ramp_toward(state[M4]["cmd_logical"], target4, MAX_STEP, KNEE_LO, KNEE_HI)
            state[M9]["cmd_logical"] = ramp_toward(state[M9]["cmd_logical"], target9, MAX_STEP, KNEE_LO, KNEE_HI)

            state[M5]["cmd_logical"] = ramp_toward(state[M5]["cmd_logical"], target5, MAX_STEP, ANKLE_LO, ANKLE_HI)
            state[M10]["cmd_logical"] = ramp_toward(state[M10]["cmd_logical"], target10, MAX_STEP, ANKLE_LO, ANKLE_HI)

            # Send (logical -> physical using direction)
            cmd1_phys = state[M1]["cmd_logical"] * float(DIR_BY_ID[M1])
            cmd6_phys = state[M6]["cmd_logical"] * float(DIR_BY_ID[M6])

            cmd2_phys = state[M2]["cmd_logical"] * float(DIR_BY_ID[M2])
            cmd3_phys = state[M3]["cmd_logical"] * float(DIR_BY_ID[M3])
            cmd7_phys = state[M7]["cmd_logical"] * float(DIR_BY_ID[M7])
            cmd8_phys = state[M8]["cmd_logical"] * float(DIR_BY_ID[M8])

            cmd4_phys = state[M4]["cmd_logical"] * float(DIR_BY_ID[M4])
            cmd9_phys = state[M9]["cmd_logical"] * float(DIR_BY_ID[M9])

            cmd5_phys = state[M5]["cmd_logical"] * float(DIR_BY_ID[M5])
            cmd10_phys = state[M10]["cmd_logical"] * float(DIR_BY_ID[M10])

            # Write frames with per-motor gains
            kp, kd = GAINS_BY_ID[M1]
            bus.write_operation_frame(f"motor_{M1}", cmd1_phys, kp, kd, 0.0, 0.0)

            kp, kd = GAINS_BY_ID[M6]
            bus.write_operation_frame(f"motor_{M6}", cmd6_phys, kp, kd, 0.0, 0.0)

            kp, kd = GAINS_BY_ID[M2]
            bus.write_operation_frame(f"motor_{M2}", cmd2_phys, kp, kd, 0.0, 0.0)

            kp, kd = GAINS_BY_ID[M3]
            bus.write_operation_frame(f"motor_{M3}", cmd3_phys, kp, kd, 0.0, 0.0)

            kp, kd = GAINS_BY_ID[M7]
            bus.write_operation_frame(f"motor_{M7}", cmd7_phys, kp, kd, 0.0, 0.0)

            kp, kd = GAINS_BY_ID[M8]
            bus.write_operation_frame(f"motor_{M8}", cmd8_phys, kp, kd, 0.0, 0.0)

            kp, kd = GAINS_BY_ID[M4]
            bus.write_operation_frame(f"motor_{M4}", cmd4_phys, kp, kd, 0.0, 0.0)

            kp, kd = GAINS_BY_ID[M5]
            bus.write_operation_frame(f"motor_{M5}", cmd5_phys, kp, kd, 0.0, 0.0)

            kp, kd = GAINS_BY_ID[M9]
            bus.write_operation_frame(f"motor_{M9}", cmd9_phys, kp, kd, 0.0, 0.0)

            kp, kd = GAINS_BY_ID[M10]
            bus.write_operation_frame(f"motor_{M10}", cmd10_phys, kp, kd, 0.0, 0.0)

    finally:
        print("\nStopping...")
        try:
            if bus is not None:
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
