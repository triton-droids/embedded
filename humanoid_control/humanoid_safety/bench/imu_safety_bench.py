"""Bench test for the humanoid_safety IMU checks on a real ESP32 + BNO085.

Runs the exact checks from checks.py (fall detection with gyro glitch filter,
stuck sensor, IMU timeout) on live data from bno085_stream.cpp, and prints
which check tripped the e-stop and why.

Setup:
    pip install pyserial
    Keep this folder at humanoid_control/humanoid_safety/bench/ so it uses the
    package's checks.py directly (or put a copy of checks.py next to this file).
Run:
    python imu_safety_bench.py --port /dev/cu.usbmodem1101   # Mac
    python imu_safety_bench.py --port COM5                    # Windows

Type a command and press Enter while it runs:
    (just Enter)  try to reset the e-stop latch
    g             inject a glitch: fake 60 deg roll with a still gyro (should be ignored)
    f             inject a fake fall: 60 deg roll with a spinning gyro (should trip)
    s             freeze: repeat the last reading over and over (should trip imu_stuck)
    s again       unfreeze
    q             quit
Real tests: tilt the board past 40 deg (fall), unplug the USB cable (timeout).
"""
import argparse
import math
import os
import queue
import sys
import threading
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
# Finds checks.py next to this file, one folder up, or (when this folder sits at
# humanoid_control/humanoid_safety/bench/) in the package itself.
sys.path[:0] = [HERE, os.path.dirname(HERE),
                os.path.join(os.path.dirname(HERE), "humanoid_safety")]
from checks import (  # noqa: E402
    FallDetectionCheck,
    ImuFreshnessCheck,
    ImuObservation,
    ImuStuckCheck,
    SafetySupervisor,
    quaternion_to_roll_pitch,
)

# Plain-English meaning of each check, printed when it trips.
EXPLAIN = {
    "fall_detection": "the robot tilted past the fall limit for several readings in a row "
                      "(or the IMU sent an invalid orientation)",
    "imu_stuck": "the IMU kept sending the exact same numbers, so the sensor is probably frozen",
    "imu_freshness": "no IMU data arrived in time (cable unplugged, ESP32 crashed, or reset)",
}

INJECT_SAMPLES = 5
FAKE_ROLL_DEG = 60.0


def parse(line):
    """Return (qw, qx, qy, qz, gyro, accel) or None for status lines."""
    parts = line.strip().split(",")
    if len(parts) != 11:
        return None
    try:
        v = [float(p) for p in parts[1:]]
    except ValueError:
        return None
    return v[0], v[1], v[2], v[3], tuple(v[4:7]), tuple(v[7:10])


def read_commands(cmds):
    for line in sys.stdin:
        cmds.put(line.strip().lower())


def open_port(port, baud):
    try:
        return serial.Serial(port, baud, timeout=0.02)
    except serial.SerialException:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--fall-deg", type=float, default=40.0)
    ap.add_argument("--reset-deg", type=float, default=30.0)
    ap.add_argument("--confirm-samples", type=int, default=3)
    ap.add_argument("--glitch-jump-deg", type=float, default=20.0)
    ap.add_argument("--glitch-max-gyro", type=float, default=0.5, help="rad/s")
    ap.add_argument("--glitch-max-samples", type=int, default=10)
    ap.add_argument("--stuck-samples", type=int, default=50)
    ap.add_argument("--timeout", type=float, default=0.25, help="s")
    args = ap.parse_args()

    start = time.monotonic()
    fall = FallDetectionCheck(
        roll_limit_deg=args.fall_deg, pitch_limit_deg=args.fall_deg,
        reset_roll_deg=args.reset_deg, reset_pitch_deg=args.reset_deg,
        confirm_samples=args.confirm_samples,
        glitch_jump_deg=args.glitch_jump_deg,
        glitch_max_gyro_rad_s=args.glitch_max_gyro,
        glitch_max_samples=args.glitch_max_samples,
    )
    sup = SafetySupervisor([
        fall,
        ImuStuckCheck(stuck_samples=args.stuck_samples),
        ImuFreshnessCheck(timeout_s=args.timeout, startup_grace_s=3.0, started_at=start),
    ])

    cmds = queue.Queue()
    threading.Thread(target=read_commands, args=(cmds,), daemon=True).start()

    ser = open_port(args.port, args.baud)
    if ser is None:
        sys.exit(f"Could not open {args.port}. Close the Serial Monitor and check the port name.")
    print("Commands: Enter=reset  g=glitch  f=fake fall  s=freeze on/off  q=quit")

    buf = b""
    roll = pitch = 0.0
    gyro_rate = 0.0
    inject = None  # ("glitch" | "fall", samples_left)
    frozen = None  # parsed sample repeated while stuck mode is on
    last_sample = None
    last_glitches = 0
    last_print = 0.0

    while True:
        now = time.monotonic()

        # Serial input; reconnect quietly after the cable is plugged back in.
        if ser is None:
            ser = open_port(args.port, args.baud)
            if ser is not None:
                print("\n[info] serial port reconnected")
        else:
            try:
                buf += ser.read(ser.in_waiting or 1)
            except (serial.SerialException, OSError):
                print("\n[info] serial port lost (cable unplugged?)")
                ser.close()
                ser = None
                buf = b""

        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="ignore")
            sample = parse(line)
            if sample is None:
                if line.startswith("#"):
                    print(f"\n[esp32] {line[1:].strip()}")
                continue

            last_sample = sample
            if frozen is not None:
                sample = frozen
            qw, qx, qy, qz, gyro, accel = sample
            r, p, valid = quaternion_to_roll_pitch(qx, qy, qz, qw)

            if inject is not None:
                kind, left = inject
                r = math.radians(FAKE_ROLL_DEG)
                if kind == "fall":
                    gyro = (3.0, 0.0, 0.0)  # Fast roll, like a real fall.
                inject = (kind, left - 1) if left > 1 else None

            sup.observe_imu(ImuObservation(
                received_at=now, roll_rad=r, pitch_rad=p, valid=valid,
                angular_velocity=gyro, linear_acceleration=accel,
            ))
            roll, pitch = math.degrees(r), math.degrees(p)
            gyro_rate = math.sqrt(sum(g * g for g in gyro))

            # Report glitch filtering as it happens (the check keeps a counter).
            glitches = fall._consecutive_glitches
            if glitches > last_glitches:
                print(f"\n[glitch ignored] roll={roll:+.1f} pitch={pitch:+.1f} jumped "
                      f">= {args.glitch_jump_deg:.0f} deg but gyro was only "
                      f"{gyro_rate:.2f} rad/s, so this reading was not counted as a fall "
                      f"({glitches}/{args.glitch_max_samples})")
            last_glitches = glitches

        if sup.evaluate(now):
            print("\n" + "=" * 70)
            print("*** SAFETY E-STOP TRIPPED ***")
            for result in sup.results:
                if result.active:
                    print(f"  {result.name}: {result.reason}")
                    print(f"    why: {EXPLAIN.get(result.name, '')}")
            print("Fix the cause, then press Enter to reset.")
            print("=" * 70)

        while not cmds.empty():
            cmd = cmds.get()
            if cmd == "":
                ok, msg = sup.try_reset(now)
                print(f"\n[reset] {msg}")
            elif cmd == "g":
                inject = ("glitch", INJECT_SAMPLES)
                print(f"\n[inject] glitch: {INJECT_SAMPLES} readings of {FAKE_ROLL_DEG:.0f} deg roll, real gyro")
            elif cmd == "f":
                inject = ("fall", INJECT_SAMPLES)
                print(f"\n[inject] fake fall: {INJECT_SAMPLES} readings of {FAKE_ROLL_DEG:.0f} deg roll, gyro 3 rad/s")
            elif cmd == "s":
                if frozen is None:
                    if last_sample is None:
                        print("\n[inject] no reading yet to freeze")
                    else:
                        frozen = last_sample
                        print("\n[inject] freeze ON: repeating the last reading")
                else:
                    frozen = None
                    print("\n[inject] freeze OFF")
            elif cmd == "q":
                return
            else:
                print(f"\n[?] unknown command {cmd!r}")

        if now - last_print > 0.2:
            state = "TRIPPED" if sup.latched else "ok"
            extra = " FROZEN" if frozen is not None else ""
            print(f"roll={roll:+6.1f}  pitch={pitch:+6.1f}  gyro={gyro_rate:5.2f} rad/s  "
                  f"[{state}{extra}]   ", end="\r", flush=True)
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()

