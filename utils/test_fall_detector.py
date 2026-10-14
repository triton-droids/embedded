"""Runs the IMUFallDetector class from gain_tuner.py on live ESP32 data.

No motors, no CAN, no ROS. Put this file in utils/ (next to gain_tuner.py
and imu_read.py) and run:
    python test_fall_detector.py --port /dev/cu.usbmodem1101

It copies the IMUFallDetector class straight out of gain_tuner.py, so you are
testing your real code. (gain_tuner.py can't be imported as a whole right now
because of the indentation errors and motor dependencies.)

Tilt the board past 40 deg -> you should see FALL DETECTED and HALT CALLED.
Press Enter to re-arm the detector and try again. Ctrl+C to quit.
"""
import argparse
import math
import re
import threading
import time
from pathlib import Path
from typing import Callable, Optional

from imu_read import iter_imu_samples, RK4DeadReckoner


def load_detector_class():
    src = (Path(__file__).parent / "gain_tuner.py").read_text()
    m = re.search(r"^class IMUFallDetector\b.*?(?=^class |\Z)", src, re.S | re.M)
    if not m:
        raise SystemExit("Couldn't find class IMUFallDetector in gain_tuner.py")
    ns = {"threading": threading, "math": math, "time": time,
          "Callable": Callable, "Optional": Optional,
          "iter_imu_samples": iter_imu_samples, "RK4DeadReckoner": RK4DeadReckoner}
    exec(compile(m.group(0), "gain_tuner.py:IMUFallDetector", "exec"), ns)
    return ns["IMUFallDetector"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--limit", type=float, default=40.0, help="trip angle, deg")
    ap.add_argument("--integrator", action="store_true",
                    help="use RK4 instead of firmware roll/pitch (currently broken, see README)")
    args = ap.parse_args()

    IMUFallDetector = load_detector_class()
    halts = []

    def fake_halt(reason):
        halts.append(reason)
        print(f"\n>>> HALT CALLED (motors would stop here): {reason}")

    det = IMUFallDetector(port=args.port, halt_fn=fake_halt, baud=args.baud,
                          fall_roll_deg=args.limit, fall_pitch_deg=args.limit,
                          use_integrator=args.integrator)
    det.start()

    # Enter re-arms the detector (it latches after one trip)
    def rearm():
        while True:
            input()
            det._tripped, det._trip_count = False, 0
            print("\nre-armed")
    threading.Thread(target=rearm, daemon=True).start()

    try:
        while True:
            r, p, _ = det.get_rpy()
            print(f"roll={math.degrees(r):+6.1f}  pitch={math.degrees(p):+6.1f}  "
                  f"samples={det.samples_read}  errors={det.errors}  "
                  f"tripped={det.tripped}  halts={len(halts)}   ", end="\r", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        det.stop()
        print()


if __name__ == "__main__":
    main()

