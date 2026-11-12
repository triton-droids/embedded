"""Record a VIO trial for video: sim kinematic state at 15 Hz from the stereo state segment.

  python3 record_route.py DURATION_S OUT.npz
No ROS: it only maps shared memory, so it adds almost no load. Ground truth and
the ball come from qpos; the estimate comes later from localization_eval's CSV.
The model is copied next to OUT after recording.
"""
import shutil
import sys
import time

import numpy as np

from humanoid_sim.state_shm import StateReader


def main():
    duration, out = float(sys.argv[1]), sys.argv[2]
    reader = StateReader(timeout_s=120)
    rows = []
    last = None
    end = time.monotonic() + duration
    next_t = time.monotonic()
    while time.monotonic() < end:
        rec = reader.read()
        if rec is not None and rec[0] != last:
            last = rec[0]
            rows.append((rec[0], rec[1].copy(), rec[2].copy(), rec[3].copy()))
        next_t += 1 / 15
        time.sleep(max(0.0, next_t - time.monotonic()))
    np.savez_compressed(out, stamp=np.array([r[0] for r in rows]), qpos=np.array([r[1] for r in rows]),
                        mocap_pos=np.array([r[2] for r in rows]), mocap_quat=np.array([r[3] for r in rows]))
    shutil.copy(reader.model_path, out.replace('.npz', '.mjb'))
    print(f'saved {len(rows)} frames to {out}')


if __name__ == '__main__':
    main()
