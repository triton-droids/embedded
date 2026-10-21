#!/usr/bin/env python3
"""Compare self-collision backends on the same random poses.

Reports time per pose for each backend and how far its distances differ
from the first backend listed. Poses are uniform within the URDF limits.
"""

import argparse
import os
import time

import numpy as np

from .checker import BACKENDS, create_backend, urdf_adjacent_link_pairs, urdf_joint_limits


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backends', nargs='+', default=sorted(BACKENDS), choices=sorted(BACKENDS))
    parser.add_argument('--poses', type=int, default=200)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--urdf', default='')
    args = parser.parse_args(argv)

    urdf = args.urdf
    if urdf:
        package_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(urdf))))
    else:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('humanoid_leg_description')
        urdf, package_dir = os.path.join(share, 'urdf', 'human_offset_corrected.urdf'), os.path.dirname(share)
    allowed = urdf_adjacent_link_pairs(urdf)
    backends = [create_backend(name, urdf, [package_dir], allowed) for name in args.backends]
    joint_names = backends[0].joint_names
    if any(b.joint_names != joint_names or b.link_pairs != backends[0].link_pairs for b in backends):
        raise SystemExit('Backends disagree on joints or link pairs; results are not comparable')

    lower, upper = urdf_joint_limits(urdf, joint_names)
    poses = np.random.default_rng(args.seed).uniform(lower, upper, size=(args.poses, len(joint_names)))
    print(f'{args.poses} random poses, {len(backends[0].link_pairs)} link pairs')
    reference = None
    for backend in backends:
        times, results = [], []
        for pose in poses:
            started = time.perf_counter()
            results.append(backend.distances(pose))
            times.append((time.perf_counter() - started) * 1000)
        results = np.asarray(results)
        line = (f'{backend.name:16s} mean {np.mean(times):8.3f} ms  p99 {np.percentile(times, 99):8.3f} ms  '
                f'max {np.max(times):8.3f} ms  collisions {int(np.sum(results <= 0))}')
        if reference is None:
            reference = results
        else:
            # Penetration depth is backend-specific, so compare clear distances
            # and whether each pair is in collision.
            clear = (reference > 0) & (results > 0)
            disagree = int(np.sum((reference <= 0) != (results <= 0)))
            line += (f'  max |diff| {np.max(np.abs(reference - results)[clear]):.2e} m'
                     f'  collision disagreements {disagree}')
        print(line)


if __name__ == '__main__':
    main()
