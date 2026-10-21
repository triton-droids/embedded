#!/usr/bin/env python3
"""Compare self-collision backends on the same random poses.

Reports time per pose for each backend and how far its distances differ
from the first backend listed. Poses are uniform within the URDF limits and
are passed to each backend by joint name. A backend that also provides
min_distance() (one query over all pairs) gets an extra timing row.
"""

import argparse
import os
import time

import numpy as np

from .checker import (
    BACKENDS,
    create_backend,
    parse_link_pairs,
    urdf_adjacent_link_pairs,
    urdf_joint_limits,
)


def timings(function, poses):
    results, times = [], []
    for pose in poses:
        started = time.perf_counter()
        results.append(function(pose))
        times.append((time.perf_counter() - started) * 1000)
    return np.asarray(results), np.asarray(times)


def summary(name, times):
    return (f'{name:24s} mean {np.mean(times):8.3f} ms  p99 {np.percentile(times, 99):8.3f} ms  '
            f'max {np.max(times):8.3f} ms')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backends', nargs='+', default=sorted(BACKENDS), choices=sorted(BACKENDS))
    parser.add_argument('--poses', type=int, default=200)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--urdf', default='')
    parser.add_argument('--extra-allowed-pairs', default='',
                        help='"link_a:link_b, ..." exempt in addition to adjacent links')
    args = parser.parse_args(argv)

    urdf = args.urdf
    if urdf:
        package_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(urdf))))
    else:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('humanoid_leg_description')
        urdf, package_dir = os.path.join(share, 'urdf', 'human_offset_corrected.urdf'), os.path.dirname(share)
    allowed = urdf_adjacent_link_pairs(urdf) | parse_link_pairs(args.extra_allowed_pairs)
    backends = [create_backend(name, urdf, [package_dir], allowed) for name in args.backends]
    reference = backends[0]
    for backend in backends[1:]:
        if set(backend.joint_names) != set(reference.joint_names) or backend.link_pairs != reference.link_pairs:
            raise SystemExit(f'{backend.name} disagrees with {reference.name} on joints or link pairs')

    names = reference.joint_names
    lower, upper = urdf_joint_limits(urdf, names)
    samples = np.random.default_rng(args.seed).uniform(lower, upper, size=(args.poses, len(names)))
    poses_by_name = [dict(zip(names, sample)) for sample in samples]
    print(f'{args.poses} random poses, {len(reference.link_pairs)} link pairs')

    reference_results = None
    for backend in backends:
        poses = [np.asarray([pose[name] for name in backend.joint_names]) for pose in poses_by_name]
        results, times = timings(backend.distances, poses)
        line = summary(backend.name, times) + f'  collisions {int(np.sum(results <= 0))}'
        if reference_results is None:
            reference_results = results
        else:
            # Penetration depth is backend-specific (MoveIt reports 0), so
            # compare clear distances and whether each pair is in collision.
            clear = (reference_results > 0) & (results > 0)
            disagree = int(np.sum((reference_results <= 0) != (results <= 0)))
            line += (f'  max |diff| {np.max(np.abs(reference_results - results)[clear]):.2e} m'
                     f'  collision disagreements {disagree}')
        print(line)
        if hasattr(backend, 'min_distance'):
            minimum, times = timings(backend.min_distance, poses)
            agrees = np.allclose(minimum, results.min(axis=1), atol=1e-6)
            print(summary(f'{backend.name} (one query)', times)
                  + f'  overall minimum only; matches per-pair minimum: {agrees}')


if __name__ == '__main__':
    main()
