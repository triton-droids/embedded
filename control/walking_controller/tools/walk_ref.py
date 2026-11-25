"""Build a NumPy-only phase-binned absolute joint-pose reference from walk_demo episodes.

Only active clocks and commands inside the inclusive forward-speed range contribute.
Every bin needs at least 20 ticks; q_std is the population standard deviation.
"""
import argparse
from pathlib import Path

import numpy as np


def build_reference(data, bins=50, vx=(0.04, 0.12), out=None):
    """Pool compatible episodes, returning (and optionally saving) the reference arrays."""
    if bins < 1 or not np.isfinite(vx).all() or len(vx) != 2 or vx[0] > vx[1]:
        raise ValueError('require bins >= 1 and a finite ordered vx range')
    paths = sorted({p.resolve() for directory in data for p in Path(directory).glob('*.npz')
                    if out is None or p.resolve() != Path(out).resolve()})
    if not paths:
        raise ValueError('no episode npz files found')
    meta, poses, indices = None, [], []
    for path in paths:
        with np.load(path, allow_pickle=False) as ep:
            current = {key: ep[key].copy() for key in ('q_nominal', 'joint_names', 'period', 'stance')}
            if current['q_nominal'].shape != (18,) or current['joint_names'].shape != (18,):
                raise ValueError(f'{path}: expected 18 joints')
            if meta is None:
                meta = current
            elif any(not np.array_equal(meta[key], current[key]) for key in meta):
                raise ValueError(f'{path}: incompatible episode metadata')
            frame, cmd = ep['frame'], ep['command']
            if frame.ndim != 2 or frame.shape[1] != 42 or cmd.shape != (len(frame), 5):
                raise ValueError(f'{path}: expected frame (ticks, 42), command (ticks, 5)')
            keep = (np.abs(cmd[:, 3:5]).sum(axis=1) > 0.5) & (cmd[:, 0] >= vx[0]) & (cmd[:, 0] <= vx[1])
            phase = (np.arctan2(cmd[keep, 3].astype(float), cmd[keep, 4].astype(float)) / (2 * np.pi)) % 1
            q = frame[keep, 6:24].astype(float) + current['q_nominal']
            if not np.isfinite(q).all() or not np.isfinite(phase).all():
                raise ValueError(f'{path}: nonfinite selected data')
            poses.append(q)
            indices.append(np.floor(phase * bins).astype(int) % bins)
    q, idx = np.concatenate(poses), np.concatenate(indices)
    count = np.bincount(idx, minlength=bins)
    if np.any(count < 20):
        raise ValueError(f'each bin needs >= 20 ticks; count={count.tolist()}')
    result = dict(phase=(np.arange(bins) + 0.5) / bins,
                  q_ref=np.stack([q[idx == b].mean(axis=0) for b in range(bins)]),
                  q_std=np.stack([q[idx == b].std(axis=0) for b in range(bins)]),
                  count=count, **meta, vx_range=np.asarray(vx, dtype=float))
    if out is not None:
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(out, **result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, action='append', required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--bins', type=int, default=50)
    parser.add_argument('--vx', type=float, nargs=2, default=(0.04, 0.12))
    args = parser.parse_args()
    try:
        ref = build_reference(args.data, args.bins, args.vx, args.out)
    except (ValueError, KeyError, OSError) as exc:
        parser.error(str(exc))
    print(f"bins={args.bins} total_ticks={ref['count'].sum()}")
    for name, span in zip(ref['joint_names'], np.ptp(ref['q_ref'], axis=0)):
        if 'shoulder' not in name and 'elbow' not in name:
            print(f'{name}: range={span:.6f} rad')


if __name__ == '__main__':
    main()
