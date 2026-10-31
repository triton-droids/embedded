import math
import numpy as np


def stamp_ns(stamp):
    return int(stamp.sec) * 10**9 + int(stamp.nanosec)


def ordered(names, values, expected):
    if len(names) != len(set(names)) or len(values) != len(names):
        raise ValueError('Duplicate names or incomplete joint array')
    mapping = dict(zip(names, values))
    if not set(expected).issubset(mapping):
        raise ValueError('Missing required joints')
    result = np.array([mapping[name] for name in expected], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('Non-finite joint array')
    return result


def fresh(stamp, received, now_ns, now_mono, timeout):
    age = (now_ns - stamp) * 1e-9
    elapsed = now_mono - received
    return stamp > 0 and -.01 <= age <= timeout and 0 <= elapsed <= timeout


class ContactSelector:
    def __init__(self, lift=.02, land=.008, delay=.04):
        if not all(math.isfinite(x) for x in (lift, land, delay)) or not 0 <= land < lift or delay < 0:
            raise ValueError('Invalid contact hysteresis')
        self.lift, self.land, self.delay = lift, land, delay
        self.active = set()
        self.pending = {}

    def update(self, measured_heights, forces, target_heights, now):
        for i, name in enumerate(('left_foot', 'right_foot')):
            gap = target_heights[i] - measured_heights[i]
            if name in self.active:
                desired = not (gap > self.lift or (measured_heights[i] > .015 and forces[i] < 1.0))
            else:
                desired = gap < self.land and (forces[i] > 2.0 or measured_heights[i] < .005)
            if desired == (name in self.active):
                self.pending.pop(name, None)
                continue
            previous, since = self.pending.get(name, (desired, now))
            if previous != desired:
                since = now
            self.pending[name] = (desired, since)
            if now - since >= self.delay:
                if desired:
                    self.active.add(name)
                else:
                    self.active.discard(name)
                self.pending.pop(name, None)
        return set(self.active)
