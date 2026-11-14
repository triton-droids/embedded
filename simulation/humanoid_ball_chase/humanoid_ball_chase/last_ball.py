"""Last consistent visual ball position in a fixed odometry frame."""
from dataclasses import dataclass, field
import math

@dataclass
class LastBall:
    max_age: float = 12.0
    samples: list = field(default_factory=list)
    point: tuple | None = None
    stamp: float | None = None
    std: float = math.inf
    ready: bool = False
    near: bool = False
    close_seen: bool = False

    def observe(self, stamp, xy, std, score):
        # Only actual image measurements enter here, never tracker predictions.
        if self.samples and stamp <= self.samples[-1][0]:
            return
        # A weak sighting is a miss: retain the last reliable anchor and its
        # original stamp. It must neither move the target nor extend its life.
        if math.isfinite(score) and score < .5:
            return
        if not all(math.isfinite(v) for v in (stamp, *xy, std, score)) or std > .12:
            self.ready = False
            self.samples.clear()
            return
        xy = tuple(float(v) for v in xy)
        if self.samples and stamp - self.samples[-1][0] > .6:
            # A gap only resets candidate confirmation. A consistent new image
            # must not revoke the still-valid last accepted position.
            self.samples.clear()
            if self.point is not None and math.dist(xy, self.point) > .06:
                self.ready = False
        elif self.samples and any(math.dist(xy, s[1]) > .06 for s in self.samples):
            self.samples.clear()
            self.ready = False
        self.samples.append((stamp, xy))
        self.samples = [s for s in self.samples[-100:] if stamp - s[0] <= .5]
        if len(self.samples) >= 3 and stamp - self.samples[0][0] >= .2:
            self.point, self.stamp, self.std = xy, stamp, std
            self.ready = True

    def usable(self, now):
        return self.ready and self.point is not None and 0 <= now - self.stamp <= self.max_age

    def status(self, now):
        return dict(near=self.near, close_seen=self.close_seen,
                    usable=self.usable(now), point_odom=self.point,
                    since_seen_s=None if self.stamp is None else now-self.stamp,
                    source='last_consistent_image', max_age_s=self.max_age)
