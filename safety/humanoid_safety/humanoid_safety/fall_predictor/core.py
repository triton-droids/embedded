"""ROS-free core of the advisory fall predictor.

Mirrors evaluation/fall_predictor/shadow_node.py, which was
validated online against the offline training features: same features, same
20-sample window, same per-head threshold and consecutive-frame confirmation.
The simulation-only gantry gate is dropped; the estop gate and the header-stamp
gap gate are kept.
"""
import collections
import json
import math
import os
from dataclasses import dataclass


@dataclass(frozen=True)
class Head:
    name: str
    horizon_s: float
    threshold: float
    confirm_frames: int


@dataclass(frozen=True)
class ModelSpec:
    name: str
    model_path: str
    model_sha256: str
    input_name: str
    history: int
    features: tuple
    min_gap_s: float
    max_gap_s: float
    reset_gap_s: float
    heads: tuple

    def head(self, name):
        for head in self.heads:
            if head.name == name:
                return head
        raise KeyError(f'unknown head {name!r}; available: {[h.name for h in self.heads]}')


def load_spec(path):
    """Read a model spec JSON; model_file is resolved next to the spec."""
    with open(path) as f:
        raw = json.load(f)
    heads = tuple(Head(str(h['name']), float(h['horizon_s']), float(h['threshold']),
                       int(h['confirm_frames'])) for h in raw['heads'])
    if not heads or any(h.confirm_frames < 1 or not 0 < h.threshold < 1 for h in heads):
        raise ValueError(f'{path}: every head needs 0 < threshold < 1 and confirm_frames >= 1')
    return ModelSpec(
        name=str(raw['name']),
        model_path=os.path.join(os.path.dirname(os.path.abspath(path)), raw['model_file']),
        model_sha256=str(raw.get('model_sha256', '')),
        input_name=str(raw['input_name']),
        history=int(raw['history']),
        features=tuple(raw['features']),
        min_gap_s=float(raw['min_gap_s']),
        max_gap_s=float(raw['max_gap_s']),
        reset_gap_s=float(raw['reset_gap_s']),
        heads=heads,
    )


def imu_features(orientation_wxyz, angular_velocity, linear_acceleration):
    """[roll, pitch, gyro xyz, accel xyz], exactly as the training collector."""
    qw, qx, qy, qz = orientation_wxyz
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(min(1.0, max(-1.0, 2 * (qw * qy - qz * qx))))
    return [roll, pitch, *map(float, angular_velocity), *map(float, linear_acceleration)]


class OnnxScorer:
    """Batch-of-one ONNX inference; returns one sigmoid score per head."""

    def __init__(self, spec):
        import numpy as np
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._np = np
        self._session = ort.InferenceSession(spec.model_path, options,
                                             providers=['CPUExecutionProvider'])
        self._input = spec.input_name
        self._heads = len(spec.heads)

    def __call__(self, window):
        np = self._np
        x = np.asarray([window], dtype=np.float32)
        logits = self._session.run(None, {self._input: x})[0][0]
        if len(logits) != self._heads:
            raise ValueError(f'model returned {len(logits)} outputs for {self._heads} heads')
        return [float(1.0 / (1.0 + math.exp(-float(v)))) for v in logits]


@dataclass
class Result:
    stamp_ns: int
    scores: dict
    alarms: dict
    onsets: list
    eligible: bool


class FallPredictor:
    """Windowing, eligibility and per-head confirmation around a scorer."""

    def __init__(self, spec, scorer):
        self.spec = spec
        self.scorer = scorer
        self.window = collections.deque(maxlen=spec.history)  # (stamp_ns, features, estop)
        self.run = {h.name: 0 for h in spec.heads}
        self.alarm = {h.name: False for h in spec.heads}
        self.resets = 0  # histories discarded for a stamp jump

    def reset(self):
        self.window.clear()
        for head in self.spec.heads:
            self.run[head.name] = 0
            self.alarm[head.name] = False

    def update(self, stamp_ns, features, estop):
        """Add one IMU sample. Returns a Result once the window is full, else None.

        A non-increasing stamp or a gap of reset_gap_s or more starts a fresh
        history (restart, replayed bag, long dropout). Windows containing an
        estop sample or an out-of-range stamp gap are scored but never alarm.
        """
        if self.window:
            gap_ns = stamp_ns - self.window[-1][0]
            if not 0 < gap_ns < self.spec.reset_gap_s * 1e9:
                self.reset()
                self.resets += 1
        self.window.append((int(stamp_ns), list(features), bool(estop)))
        if len(self.window) < self.spec.history:
            return None
        stamps = [w[0] for w in self.window]
        gaps_ok = all(self.spec.min_gap_s - 1e-9 <= (b - a) / 1e9 <= self.spec.max_gap_s + 1e-9
                      for a, b in zip(stamps, stamps[1:]))
        eligible = gaps_ok and not any(w[2] for w in self.window)
        scores = self.scorer([w[1] for w in self.window])
        onsets = []
        out_scores = {}
        for head, score in zip(self.spec.heads, scores):
            out_scores[head.name] = score
            self.run[head.name] = self.run[head.name] + 1 if (eligible and score >= head.threshold) else 0
            active = self.run[head.name] >= head.confirm_frames
            if active and not self.alarm[head.name]:
                onsets.append(head.name)
            self.alarm[head.name] = active
        return Result(stamps[-1], out_scores, dict(self.alarm), onsets, eligible)
