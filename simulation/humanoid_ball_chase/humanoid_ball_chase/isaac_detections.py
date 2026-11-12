"""Use the Isaac ROS YOLO pipeline (scripts/run_isaac_yolo.sh) as ball_chase_vision's detector.

The pipeline publishes vision_msgs/Detection2DArray with the color image's stamp
and boxes in 640x640 network pixels (aspect-preserving resize, centered padding).
This source keeps the recent arrays by stamp; for each synchronized RGB-D frame
it waits briefly for the array with the same stamp, keeps the detections of
``class_id``, maps them back to image pixels and reads their depth like
FootballDetector. Which object the robot chases is therefore a parameter
(td-perception: 0 ball, 1 goalpost, 2 robot, 3 L-, 4 T-, 5 X-intersection,
6 penalty mark).

A class can have several instances (robots, line intersections). Targets do not
jump between frames, so the source locks on: it keeps the 3D point (camera
optical axes) nearest to the previous target. With no target for
``reacquire_s`` it picks the instance nearest to the camera again.
"""
from __future__ import annotations

import collections
import threading
import time

import numpy as np

from .football_detector import point_from_box

NETWORK_SIZE = 640


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def network_to_image(cx, cy, bw, bh, width, height, network=NETWORK_SIZE):
    """Box center and size from network pixels to image pixels (centered padding)."""
    scale = min(network / width, network / height)
    pad_x, pad_y = (network - width * scale) / 2.0, (network - height * scale) / 2.0
    return (cx - pad_x) / scale, (cy - pad_y) / scale, bw / scale, bh / scale


class IsaacDetectionSource:
    """Detector interface for BallChaseVision fed by /isaac_yolo/detections."""

    device = 'isaac'
    engine_path = None
    needs_stamp = True

    def __init__(self, node=None, topic='/isaac_yolo/detections', class_id=0, wait_s=0.3,
                 min_score=0.0, history=30, reacquire_s=1.0, clock=time.monotonic):
        self.class_id = str(int(class_id))
        self.wait_s = float(wait_s)
        self.min_score = float(min_score)
        self.last_confidence = 0.0
        self.reacquire_s = float(reacquire_s)
        self._clock = clock
        self._target = None      # Last target point (x, y, z), camera optical axes.
        self._target_time = None
        self._arrays = collections.OrderedDict()
        self._history = int(history)
        self._cond = threading.Condition()
        if node is not None:
            from vision_msgs.msg import Detection2DArray
            node.create_subscription(Detection2DArray, topic, self.on_detections, 10)

    def on_detections(self, msg) -> None:
        with self._cond:
            self._arrays[stamp_ns(msg.header.stamp)] = msg
            while len(self._arrays) > self._history:
                self._arrays.popitem(last=False)
            self._cond.notify_all()

    def _wait_for(self, key):
        with self._cond:
            self._cond.wait_for(lambda: key in self._arrays, timeout=self.wait_s)
            return self._arrays.get(key)

    def detect(self, rgb, depth_m, k, stamp):
        """(u, v, x, y, z) of the best class_id detection in this frame, or None."""
        self.last_confidence = 0.0
        h, w = rgb.shape[:2]
        if depth_m.shape != (h, w):
            return None
        msg = self._wait_for(stamp_ns(stamp))
        if msg is None:
            return None  # The pipeline produced nothing for this frame in time.
        candidates = []
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if hyp.class_id != self.class_id or hyp.score < self.min_score:
                continue
            box = det.bbox
            candidates.append((float(hyp.score), network_to_image(
                box.center.position.x, box.center.position.y, box.size_x, box.size_y, w, h)))
        k = np.asarray(k).reshape(3, 3)
        points = []
        for score, (u, v, bw, bh) in candidates:
            point = point_from_box(u, v, bw, bh, depth_m, k)
            if point is not None:
                points.append((score, point))
        if not points:
            return None
        now = self._clock()
        locked = (self._target is not None and now - self._target_time <= self.reacquire_s)
        reference = self._target if locked else np.zeros(3)  # Else: nearest to the camera.
        score, point = min(points, key=lambda p: float(np.linalg.norm(np.asarray(p[1][2:]) - reference)))
        self._target, self._target_time = np.asarray(point[2:], dtype=float), now
        self.last_confidence = score
        return point
