"""Compare Isaac YOLO detections with ball_chase_vision's ball pixel, per image stamp.

  python3 compare_yolo.py DURATION_S [IMAGE_W IMAGE_H]
"""
import collections
import json
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

NAMES = ['ball', 'goalpost', 'robot', 'L-int', 'T-int', 'X-int', 'penalty']


def key(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class Compare(Node):
    def __init__(self, w, h):
        super().__init__('compare_yolo')
        self.scale = min(640 / w, 640 / h)
        self.pad = ((640 - w * self.scale) / 2, (640 - h * self.scale) / 2)
        self.isaac, self.python = {}, {}
        self.classes = collections.Counter()
        self.latency, self.py_latency, self.frames = [], [], 0
        self.create_subscription(Detection2DArray, '/isaac_yolo/detections', self._isaac, 10)
        self.create_subscription(PointStamped, '/vision/target_center', self._python, 10)

    def _isaac(self, msg):
        self.frames += 1
        now = self.get_clock().now().nanoseconds
        self.latency.append((now - key(msg.header.stamp)) * 1e-6)
        best = None
        for d in msg.detections:
            if not d.results:
                continue
            cls = int(d.results[0].hypothesis.class_id)
            score = d.results[0].hypothesis.score
            self.classes[NAMES[cls] if cls < len(NAMES) else cls] += 1
            if cls == 0 and (best is None or score > best[2]):
                best = (d.bbox.center.position.x, d.bbox.center.position.y, score)
        if best is not None:
            self.isaac[key(msg.header.stamp)] = best

    def _python(self, msg):
        self.py_latency.append((self.get_clock().now().nanoseconds - key(msg.header.stamp)) * 1e-6)
        self.python[key(msg.header.stamp)] = (msg.point.x, msg.point.y)

    def summary(self):
        common = sorted(set(self.isaac) & set(self.python))
        out = {'isaac_frames': self.frames, 'isaac_ball_frames': len(self.isaac),
               'python_ball_frames': len(self.python), 'both': len(common),
               'isaac_only': len(set(self.isaac) - set(self.python)),
               'python_only': len(set(self.python) - set(self.isaac)),
               'classes_seen': dict(self.classes)}
        if self.latency:
            lat = np.array(self.latency)
            out['isaac_latency_ms'] = {'median': round(float(np.median(lat)), 1),
                                       'p95': round(float(np.percentile(lat, 95)), 1)}
        if self.py_latency:
            lat = np.array(self.py_latency)
            out['python_latency_ms'] = {'median': round(float(np.median(lat)), 1),
                                        'p95': round(float(np.percentile(lat, 95)), 1)}
        for name, (px, py) in (('top_left', (0.0, 0.0)), ('centered', self.pad)):
            errs = [np.hypot((self.isaac[k][0] - px) / self.scale - self.python[k][0],
                             (self.isaac[k][1] - py) / self.scale - self.python[k][1]) for k in common]
            if errs:
                out[f'pixel_diff_{name}_padding'] = {'median': round(float(np.median(errs)), 2),
                                                     'max': round(float(np.max(errs)), 2)}
        return out


def main():
    duration = float(sys.argv[1])
    w, h = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else (424, 240)
    rclpy.init()
    node = Compare(w, h)
    end = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(json.dumps(node.summary(), indent=1))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
