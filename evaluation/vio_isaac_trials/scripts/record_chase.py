"""Record a chase for video: kinematic state (shared memory, 15 Hz) + Isaac detections + chased
pixel + what the robot believes: /localization/odom and the chased target's camera-frame point.

  python3 record_chase.py DURATION_S OUT.npz      (sim with share_state:=true)
"""
import json
import shutil
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from humanoid_sim.state_shm import StateReader


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class Rec(Node):
    def __init__(self):
        super().__init__('chase_recorder')
        self.detections, self.targets, self.poses, self.points, self.cmds = [], [], [], [], []
        self.create_subscription(Detection2DArray, '/isaac_yolo/detections', self._det, 10)
        self.create_subscription(PointStamped, '/vision/target_center', self._tgt, 10)
        self.create_subscription(PointStamped, '/vision/ball_3d_position', self._pt, 10)
        self.create_subscription(Odometry, '/localization/odom', self._odom, 10)
        self.create_subscription(TwistStamped, '/ball_chase/cmd_vel', self._cmd, 10)

    def _det(self, msg):
        boxes = [(int(d.results[0].hypothesis.class_id), float(d.results[0].hypothesis.score),
                  d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y)
                 for d in msg.detections if d.results]
        self.detections.append((ns(msg.header.stamp), boxes))

    def _pt(self, msg):
        self.points.append((ns(msg.header.stamp), msg.point.x, msg.point.y, msg.point.z))

    def _odom(self, msg):
        if len(self.poses) and ns(msg.header.stamp) - self.poses[-1][0] < 50_000_000:
            return  # 20 Hz is plenty for the video.
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.poses.append((ns(msg.header.stamp), p.x, p.y, p.z, q.w, q.x, q.y, q.z))

    def _cmd(self, msg):
        self.cmds.append((ns(msg.header.stamp), msg.twist.linear.x, msg.twist.angular.z))

    def _tgt(self, msg):
        self.targets.append((ns(msg.header.stamp), msg.point.x, msg.point.y))


def main():
    duration, out = float(sys.argv[1]), sys.argv[2]
    rclpy.init()
    reader = StateReader(timeout_s=120)
    node = Rec()
    rows, last = [], None
    end = time.monotonic() + duration
    next_t = time.monotonic()
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=max(0.0, next_t - time.monotonic()))
        if time.monotonic() >= next_t:
            next_t += 1 / 15
            rec = reader.read()
            if rec is not None and rec[0] != last:
                last = rec[0]
                rows.append((rec[0], rec[1].copy(), rec[2].copy(), rec[3].copy()))
    np.savez_compressed(out, stamp=np.array([r[0] for r in rows]), qpos=np.array([r[1] for r in rows]),
                        mocap_pos=np.array([r[2] for r in rows]), mocap_quat=np.array([r[3] for r in rows]))
    with open(out.replace('.npz', '.json'), 'w') as f:
        json.dump({'detections': node.detections, 'targets': node.targets,
                   'poses': node.poses, 'points': node.points, 'cmds': node.cmds}, f)
    shutil.copy(reader.model_path, out.replace('.npz', '.mjb'))
    print(f'saved {len(rows)} frames, {len(node.detections)} detection arrays, {len(node.targets)} targets, '
          f'{len(node.poses)} poses, {len(node.points)} points')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
