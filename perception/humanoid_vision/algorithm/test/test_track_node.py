"""Verify visual tracking with synthetic measurements on isolated ROS domain 71.

Source ROS and the tracker install, then run with /usr/bin/python3. ``--launch``
exercises the unified launch with camera, detector, and Agent disabled. Commands
are remapped to /track_test/command; no physical servo command topic is used.
The final plant simulation checks pixel centering with an ideal instantaneous
servo, not real actuator dynamics, transport latency, backlash, or camera noise.
"""
import math
import os
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '71'

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32


def main():
    """Start a test controller, verify output behavior, and stop its process group.

    Raises:
        AssertionError: Startup, freshness, direction, limits, or centering fails.
        subprocess.TimeoutExpired: The child fails to exit within cleanup timeout.
    """
    command = [
        'ros2', 'run', 'humanoid_vision', 'track_node', '--ros-args',
        '-p', 'target_topic:=/track_test/center',
        '-p', 'camera_info_topic:=/track_test/info',
        '-p', 'command_topic:=/track_test/command',
        '-p', 'max_angle_rad:=0.12', '-p', 'min_angle_rad:=-0.12',
        '-p', 'max_speed_rad_s:=0.2', '-p', 'max_accel_rad_s2:=0.5',
    ]
    if '--launch' in sys.argv:
        command = [
            'ros2', 'launch', 'humanoid_vision', 'head_tracking.launch.py',
            'start_camera:=false', 'start_agent:=false', 'start_detector:=false',
            'target_topic:=/track_test/center', 'camera_info_topic:=/track_test/info',
            'command_topic:=/track_test/command',
            'max_angle_rad:=0.12', 'min_angle_rad:=-0.12',
            'max_speed_rad_s:=0.2', 'max_accel_rad_s2:=0.5',
        ]
    process = subprocess.Popen(command, start_new_session=True)
    rclpy.init()
    node = Node('track_integration_test')
    centers = node.create_publisher(PointStamped, '/track_test/center', qos_profile_sensor_data)
    infos = node.create_publisher(CameraInfo, '/track_test/info', qos_profile_sensor_data)
    received = []
    sub = node.create_subscription(Float32, '/track_test/command',
                                   lambda msg: received.append((time.monotonic(), msg.data)), 100)
    # Synthetic 640x480 pinhole camera: centered principal point, focal length 500 px.
    info = CameraInfo()
    info.header.frame_id = 'camera_color_optical_frame'
    info.width, info.height = 640, 480
    info.k = [500., 0., 320., 0., 500., 240., 0., 0., 1.]

    def run(seconds, pixel=None, stale=False, send_info=True, plant=False):
        """Publish a scenario while spinning the command subscriber.

        Args:
            seconds: Scenario duration measured with monotonic wall time.
            pixel: Optional (u, v) tuple; None simulates missing detections.
            stale: Shift image acquisition stamps two seconds into the past.
            send_info: Publish intrinsics; false tests missing calibration.
            plant: Recompute the target pixel using the latest angle command and
                a fixed world-target offset of 0.08 rad on each iteration.
        """
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            stamp = node.get_clock().now().to_msg()
            info.header.stamp = stamp
            if send_info:
                infos.publish(info)
            if plant:
                # Fixed world target: changing the joint command shifts its
                # observed pixel position. The ideal servo follows instantly.
                joint_angle = received[-1][1] if received else 0.0
                pixel = (320. + 500. * math.tan(0.08 + joint_angle), 240.)
            if pixel is not None:
                point = PointStamped()
                point.header.frame_id = info.header.frame_id
                point.header.stamp = stamp
                if stale:
                    point.header.stamp.sec -= 2
                point.point.x, point.point.y = pixel
                centers.publish(point)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)

    try:
        deadline = time.monotonic() + 10
        while centers.get_subscription_count() == 0:
            if process.poll() is not None or time.monotonic() > deadline:
                raise AssertionError('tracker did not start')
            rclpy.spin_once(node, timeout_sec=0.1)
        # Readiness gates must prevent output even with an off-center detection.
        run(0.3, (480., 240.), send_info=False)
        assert not received, 'motion without CameraInfo'
        run(0.3)
        assert not received, 'motion without target'
        # Constant right-side error should integrate negatively with direction=-1.
        run(1.8, (480., 240.))
        assert len(received) > 30, 'insufficient command samples'
        assert received[-1][1] < -0.10, 'wrong direction / no integration'
        assert all(math.isfinite(v) and abs(v) <= 0.120001 for _, v in received)
        # Centered target stays still after acceleration-limited deceleration.
        run(0.5, (320., 240.))
        angle = received[-1][1]
        run(0.3, (324., 240.))
        assert abs(received[-1][1] - angle) < 1e-5, 'deadband drift'
        run(1.2, (160., 240.))
        assert received[-1][1] > angle + 0.10, 'target reversal failed'
        # After the freshness grace period, no new commands may be published.
        run(0.3)
        count = len(received)
        run(0.4)
        assert len(received) == count, 'continued output after target loss'
        run(0.4, (480., 240.), stale=True)
        assert len(received) == count, 'stale detections drove servo'
        run(0.4, (float('nan'), 240.))
        assert len(received) == count, 'NaN detection drove servo'
        run(5.0, plant=True)
        final_pixel_error = 500. * math.tan(0.08 + received[-1][1])
        assert abs(final_pixel_error) <= 9., 'closed-loop image centering failed'
        print('PASS: missing intrinsics/target, direction, integration, limits, '
              'deadband, reversal, loss, stale timestamp, NaN, simulated image '
              f'centering (error={final_pixel_error:.2f}px); hardware untouched')
    finally:
        # ros2 run starts a child executable; terminate the isolated process group.
        import signal
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
