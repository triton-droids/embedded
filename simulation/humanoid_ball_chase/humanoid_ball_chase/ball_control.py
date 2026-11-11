"""Simulation-only ball throw/reset controls; never commands the robot."""
import math
import time

import mujoco
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class BallControl(Node):
    def __init__(self, world):
        super().__init__('sim_ball_control')
        self.world = world
        self.joint = world.model.joint('chase_ball_free')
        self.base_id = world.model.body('floating_base').id
        self.position_pub = self.create_publisher(PointStamped, '/ball_chase/ground_truth/ball', 2)
        self.head_pub = self.create_publisher(JointState, '/head/joint_states', 2)
        self.create_subscription(PointStamped, '/ball_chase/throw_position', self.place, 2)
        self.create_subscription(Twist, '/ball_chase/throw_velocity', self.throw, 2)
        self.create_service(Trigger, '/ball_chase/reset_ball', self.reset_ball)
        # [radius_m, speed_m_s]: the ball circles the robot's current floor position at a constant
        # speed (positive: counter-clockwise seen from above). [] or speed 0 stops it in place.
        self.create_subscription(Float32MultiArray, '/ball_chase/orbit', self.set_orbit, 2)
        self.orbit = None  # (radius, speed, angle)
        self._orbit_t = None
        self.create_timer(0.01, self.drive_orbit)
        self.create_timer(0.1, self.publish_position)

    def set_orbit(self, msg):
        values = list(msg.data)
        if len(values) != 2 or not all(math.isfinite(v) for v in values) or abs(values[1]) < 1e-6:
            if self.orbit is not None:
                with self.world.lock:
                    v = self.joint.dofadr[0]
                    self.world.data.qvel[v:v+6] = 0
            self.orbit, self._orbit_t = None, None
            return
        radius, speed = values
        if not 0.3 <= radius <= 10 or abs(speed) > 5:
            self.get_logger().warn('Orbit needs radius in [0.3, 10] m and |speed| <= 5 m/s')
            return
        with self.world.lock:
            center = self.world.data.xpos[self.base_id]
            ball = self.world.data.body('chase_ball').xpos
            angle = math.atan2(ball[1] - center[1], ball[0] - center[0])
        self.orbit, self._orbit_t = (radius, speed, angle), None
        self.get_logger().info(f'Ball orbits the robot: radius {radius:.2f} m, speed {speed:.2f} m/s')

    def drive_orbit(self):
        """Kinematic ball: placed on the circle with its tangential velocity at 100 Hz."""
        if self.orbit is None:
            return
        now = time.monotonic()
        dt = 0.0 if self._orbit_t is None else min(now - self._orbit_t, 0.1)
        self._orbit_t = now
        radius, speed, angle = self.orbit
        angle += speed / radius * dt
        self.orbit = (radius, speed, angle)
        c, s = math.cos(angle), math.sin(angle)
        with self.world.lock:
            center = self.world.data.xpos[self.base_id]
            q, v = self.joint.qposadr[0], self.joint.dofadr[0]
            self.world.data.qpos[q:q+3] = [center[0] + radius * c, center[1] + radius * s, 0.11]
            self.world.data.qvel[v:v+3] = [-speed * s, speed * c, 0.0]
            self.world.data.qvel[v+3:v+6] = 0

    def place(self, msg):
        p = [msg.point.x, msg.point.y, msg.point.z]
        if msg.header.frame_id != 'world' or not all(math.isfinite(v) for v in p) or p[2] < 0.1:
            self.get_logger().warn('Ball position needs finite world coordinates and z >= 0.1 m')
            return
        with self.world.lock:
            q, v = self.joint.qposadr[0], self.joint.dofadr[0]
            self.world.data.qpos[q:q+7] = [*p,1,0,0,0]
            self.world.data.qvel[v:v+6] = 0
            mujoco.mj_forward(self.world.model, self.world.data)

    def throw(self, msg):
        values = [msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        if not all(math.isfinite(v) and abs(v) <= 10 for v in values):
            return
        with self.world.lock:
            v = self.joint.dofadr[0]
            self.world.data.qvel[v:v+6] = values

    def reset_ball(self, request, response):
        p = PointStamped()
        p.header.frame_id = 'world'
        p.point.x, p.point.y, p.point.z = -0.25, 1.8, 0.11
        self.place(p)
        response.success = True
        response.message = 'Ball reset; robot state unchanged'
        return response

    def publish_position(self):
        with self.world.lock:
            p = self.world.data.body('chase_ball').xpos.copy()
            head = JointState()
            head.header.stamp = self.get_clock().now().to_msg()
            head.name = ['head_yaw_joint','head_pitch_joint']
            head.position = [float(self.world.data.qpos[self.world.model.joint(n).qposadr[0]]) for n in head.name]
            head.velocity = [float(self.world.data.qvel[self.world.model.joint(n).dofadr[0]]) for n in head.name]
        self.head_pub.publish(head)
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x,msg.point.y,msg.point.z = map(float,p)
        self.position_pub.publish(msg)


def create_nodes(world):
    return [BallControl(world)]
