"""Simulation-only ball throw/reset controls; never commands the robot."""
import math

import mujoco
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState


class BallControl(Node):
    def __init__(self, world):
        super().__init__('sim_ball_control')
        self.world = world
        self.joint = world.model.joint('chase_ball_free')
        self.position_pub = self.create_publisher(PointStamped, '/ball_chase/ground_truth/ball', 2)
        self.head_pub = self.create_publisher(JointState, '/head/joint_states', 2)
        self.create_subscription(PointStamped, '/ball_chase/throw_position', self.place, 2)
        self.create_subscription(Twist, '/ball_chase/throw_velocity', self.throw, 2)
        self.create_service(Trigger, '/ball_chase/reset_ball', self.reset_ball)
        self.create_timer(0.1, self.publish_position)

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
