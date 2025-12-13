#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

from .imu_i2c_reader import MPU6050Reader


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_normalize(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=float)


def quat_rotate(q, v):
    q = quat_normalize(q)
    vq = np.array([v[0], v[1], v[2], 0.0], dtype=float)
    q_conj = np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)
    return quat_mul(quat_mul(q, vq), q_conj)[:3]


def quat_to_rpy(q):
    x, y, z, w = quat_normalize(q)

    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuDRRK4Node(Node):
    """
      /lin_pos (odom): Vector3Stamped [m]
      /lin_vel (odom): Vector3Stamped [m/s]
      /ang_vel (base_link): Vector3Stamped [rad/s] 
      /ang_rpy (base_link): Vector3Stamped [rad] 
    """

    def __init__(self):
        super().__init__('imu_dr_rk4')

        # ----- source -----
        self.declare_parameter('source', 'topic')   # 'topic' or 'i2c'
        self.declare_parameter('imu_topic', '/imu')

        # i2c params
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x68)
        self.declare_parameter('i2c_rate_hz', 200.0)  

        # stationary detection (IMU only)
        self.declare_parameter('eps_gyro_norm', 0.08)
        self.declare_parameter('eps_acc_g', 0.25)

        # bias update
        self.declare_parameter('alpha_gyro_bias', 0.01)

        # gravity handling
        self.declare_parameter('acc_includes_gravity', True)
        self.declare_parameter('gravity_vector', [0.0, 0.0, -9.80665])

        # dt guard
        self.declare_parameter('max_dt', 0.2)

        # ZUPT
        self.declare_parameter('enable_zupt', True)

        # orientation correction (only meaningful if topic provides orientation)
        self.declare_parameter('orientation_correction_gain', 0.0)

        self.source = str(self.get_parameter('source').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)

        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.i2c_addr = int(self.get_parameter('i2c_addr').value)
        self.i2c_rate_hz = float(self.get_parameter('i2c_rate_hz').value)

        self.eps_gyro_norm = float(self.get_parameter('eps_gyro_norm').value)
        self.eps_acc_g = float(self.get_parameter('eps_acc_g').value)
        self.alpha_bias = float(self.get_parameter('alpha_gyro_bias').value)

        self.acc_includes_gravity = bool(self.get_parameter('acc_includes_gravity').value)
        self.g_vec = np.array(self.get_parameter('gravity_vector').value, dtype=float)
        self.g_mag = float(np.linalg.norm(self.g_vec))

        self.max_dt = float(self.get_parameter('max_dt').value)
        self.enable_zupt = bool(self.get_parameter('enable_zupt').value)
        self.ori_corr_gain = float(self.get_parameter('orientation_correction_gain').value)

        # ----- state -----
        self.t_prev = None
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.a_prev = np.zeros(3)

        self.gyro_bias = np.zeros(3)
        self.omega_prev_raw = np.zeros(3)

        self.q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        # ----- pubs -----
        self.pub_lin_pos = self.create_publisher(Vector3Stamped, '/lin_pos', 10)
        self.pub_lin_vel = self.create_publisher(Vector3Stamped, '/lin_vel', 10)
        self.pub_ang_vel = self.create_publisher(Vector3Stamped, '/ang_vel', 10)
        self.pub_ang_rpy = self.create_publisher(Vector3Stamped, '/ang_rpy', 10)

        # ----- inputs -----
        self.sub = None
        self.timer = None
        self.i2c = None

        if self.source == 'topic':
            self.sub = self.create_subscription(Imu, self.imu_topic, self.cb_imu_msg, 50)
            self.get_logger().info(f"imu_dr_rk4 source=topic, subscribing {self.imu_topic}")
        elif self.source == 'i2c':
            # I2C reader
            self.i2c = MPU6050Reader(bus_id=self.i2c_bus, addr=self.i2c_addr)
            period = 1.0 / max(self.i2c_rate_hz, 1.0)
            self.timer = self.create_timer(period, self.cb_i2c_tick)
            self.get_logger().info(f"imu_dr_rk4 source=i2c, /dev/i2c-{self.i2c_bus} addr=0x{self.i2c_addr:02x}, rate={self.i2c_rate_hz}Hz")
        else:
            raise RuntimeError("parameter 'source' must be 'topic' or 'i2c'")

    def destroy_node(self):
        if self.i2c is not None:
            self.i2c.close()
        super().destroy_node()

    # ---------- RK4 helpers ----------
    def _q_dot(self, q, omega):
        ox, oy, oz = omega
        omega_q = np.array([ox, oy, oz, 0.0], dtype=float)
        return 0.5 * quat_mul(q, omega_q)

    def integrate_quat_rk4(self, q0, omega0, omega1, dt):
        def omega_of_tau(tau):
            return omega0 + (omega1 - omega0) * (tau / dt)

        k1 = self._q_dot(q0, omega_of_tau(0.0))
        k2 = self._q_dot(q0 + 0.5 * dt * k1, omega_of_tau(0.5 * dt))
        k3 = self._q_dot(q0 + 0.5 * dt * k2, omega_of_tau(0.5 * dt))
        k4 = self._q_dot(q0 + dt * k3, omega_of_tau(dt))

        q_new = q0 + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return quat_normalize(q_new)

    def integrate_pv_rk4(self, p0, v0, a0, a1, dt):
        def a_of_tau(tau):
            return a0 + (a1 - a0) * (tau / dt)

        def f(s, tau):
            v = s[3:]
            a = a_of_tau(tau)
            return np.hstack([v, a])

        s0 = np.hstack([p0, v0])
        k1 = f(s0, 0.0)
        k2 = f(s0 + 0.5*dt*k1, 0.5*dt)
        k3 = f(s0 + 0.5*dt*k2, 0.5*dt)
        k4 = f(s0 + dt*k3, dt)
        s1 = s0 + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        return s1[:3], s1[3:]

    # ---------- input: I2C ----------
    def cb_i2c_tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9

        try:
            acc_body, omega_raw = self.i2c.read()
        except Exception as e:
            self.get_logger().error(f"I2C read failed: {e}")
            return

        q_msg = None
        self.process_sample(t, np.array(omega_raw, float), np.array(acc_body, float), q_msg)

    # ---------- input: ROS Imu msg ----------
    def cb_imu_msg(self, msg: Imu):
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        omega_raw = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=float)
        acc_body = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z], dtype=float)
        q_msg = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype=float)
        if np.linalg.norm(q_msg) < 1e-6:
            q_msg = None
        self.process_sample(t, omega_raw, acc_body, q_msg)

    # ---------- shared processing ----------
    def process_sample(self, t, omega_raw, acc_body, q_msg):
        if self.t_prev is None:
            self.t_prev = t
            if q_msg is not None:
                self.q = quat_normalize(q_msg)
            self.omega_prev_raw = omega_raw.copy()
            self.a_prev = np.zeros(3)
            return

        dt = t - self.t_prev
        self.t_prev = t
        if dt <= 0.0 or dt > self.max_dt:
            self.omega_prev_raw = omega_raw.copy()
            return

        omega_norm = float(np.linalg.norm(omega_raw))
        acc_norm = float(np.linalg.norm(acc_body))
        stationary = (omega_norm < self.eps_gyro_norm) and (abs(acc_norm - self.g_mag) < self.eps_acc_g)

        # ZARU
        if stationary:
            self.gyro_bias = (1.0 - self.alpha_bias) * self.gyro_bias + self.alpha_bias * omega_raw

        omega = omega_raw - self.gyro_bias  # corrected

        # attitude RK4
        omega0 = (self.omega_prev_raw - self.gyro_bias)
        omega1 = omega
        self.q = self.integrate_quat_rk4(self.q, omega0, omega1, dt)

        # optional correction (topic mode only)
        if (self.ori_corr_gain > 0.0) and (q_msg is not None):
            self.q = quat_normalize((1.0 - self.ori_corr_gain) * self.q + self.ori_corr_gain * quat_normalize(q_msg))

        # acc to world
        acc_world = quat_rotate(self.q, acc_body)
        if self.acc_includes_gravity:
            a_world = acc_world - self.g_vec
        else:
            a_world = acc_world

        # integrate p,v RK4
        if stationary and self.enable_zupt:
            self.v[:] = 0.0
        else:
            self.p, self.v = self.integrate_pv_rk4(self.p, self.v, self.a_prev, a_world, dt)

        self.a_prev = a_world
        self.omega_prev_raw = omega_raw.copy()

        roll, pitch, yaw = quat_to_rpy(self.q)
        yaw = wrap_pi(yaw)

        self._pub_vec3(self.pub_lin_pos, t, self.p, 'odom')
        self._pub_vec3(self.pub_lin_vel, t, self.v, 'odom')
        self._pub_vec3(self.pub_ang_vel, t, omega, 'base_link')
        self._pub_vec3(self.pub_ang_rpy, t, np.array([roll, pitch, yaw]), 'base_link')

    def _pub_vec3(self, pub, t_sec, vec3, frame_id=''):
        msg = Vector3Stamped()
        msg.header.stamp.sec = int(t_sec)
        msg.header.stamp.nanosec = int((t_sec - int(t_sec)) * 1e9)
        msg.header.frame_id = frame_id
        msg.vector.x = float(vec3[0])
        msg.vector.y = float(vec3[1])
        msg.vector.z = float(vec3[2])
        pub.publish(msg)


def main():
    rclpy.init()
    node = ImuDRRK4Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
