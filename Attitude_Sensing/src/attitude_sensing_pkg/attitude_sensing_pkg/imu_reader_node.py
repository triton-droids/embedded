# imu_reader_node.py
from __future__ import annotations

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from .imu_read import iter_imu_samples, RK4DeadReckoner


def q_to_msg(q: np.ndarray) -> Quaternion:
    # q=[w,x,y,z] -> geometry_msgs Quaternion(x,y,z,w)
    m = Quaternion()
    m.w = float(q[0])
    m.x = float(q[1])
    m.y = float(q[2])
    m.z = float(q[3])
    return m


class ImuReaderNode(Node):
    def __init__(self):
        super().__init__("imu_reader_node")

        # parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_odom", False)
        self.declare_parameter("use_rk4_orientation", True)
        self.declare_parameter("acc_units", "m/s^2")   # or "g"
        self.declare_parameter("gyro_units", "rad/s")  # or "deg/s"

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_odom = bool(self.get_parameter("publish_odom").value)
        self.use_rk4_orientation = bool(self.get_parameter("use_rk4_orientation").value)

        acc_units = self.get_parameter("acc_units").value
        gyro_units = self.get_parameter("gyro_units").value

        self.pub_imu = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10) if self.publish_odom else None

        self.integrator = RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665))

        # 用线程/定时器拉取生成器
        self.gen = iter_imu_samples(
            source="serial",
            port=port,
            baud=baud,
            rate_hz=None,
            include_all=False,
            integrator=self.integrator,
            acc_units=acc_units,
            gyro_units=gyro_units,
        )

        self.timer = self.create_timer(0.0, self._tick)  # 0.0 -> 尽快调度（由串口阻塞控制节奏）

    def _tick(self):
        try:
            s = next(self.gen)
        except Exception as e:
            self.get_logger().error(f"IMU read error: {e}")
            return

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        # angular velocity (rad/s)
        imu.angular_velocity.x = float(s["gyro_rad_s"][0])
        imu.angular_velocity.y = float(s["gyro_rad_s"][1])
        imu.angular_velocity.z = float(s["gyro_rad_s"][2])

        # linear acceleration (m/s^2) 这里是机体系原始加速度（包含重力）
        imu.linear_acceleration.x = float(s["acc_m_s2"][0])
        imu.linear_acceleration.y = float(s["acc_m_s2"][1])
        imu.linear_acceleration.z = float(s["acc_m_s2"][2])

        if self.use_rk4_orientation and "quat_wb" in s:
            imu.orientation = q_to_msg(s["quat_wb"])
        # 否则 orientation 留默认 0（下游滤波器会自己算）

        self.pub_imu.publish(imu)

        if self.publish_odom and self.pub_odom is not None and "lin_pos_m" in s:
            od = Odometry()
            od.header = imu.header
            od.child_frame_id = "base_link"
            od.pose.pose.position.x = float(s["lin_pos_m"][0])
            od.pose.pose.position.y = float(s["lin_pos_m"][1])
            od.pose.pose.position.z = float(s["lin_pos_m"][2])
            if self.use_rk4_orientation and "quat_wb" in s:
                od.pose.pose.orientation = q_to_msg(s["quat_wb"])
            od.twist.twist.linear.x = float(s["lin_vel_m_s"][0])
            od.twist.twist.linear.y = float(s["lin_vel_m_s"][1])
            od.twist.twist.linear.z = float(s["lin_vel_m_s"][2])
            self.pub_odom.publish(od)


def main():
    rclpy.init()
    node = ImuReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()