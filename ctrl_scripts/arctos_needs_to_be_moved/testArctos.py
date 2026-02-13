import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajClient(Node):
    def __init__(self):
        super().__init__('traj_client')
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/arctos_controller/follow_joint_trajectory')

    def send(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['X_joint','Y_joint','Z_joint','B_joint','A_joint','C_joint']

        p = JointTrajectoryPoint()
        p.positions = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
        p.time_from_start.sec = 2
        goal.trajectory.points = [p]

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"result: {result_future.result().result.error_code}")

def main():
    rclpy.init()
    node = TrajClient()
    node.send()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()