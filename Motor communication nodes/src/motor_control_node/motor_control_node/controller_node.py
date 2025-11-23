# robstride_motor_ros/controller_node.py

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from motor_control_interfaces.srv import RobStrideJointControl



class RobStrideControllerNode(Node):
    """
    High-level controller node.

    - Subscribes to /joint_commands (sensor_msgs/JointState).
      This is where your RL policy or any higher-level controller
      can publish desired joint targets (positions / velocities).
    - For each incoming joint command, calls the /robstride_joint_control
      service exposed by RobStrideCanNode, using joint_name as the key.
    """

    def __init__(self):
        super().__init__('robstride_controller_node')

        # Parameters:
        #   service_name: name of the service provided by the CAN node
        #   default_mode: 'position_pp' or 'velocity' (how to interpret commands)
        self.declare_parameter('service_name', 'robstride_joint_control')
        self.declare_parameter('default_mode', 'position_pp')  # or 'velocity'

        self.service_name = self.get_parameter(
            'service_name').get_parameter_value().string_value
        self.default_mode = self.get_parameter(
            'default_mode').get_parameter_value().string_value

        # Service client
        self.client = self.create_client(RobStrideJointControl, self.service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for service {self.service_name}...')

        # Subscribe to high-level joint commands
        self.sub_cmd = self.create_subscription(
            JointState,
            'joint_commands',
            self.joint_commands_callback,
            10
        )

        self.get_logger().info(
            f'RobStride controller node started. mode={self.default_mode}, '
            f'service={self.service_name}'
        )

    # -------- Callback: convert JointState to service calls --------

    def joint_commands_callback(self, msg: JointState):
        """
        Called whenever a new JointState is published on /joint_commands.

        For each joint name in msg.name, we build a RobStrideJointControl
        request and call the service asynchronously.
        """
        # Sanity check
        if not msg.name:
            return

        for i, joint in enumerate(msg.name):
            req = RobStrideJointControl.Request()
            req.joint_name = joint

            # Choose control mode based on default_mode parameter
            if self.default_mode == 'velocity':
                # Velocity control: use msg.velocity as target
                req.command_type = RobStrideJointControl.Request.CMD_VELOCITY
                req.velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
                req.acceleration = 0.0  # can be parameterized later if needed

            else:
                # Position-PP control (position + optional velocity)
                req.command_type = RobStrideJointControl.Request.CMD_POSITION_PP
                req.position = msg.position[i] if i < len(msg.position) else 0.0
                req.velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
                req.acceleration = 0.0  # can be parameterized

            self.get_logger().debug(
                f'Sending command for joint={joint}, mode={self.default_mode}, '
                f'pos={req.position:.3f}, vel={req.velocity:.3f}'
            )

            future = self.client.call_async(req)
            # Attach a small callback to log the result (optional)
            future.add_done_callback(
                lambda f, j=joint: self._handle_response(j, f)
            )

    # -------- Handle service responses (mostly for logging / debugging) --------

    def _handle_response(self, joint: str, future):
        try:
            resp = future.result()
            if resp is None:
                return

            if resp.success:
                # Use DEBUG to avoid spamming logs; you can switch to INFO for debugging
                self.get_logger().debug(
                    f'{joint}: OK pos={resp.position:.3f}, '
                    f'vel={resp.velocity:.3f}, '
                    f'tau={resp.torque:.3f}, '
                    f'T={resp.temperature:.1f}°C'
                )
            else:
                self.get_logger().warn(
                    f'{joint}: command failed: {resp.message}'
                )
        except Exception as e:
            self.get_logger().error(f'Service call failed for {joint}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RobStrideControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
