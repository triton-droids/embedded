import rclpy
from rclpy.node import Node

from .robstride_motor_linux import RobStrideMotorLinux


class MotorControlNode(Node):
    """
    Minimal demo node:
    - Open CAN interface can0
    - Enable one RobStride motor
    - Periodically send a simple velocity command so the motor moves
    """

    def __init__(self):
        super().__init__('motor_control_node')

        # --- Hard-coded motor configuration (adjust to your setup) ---
        iface = 'can0'        # Linux SocketCAN interface
        master_id =  1     # Usually 0xFF for the PC/master
        motor_id = 127       # Change to your motor's CAN ID
        actuator_type = 2     # 0..6, depends on your motor type

        self.get_logger().info(
            f'Initializing RobStride motor on {iface}, '
            f'master_id=0x{master_id:02X}, motor_id=0x{motor_id:02X}, '
            f'actuator_type={actuator_type}'
        )

        # Create the low-level driver instance
        self.motor = RobStrideMotorLinux(
            iface=iface,
            master_id=master_id,
            motor_id=motor_id,
            actuator_type=actuator_type,
        )

        # Enable motor once at startup
        try:
            pos, vel, tq, temp = self.motor.enable_motor()
            self.get_logger().info(
                f'Motor enabled. '
                f'pos={pos:.3f} rad, vel={vel:.3f} rad/s, '
                f'torque={tq:.3f} Nm, temp={temp:.1f} °C'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to enable motor: {e}')

        # Commanded velocity [rad/s]; keep it small for safety
        self.command_velocity = 2.0

        # Timer to send velocity command periodically (20 Hz)
        self._step = 0
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        """
        Periodically send a velocity command to make the motor move.
        For now we just send a constant velocity.
        """
        try:
            pos, vel, tq, temp = self.motor.send_velocity_mode_command(
                self.command_velocity
            )

            # Only log every 20 ticks to avoid spamming (about once per second)
            self._step += 1
            if self._step % 20 == 0:
                self.get_logger().info(
                    f'cmd_vel={self.command_velocity:.2f} rad/s | '
                    f'fb pos={pos:.3f} rad, vel={vel:.3f} rad/s, '
                    f'torque={tq:.3f} Nm, temp={temp:.1f} °C'
                )

        except Exception as e:
            self.get_logger().error(f'Error sending velocity command: {e}')

    def destroy_node(self):
        """
        Make sure the motor is stopped before the node is destroyed.
        """
        try:
            self.get_logger().info('Stopping motor before shutdown...')
            # Send zero velocity once
            try:
                self.motor.send_velocity_mode_command(0.0)
            except Exception:
                pass

            # Disable motor (clear_error = 0)
            self.motor.disable_motor(0)
            self.get_logger().info('Motor disabled.')
        except Exception as e:
            self.get_logger().warn(f'Error while stopping motor: {e}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
