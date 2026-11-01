// Probe real publisher loan capability. A positive result alone is NOT proof
// of zero-copy delivery to an application subscriber.
#include <iostream>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <motor_control_interfaces/msg/motor_command.hpp>

template<class Message>
void inspect(const rclcpp::Node::SharedPtr& node, const char* name) {
  auto writer = node->create_publisher<Message>(
    std::string("/research/zero_copy/")+name, rclcpp::QoS(2).reliable());
  const bool capable = writer->can_loan_messages();
  std::cout << name << " depth=" << writer->get_actual_qos().depth()
            << " publisher_can_loan=" << capable;
  if (capable) {
    auto loan = writer->borrow_loaned_message();
    std::cout << " native_loan_valid=" << loan.is_valid();
    // Default-valued research messages only; never publish to control topics.
    writer->publish(std::move(loan));
  }
  std::cout << '\n';
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("zero_copy_capability");
  std::cout << "rmw=" << rmw_get_implementation_identifier() << '\n';
  inspect<sensor_msgs::msg::Imu>(node,"imu");
  inspect<sensor_msgs::msg::Image>(node,"image");
  inspect<sensor_msgs::msg::JointState>(node,"joint_state");
  inspect<motor_control_interfaces::msg::MotorCommand>(node,"motor_command");
  inspect<std_msgs::msg::Float32>(node,"float32");
  rclcpp::shutdown();
}
