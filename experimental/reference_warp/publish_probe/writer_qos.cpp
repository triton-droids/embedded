// Read the actual DDS writer, rather than inferring its QoS from environment variables.
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rcl/publisher.h>
#include <rmw_fastrtps_shared_cpp/custom_publisher_info.hpp>
#include <fastrtps/config.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("writer_qos_readback");
  std::cout << "header_strict_realtime=" << HAVE_STRICT_REALTIME << '\n';
  for (const auto *topic : {"/imu/data_raw", "/joint_states", "/safety/estop"}) {
    auto pub = node->create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::QoS(10).reliable());
    auto *handle = rcl_publisher_get_rmw_handle(pub->get_publisher_handle().get());
    auto *info = static_cast<CustomPublisherInfo *>(handle->data);
    auto qos = info->data_writer_->get_qos();
    std::cout << topic << " max_blocking_ns=" << qos.reliability().max_blocking_time.to_ns()
              << " publish_mode=" << static_cast<int>(qos.publish_mode().kind)
              << " memory_policy=" << qos.endpoint().history_memory_policy
              << " history=" << static_cast<int>(qos.history().kind) << " depth=" << qos.history().depth
              << " data_sharing=" << static_cast<int>(qos.data_sharing().kind()) << '\n';
  }
  std::ifstream maps("/proc/self/maps");
  std::string line;
  while (std::getline(maps, line)) {
    if ((line.find("libfastrtps.so") != std::string::npos ||
         line.find("librmw_fastrtps") != std::string::npos) && line.find("r-xp") != std::string::npos)
      std::cout << line << '\n';
  }
  rclcpp::shutdown();
}
