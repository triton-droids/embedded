cmake_minimum_required(VERSION 3.8)
project(motor_control_hybrid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# C++ executable
add_executable(cpp_control_node src/cpp_control_node.cpp)
ament_target_dependencies(cpp_control_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install
install(TARGETS
  cpp_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package with setup.py
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
install(PROGRAMS
  motor_control_hybrid/python_can_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME python_can_node
)
ament_package()
