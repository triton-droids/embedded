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
find_package(motor_control_interfaces REQUIRED)

# =========================
# Non-ROS control core lib (from repo root: ../../Interfaces/control_core)
# =========================
set(CONTROL_CORE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../Interfaces/control_core)

add_library(control_core SHARED
  ${CONTROL_CORE_ROOT}/src/api.cpp
)

target_include_directories(control_core PUBLIC
  ${CONTROL_CORE_ROOT}/include
)


# (No ROS deps here by design)

# =========================
# C++ executable
# =========================
add_executable(cpp_control_node src/cpp_control_node.cpp)

set_target_properties(cpp_control_node PROPERTIES
  BUILD_RPATH "$ORIGIN"
  INSTALL_RPATH "$ORIGIN"
)
ament_target_dependencies(cpp_control_node
  rclcpp
  sensor_msgs
  std_msgs
  motor_control_interfaces
)

# Link the control core library
target_link_libraries(cpp_control_node
  control_core
)

# Make sure the node can include headers
target_include_directories(cpp_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CONTROL_CORE_ROOT}/include
)

# =========================
# Install
# =========================
install(TARGETS
  control_core
  cpp_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install public headers (so other packages/programs can include them if needed)
install(DIRECTORY include/
  DESTINATION include
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
install(PROGRAMS
  motor_control_hybrid/motor_sdk_gateway_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_sdk_gateway_node
)
install(PROGRAMS
  motor_control_hybrid/fake_motor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_motor_node
)
install(PROGRAMS
  motor_control_hybrid/double_pendulum_websocket_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME double_pendulum_websocket_node
)

ament_package()
