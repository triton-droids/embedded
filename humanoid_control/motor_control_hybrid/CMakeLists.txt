cmake_minimum_required(VERSION 3.18)
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
find_package(diagnostic_msgs REQUIRED)
find_package(motor_control_interfaces REQUIRED)

# =========================
# Non-ROS control core lib (from repo root: ../../Interfaces/control_core)
# =========================
set(CONTROL_CORE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../Interfaces/control_core)

add_library(control_core SHARED
  ${CONTROL_CORE_ROOT}/src/api.cpp
  ${CONTROL_CORE_ROOT}/src/operation.cpp
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
  diagnostic_msgs
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
# ENCOS motor node: prefer an installed driver, otherwise fetch a pinned revision.
# =========================
option(MOTOR_CONTROL_BUILD_ENCOS "Build the ENCOS CAN backend" ON)
option(MOTOR_CONTROL_FETCH_ENCOS "Download encos_driver if not installed" ON)
if(MOTOR_CONTROL_BUILD_ENCOS)
  include(cmake/EncosDriver.cmake)
  motor_control_find_encos_driver()
  find_package(yaml-cpp REQUIRED)
  add_executable(encos_can_node src/encos_can_node.cpp)
  target_compile_features(encos_can_node PRIVATE cxx_std_17)
  ament_target_dependencies(encos_can_node
    rclcpp
    sensor_msgs
    std_msgs
    motor_control_interfaces
  )
  target_link_libraries(encos_can_node Encos::encos_driver yaml-cpp)
  install(TARGETS encos_can_node DESTINATION lib/${PROJECT_NAME})
endif()

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

# Install deployment models and their configuration-sync XML snapshots
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(tracking_onnx_test test/test_tracking_onnx.py TIMEOUT 90)
  ament_add_pytest_test(tracking_node_test test/test_tracking_node.py TIMEOUT 90)
  ament_add_pytest_test(inference_deadlines_test test/test_inference_deadlines.py TIMEOUT 30)
  ament_add_pytest_test(control_watchdogs_test test/test_control_watchdogs.py TIMEOUT 30)
  ament_add_pytest_test(model_control_settings_test test/test_model_control_settings.py TIMEOUT 30)
  ament_add_pytest_test(can_transport_test test/test_can_transport.py TIMEOUT 30)
  ament_add_pytest_test(motor_services_test test/test_motor_services.py TIMEOUT 30)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(cpp_state_watchdog_test test/test_cpp_state_watchdog.cpp)
  ament_target_dependencies(cpp_state_watchdog_test rclcpp sensor_msgs std_msgs diagnostic_msgs motor_control_interfaces)
  target_link_libraries(cpp_state_watchdog_test control_core)
  ament_add_gtest(core_operation_test test/test_core_operation.cpp)
  target_link_libraries(core_operation_test control_core)
  set_tests_properties(cpp_state_watchdog_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=179;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/cpp-state-watchdog-logs")
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
install(PROGRAMS
  motor_control_hybrid/policy_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME policy_bridge_node
)

install(PROGRAMS
  motor_control_hybrid/tracking_policy_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tracking_policy_node
)

install(PROGRAMS
  motor_control_hybrid/joint_state_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_monitor_node
)

ament_package()
