cmake_minimum_required(VERSION 3.8)
project(humanoid_vision)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn highgui imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 QUIET)

add_library(${PROJECT_NAME}_algorithm SHARED
  algorithm/src/depth_sampler.cpp
  algorithm/src/deprojection.cpp
  algorithm/src/detection_postprocess.cpp
  algorithm/src/target_3d_estimator.cpp
  algorithm/src/target_filter.cpp
  algorithm/src/target_tracker.cpp
)

target_compile_features(${PROJECT_NAME}_algorithm PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algorithm/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_algorithm
  Eigen3::Eigen
  ${OpenCV_LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_algorithm
  cv_bridge
  sensor_msgs
)

if(realsense2_FOUND)
  target_compile_definitions(${PROJECT_NAME}_algorithm
    PUBLIC HUMANOID_VISION_HAS_REALSENSE=1
  )

  if(TARGET realsense2::realsense2)
    target_link_libraries(${PROJECT_NAME}_algorithm realsense2::realsense2)
  else()
    target_include_directories(${PROJECT_NAME}_algorithm
      PUBLIC ${realsense2_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}_algorithm ${realsense2_LIBRARY})
  endif()
else()
  message(STATUS
    "librealsense2 not found: building only the ROS 2 depth-image overload")
endif()

add_executable(ball_detection_node src/ball_detection_node.cpp)
target_compile_features(ball_detection_node PRIVATE cxx_std_17)
target_link_libraries(ball_detection_node
  ${PROJECT_NAME}_algorithm
  ${OpenCV_LIBS}
)
ament_target_dependencies(ball_detection_node
  ament_index_cpp
  cv_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
)

add_executable(image_viewer_node src/image_viewer_node.cpp)
target_compile_features(image_viewer_node PRIVATE cxx_std_17)
target_link_libraries(image_viewer_node ${OpenCV_LIBS})
ament_target_dependencies(image_viewer_node
  cv_bridge
  rclcpp
  sensor_msgs
)

install(
  TARGETS
    ${PROJECT_NAME}_algorithm
    ball_detection_node
    image_viewer_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY algorithm/include/
  DESTINATION include
)

install(
  FILES models/yolov8n-face-lindevs.onnx
  DESTINATION share/${PROJECT_NAME}/models
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cv_bridge Eigen3 OpenCV sensor_msgs)

ament_package()
