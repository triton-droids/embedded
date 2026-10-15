cmake_minimum_required(VERSION 3.8)
project(humanoid_vision)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Prefer the project-local Ubuntu 4.5 SDK when available. Select it before
# cv_bridge, whose dependency lookup also calls find_package(OpenCV).
set(_opencv45_config "${CMAKE_CURRENT_SOURCE_DIR}/../.deps/opencv45/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4")
if(EXISTS "${_opencv45_config}/OpenCVConfig.cmake")
  set(OpenCV_DIR "${_opencv45_config}" CACHE PATH "OpenCV 4.5 SDK" FORCE)
endif()
find_package(OpenCV 4.5 REQUIRED COMPONENTS core dnn highgui imgcodecs imgproc)
if(NOT OpenCV_VERSION VERSION_LESS "4.6")
  message(FATAL_ERROR "humanoid_vision requires OpenCV 4.5.x. Run scripts/setup_opencv45.sh or set OpenCV_DIR to a 4.5 SDK.")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 QUIET)

add_library(${PROJECT_NAME}_algorithm SHARED
  algorithm/src/tensorrt_inference.cpp
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
target_link_libraries(${PROJECT_NAME}_algorithm PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_algorithm PUBLIC
  cv_bridge
  sensor_msgs
)

option(ENABLE_TENSORRT "Enable CUDA inference with TensorRT 10" ON)
if(ENABLE_TENSORRT)
  find_package(CUDAToolkit QUIET)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATH_SUFFIXES aarch64-linux-gnu)
  find_library(TENSORRT_LIBRARY nvinfer)
  if(CUDAToolkit_FOUND AND TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY)
    target_compile_definitions(${PROJECT_NAME}_algorithm PUBLIC HUMANOID_VISION_HAS_TENSORRT=1)
    target_include_directories(${PROJECT_NAME}_algorithm PRIVATE ${TENSORRT_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME}_algorithm PRIVATE CUDA::cudart ${TENSORRT_LIBRARY})
    message(STATUS "CUDA inference: TensorRT enabled")
  else()
    message(STATUS "CUDA inference: TensorRT/CUDA not found; CPU only")
  endif()
endif()

if(realsense2_FOUND)
  target_compile_definitions(${PROJECT_NAME}_algorithm
    PUBLIC HUMANOID_VISION_HAS_REALSENSE=1
  )

  if(TARGET realsense2::realsense2)
    target_link_libraries(${PROJECT_NAME}_algorithm PUBLIC realsense2::realsense2)
  else()
    target_include_directories(${PROJECT_NAME}_algorithm
      PUBLIC ${realsense2_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}_algorithm PUBLIC ${realsense2_LIBRARY})
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
target_include_directories(image_viewer_node PRIVATE algorithm/include)
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
  FILES models/yolov8n-face-lindevs-opencv45.onnx
  DESTINATION share/${PROJECT_NAME}/models
)

install(FILES ../.deps/tensorrt/yolov8n-face-fp16.engine
  DESTINATION share/${PROJECT_NAME}/models OPTIONAL)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cv_bridge Eigen3 OpenCV sensor_msgs)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_yolo_face algorithm/test/test_yolo_face.cpp)
  target_link_libraries(test_yolo_face ${PROJECT_NAME}_algorithm ${OpenCV_LIBS})
  add_test(NAME yolo_face_cpu COMMAND test_yolo_face
    ${CMAKE_CURRENT_SOURCE_DIR}/models/yolov8n-face-lindevs-opencv45.onnx
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/test/data/lena.jpg
    ${CMAKE_CURRENT_BINARY_DIR}/face_test_result.jpg
    --reference-face
  )
endif()

ament_package()
