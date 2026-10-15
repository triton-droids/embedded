#pragma once

#include <optional>

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "humanoid_vision/types.hpp"

#ifdef HUMANOID_VISION_HAS_REALSENSE
#include <librealsense2/rs.hpp>

std::optional<Eigen::Vector3f>
project_2d_to_3d(const cv::Rect2f &box, const rs2_intrinsics &intrinsics,
                 const rs2::depth_frame &depth_frame);
#endif

std::optional<Eigen::Vector3f> project_2d_to_3d(
    const cv::Rect2f &box,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info);
