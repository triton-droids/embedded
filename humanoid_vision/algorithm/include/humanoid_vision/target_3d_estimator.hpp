/** @file target_3d_estimator.hpp
 * @brief Estimate a camera-frame point using a box-center ray and local depth.
 * Inputs must already share pixel geometry and suitable acquisition times;
 * these helpers perform neither image alignment nor time synchronization.
 */
#pragma once

#include <optional>

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "humanoid_vision/types.hpp"

#ifdef HUMANOID_VISION_HAS_REALSENSE
#include <librealsense2/rs.hpp>

/** @brief Deproject the box center using RealSense intrinsics/distortion handling.
 * @param box Bounding box in the supplied aligned depth frame's pixel coordinates.
 * @param intrinsics Intrinsics corresponding to that pixel geometry.
 * @param depth_frame Frame exposing distances in meters.
 * @return Optical-frame XYZ in meters, or nullopt for unusable box/depth.
 * Depth is the upper median of positive finite samples in a centered 7x7 patch.
 */
std::optional<Eigen::Vector3f>
project_2d_to_3d(const cv::Rect2f &box, const rs2_intrinsics &intrinsics,
                 const rs2::depth_frame &depth_frame);
#endif

/** @brief Pinhole deprojection of a ROS box center with robust local median depth.
 * @param box Box in aligned color/depth pixel coordinates.
 * @param depth_msg 16UC1 millimeters or 32FC1 meters, already aligned to color.
 * @param camera_info Intrinsics for that image geometry. Distortion is not applied;
 * use appropriately rectified inputs when pinhole accuracy is required.
 * @return Camera optical XYZ (x right, y down, z forward), meters; nullopt when
 * inputs/depth/focal lengths are unusable. cv_bridge conversion errors may throw.
 */
std::optional<Eigen::Vector3f> project_2d_to_3d(
    const cv::Rect2f &box,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info);
