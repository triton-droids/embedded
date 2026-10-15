#include "humanoid_vision/target_3d_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#ifdef HUMANOID_VISION_HAS_REALSENSE
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

std::optional<Eigen::Vector3f>
project_2d_to_3d(const cv::Rect2f &box, const rs2_intrinsics &intrinsics,
                 const rs2::depth_frame &depth_frame) {
  if (box.width <= 0 || box.height <= 0) {
    return std::nullopt; // Return empty optional if the bounding box is invalid
  }

  // Calculate the center of the bounding box
  float center_x = box.x + box.width / 2.0f;
  float center_y = box.y + box.height / 2.0f;

  if (center_x < 0 || center_x >= depth_frame.get_width() || center_y < 0 ||
      center_y >= depth_frame.get_height()) {
    return std::nullopt; // Return empty optional if the center is out of bounds
  }

  constexpr int radius = 3; // Centered 7x7 depth region.
  const int center_pixel_x = static_cast<int>(center_x);
  const int center_pixel_y = static_cast<int>(center_y);

  const int x_min = std::max(0, center_pixel_x - radius);
  const int x_max =
      std::min(depth_frame.get_width() - 1, center_pixel_x + radius);

  const int y_min = std::max(0, center_pixel_y - radius);
  const int y_max =
      std::min(depth_frame.get_height() - 1, center_pixel_y + radius);

  std::vector<float> valid_depths;

  for (int y = y_min; y <= y_max; ++y) {
    for (int x = x_min; x <= x_max; ++x) {
      const float value_m = depth_frame.get_distance(x, y);

      if (std::isfinite(value_m) && value_m > 0.0F) {
        valid_depths.push_back(value_m);
      }
    }
  }

  if (valid_depths.empty()) {
    return std::nullopt;
  }

  const auto middle = valid_depths.begin() + valid_depths.size() / 2;
  std::nth_element(valid_depths.begin(), middle, valid_depths.end());

  const float depth_value = *middle;

  float pixel[2] = {static_cast<float>(center_x), static_cast<float>(center_y)};
  float point[3];
  rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_value);
  return Eigen::Vector3f{point[0], point[1], point[2]};
}
#endif

// ROS2 Override

std::optional<Eigen::Vector3f> project_2d_to_3d(
    const cv::Rect2f &box,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info) {
  if (!depth_msg || !camera_info || box.width <= 0.0F || box.height <= 0.0F) {
    return std::nullopt;
  }

  const int center_x = static_cast<int>(std::lround(box.x + box.width * 0.5F));

  const int center_y = static_cast<int>(std::lround(box.y + box.height * 0.5F));

  if (center_x < 0 || center_x >= static_cast<int>(depth_msg->width) ||
      center_y < 0 || center_y >= static_cast<int>(depth_msg->height)) {
    return std::nullopt;
  }

  const cv_bridge::CvImageConstPtr depth_cv =
      cv_bridge::toCvShare(depth_msg, depth_msg->encoding);

  std::vector<float> valid_depths;
  constexpr int radius = 3; // Centered 7x7 depth region.

  const int x_min = std::max(0, center_x - radius);
  const int x_max =
      std::min(static_cast<int>(depth_msg->width) - 1, center_x + radius);

  const int y_min = std::max(0, center_y - radius);
  const int y_max =
      std::min(static_cast<int>(depth_msg->height) - 1, center_y + radius);

  for (int y = y_min; y <= y_max; ++y) {
    for (int x = x_min; x <= x_max; ++x) {
      float value_m = 0.0F;

      if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        const std::uint16_t value_mm = depth_cv->image.at<std::uint16_t>(y, x);

        value_m = static_cast<float>(value_mm) * 0.001F;
      } else if (depth_msg->encoding ==
                 sensor_msgs::image_encodings::TYPE_32FC1) {
        value_m = depth_cv->image.at<float>(y, x);
      } else {
        return std::nullopt;
      }

      if (std::isfinite(value_m) && value_m > 0.0F) {
        valid_depths.push_back(value_m);
      }
    }
  }

  if (valid_depths.empty()) {
    return std::nullopt;
  }

  const auto middle = valid_depths.begin() + valid_depths.size() / 2;

  std::nth_element(valid_depths.begin(), middle, valid_depths.end());

  const float depth_m = *middle;

  if (!std::isfinite(depth_m) || depth_m <= 0.0F) {
    return std::nullopt;
  }

  const float fx = static_cast<float>(camera_info->k[0]);
  const float fy = static_cast<float>(camera_info->k[4]);
  const float cx = static_cast<float>(camera_info->k[2]);
  const float cy = static_cast<float>(camera_info->k[5]);

  if (fx <= 0.0F || fy <= 0.0F) {
    return std::nullopt;
  }

  const float x = (static_cast<float>(center_x) - cx) * depth_m / fx;

  const float y = (static_cast<float>(center_y) - cy) * depth_m / fy;

  return Eigen::Vector3f{x, y, depth_m};
}
