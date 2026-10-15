#pragma once

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

struct Detection2D {
  int class_id;
  float confidence;
  cv::Rect2f box;
};

struct Detection3D {
  int class_id;
  float confidence;
  int track_id;
  Eigen::Vector3f position;
  float valid_depth_ratio;
};
