/** @file types.hpp
 * @brief Shared detection value types; these are C++ structs, not ROS messages.
 */
#pragma once

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

/** @brief Model class, confidence, and box in original image pixel coordinates. */
struct Detection2D {
  int class_id;
  float confidence;
  cv::Rect2f box;  ///< Top-left x/y and width/height in pixels, after undoing padding.
};

/** @brief Reserved richer target record; not currently emitted by the ROS node.
 * Track ID and depth-validity ratio require a future association/quality stage.
 */
struct Detection3D {
  int class_id;
  float confidence;
  int track_id;
  Eigen::Vector3f position;  ///< XYZ meters in the producer's declared frame.
  float valid_depth_ratio;  ///< Intended fraction [0,1] of usable depth samples.
};
