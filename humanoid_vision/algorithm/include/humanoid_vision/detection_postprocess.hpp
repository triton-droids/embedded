/** @file detection_postprocess.hpp
 * @brief YOLO preprocessing, backend dispatch, and box decoding API.
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <sensor_msgs/msg/image.hpp>

#include "humanoid_vision/types.hpp"

class TensorRtInference;

/** @brief Batch-one YOLO detector returning boxes in original image pixels.
 * Supports YOLO-style center-x/center-y/width/height plus class scores; separate
 * objectness and landmark output formats are not decoded. An instance owns
 * mutable inference buffers and should be called serially.
 */
class YoloDetector {
public:
  /** @brief Model dimensions and thresholds; CUDA selects a TensorRT engine file. */
  struct Config {
    int input_width = 640;
    int input_height = 640;
    float confidence_threshold = 0.25F;
    float nms_threshold = 0.45F;
    bool use_cuda = false;
  };

  /** @brief Release backend resources; defined out of line for the opaque backend. */
  ~YoloDetector();
  /** @brief Load an ONNX model with default CPU settings. */
  explicit YoloDetector(const std::string &model_path);
  /** @brief Load ONNX (CPU) or a serialized engine (CUDA) using explicit settings.
   * @throws std::invalid_argument For invalid input dimensions.
   * @throws std::runtime_error If the selected backend/model is unavailable.
   */
  YoloDetector(const std::string &model_path, const Config &config);

  /** @brief Infer a BGR image; return confidence-filtered, NMS-selected pixel boxes.
   * @return Empty detections for an empty input or an empty backend output.
   */
  std::vector<Detection2D> detect(const cv::Mat &bgr_image);
  /** @brief Convert/share a ROS image as BGR8 and invoke the image overload.
   * A null message produces no detections; conversion errors propagate.
   */
  std::vector<Detection2D>
  detect(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);

private:
  /** @brief Padded model input plus the inverse transform needed for box decoding. */
  struct LetterboxResult {
    cv::Mat image;
    float scale;
    int pad_x;
    int pad_y;
  };

  /** @brief Resize without changing aspect ratio, then pad with YOLO gray (114).
   * scale maps original pixels to resized pixels; pad_x/pad_y are left/top pads.
   */
  LetterboxResult letterbox(const cv::Mat &image) const;

  std::unique_ptr<TensorRtInference> tensorrt_;
  cv::dnn::Net net_;
  Config config_;
};
