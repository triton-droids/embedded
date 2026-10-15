#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <sensor_msgs/msg/image.hpp>

#include "humanoid_vision/types.hpp"

class TensorRtInference;

class YoloDetector {
public:
  struct Config {
    int input_width = 640;
    int input_height = 640;
    float confidence_threshold = 0.25F;
    float nms_threshold = 0.45F;
    bool use_cuda = false;
  };

  ~YoloDetector();
  explicit YoloDetector(const std::string &model_path);
  YoloDetector(const std::string &model_path, const Config &config);

  std::vector<Detection2D> detect(const cv::Mat &bgr_image);
  std::vector<Detection2D>
  detect(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);

private:
  struct LetterboxResult {
    cv::Mat image;
    float scale;
    int pad_x;
    int pad_y;
  };

  LetterboxResult letterbox(const cv::Mat &image) const;

  std::unique_ptr<TensorRtInference> tensorrt_;
  cv::dnn::Net net_;
  Config config_;
};
