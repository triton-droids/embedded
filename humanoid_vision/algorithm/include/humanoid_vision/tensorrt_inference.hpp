#pragma once

#include <memory>
#include <string>
#include <opencv2/core.hpp>

// Fixed-shape, single-input/output TensorRT engine. All CUDA details stay private.
class TensorRtInference {
public:
  TensorRtInference(const std::string &engine_path, int width, int height);
  ~TensorRtInference();
  cv::Mat forward(const cv::Mat &blob);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
