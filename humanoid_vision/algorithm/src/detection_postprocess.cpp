#include "humanoid_vision/detection_postprocess.hpp"
#include "humanoid_vision/tensorrt_inference.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "humanoid_vision/cv_bridge_compat.hpp"
#include <sensor_msgs/image_encodings.hpp>

YoloDetector::YoloDetector(const std::string &model_path)
    : YoloDetector(model_path, Config{}) {}

YoloDetector::YoloDetector(const std::string &model_path, const Config &config)
    : config_(config) {
  if (config_.input_width <= 0 || config_.input_height <= 0) {
    throw std::invalid_argument("YOLO input dimensions must be positive");
  }
  if (config_.use_cuda) {
#ifdef HUMANOID_VISION_HAS_TENSORRT
    tensorrt_ = std::make_unique<TensorRtInference>(
        model_path, config_.input_width, config_.input_height);
#else
    throw std::runtime_error("CUDA requested but this package was built without TensorRT");
#endif
  } else {
    net_ = cv::dnn::readNetFromONNX(model_path);
    if (net_.empty()) throw std::runtime_error("Failed to load YOLO ONNX model: " + model_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }
}

YoloDetector::~YoloDetector() = default;

YoloDetector::LetterboxResult
YoloDetector::letterbox(const cv::Mat &image) const {
  const float scale =
      std::min(static_cast<float>(config_.input_width) / image.cols,
               static_cast<float>(config_.input_height) / image.rows);

  const int resized_width =
      static_cast<int>(static_cast<float>(image.cols) * scale);
  const int resized_height =
      static_cast<int>(static_cast<float>(image.rows) * scale);

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(resized_width, resized_height));

  const int remaining_width = config_.input_width - resized_width;
  const int remaining_height = config_.input_height - resized_height;
  const int pad_left = remaining_width / 2;
  const int pad_right = remaining_width - pad_left;
  const int pad_top = remaining_height / 2;
  const int pad_bottom = remaining_height - pad_top;

  cv::Mat padded;
  cv::copyMakeBorder(resized, padded, pad_top, pad_bottom, pad_left, pad_right,
                     cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

  return LetterboxResult{std::move(padded), scale, pad_left, pad_top};
}

std::vector<Detection2D> YoloDetector::detect(const cv::Mat &bgr_image) {
  if (bgr_image.empty()) {
    return {};
  }

  const LetterboxResult input = letterbox(bgr_image);

  const cv::Mat blob = cv::dnn::blobFromImage(
      input.image, 1.0 / 255.0,
      cv::Size(config_.input_width, config_.input_height), cv::Scalar(), true,
      false);

  std::vector<cv::Mat> outputs;
  if (tensorrt_) {
#ifdef HUMANOID_VISION_HAS_TENSORRT
    outputs.push_back(tensorrt_->forward(blob));
#endif
  } else {
    net_.setInput(blob);
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());
  }

  if (outputs.empty()) {
    return {};
  }

  const cv::Mat &output = outputs.front();
  cv::Mat predictions;

  if (output.dims == 3) {
    const int first_dimension = output.size[1];
    const int second_dimension = output.size[2];
    const cv::Mat raw(first_dimension, second_dimension, CV_32F,
                      const_cast<float *>(output.ptr<float>()));

    // Ultralytics exports commonly use [1, 4 + classes, anchors].
    // Some exporters transpose it to [1, anchors, 4 + classes].
    if (first_dimension < second_dimension) {
      cv::transpose(raw, predictions);
    } else {
      predictions = raw;
    }
  } else if (output.dims == 2) {
    predictions = output;
  } else {
    throw std::runtime_error("Unsupported YOLO output tensor dimensions");
  }

  if (predictions.cols < 5) {
    throw std::runtime_error("Unsupported YOLO output tensor shape");
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;

  for (int row_index = 0; row_index < predictions.rows; ++row_index) {
    const float *row = predictions.ptr<float>(row_index);

    const cv::Mat class_scores(1, predictions.cols - 4, CV_32F,
                               const_cast<float *>(row + 4));
    cv::Point best_class;
    double best_score = 0.0;
    cv::minMaxLoc(class_scores, nullptr, &best_score, nullptr, &best_class);

    const float confidence = static_cast<float>(best_score);
    if (confidence < config_.confidence_threshold) {
      continue;
    }

    const float center_x = (row[0] - input.pad_x) / input.scale;
    const float center_y = (row[1] - input.pad_y) / input.scale;
    const float width = row[2] / input.scale;
    const float height = row[3] / input.scale;

    const int left = std::clamp(static_cast<int>(center_x - width * 0.5F), 0,
                                bgr_image.cols - 1);
    const int top = std::clamp(static_cast<int>(center_y - height * 0.5F), 0,
                               bgr_image.rows - 1);
    const int right = std::clamp(static_cast<int>(center_x + width * 0.5F), 0,
                                 bgr_image.cols);
    const int bottom = std::clamp(static_cast<int>(center_y + height * 0.5F), 0,
                                  bgr_image.rows);

    if (right <= left || bottom <= top) {
      continue;
    }

    boxes.emplace_back(left, top, right - left, bottom - top);
    confidences.push_back(confidence);
    class_ids.push_back(best_class.x);
  }

  std::vector<int> kept_indices;
  cv::dnn::NMSBoxes(boxes, confidences, config_.confidence_threshold,
                    config_.nms_threshold, kept_indices);

  std::vector<Detection2D> detections;
  detections.reserve(kept_indices.size());

  for (const int index : kept_indices) {
    detections.push_back(Detection2D{class_ids[index], confidences[index],
                                     cv::Rect2f(boxes[index])});
  }

  return detections;
}

std::vector<Detection2D>
YoloDetector::detect(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg) {
  if (!image_msg) {
    return {};
  }

  const cv_bridge::CvImageConstPtr image_cv =
      cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);

  return detect(image_cv->image);
}
