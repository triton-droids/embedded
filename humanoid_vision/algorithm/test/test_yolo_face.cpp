#include "humanoid_vision/detection_postprocess.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0]
              << " MODEL.onnx INPUT_IMAGE OUTPUT_IMAGE\n";
    return 2;
  }

  const cv::Mat image = cv::imread(argv[2], cv::IMREAD_COLOR);
  if (image.empty()) {
    std::cerr << "Failed to read test image: " << argv[2] << '\n';
    return 2;
  }

  try {
    YoloDetector::Config config;
    config.confidence_threshold = 0.25F;
    config.nms_threshold = 0.45F;
    config.use_cuda = false;

    YoloDetector detector(argv[1], config);
    const std::vector<Detection2D> detections = detector.detect(image);

    cv::Mat annotated = image.clone();
    std::cout << "detections=" << detections.size() << '\n';

    for (const Detection2D &detection : detections) {
      std::cout << "class=" << detection.class_id
                << " confidence=" << detection.confidence
                << " box=" << detection.box.x << ',' << detection.box.y << ','
                << detection.box.width << ',' << detection.box.height << '\n';

      cv::rectangle(annotated, detection.box, cv::Scalar(0, 255, 0), 2);
      cv::putText(annotated, std::to_string(detection.confidence),
                  detection.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 255, 0), 2);
    }

    if (!cv::imwrite(argv[3], annotated)) {
      std::cerr << "Failed to write result image: " << argv[3] << '\n';
      return 2;
    }

    return detections.empty() ? 1 : 0;
  } catch (const std::exception &error) {
    std::cerr << "Face detection failed: " << error.what() << '\n';
    return 2;
  }
}
