/** @file test_yolo_face.cpp
 * @brief CLI smoke/regression test and optional throughput benchmark for YOLO.
 * MODEL INPUT_IMAGE OUTPUT_IMAGE are required. --reference-face verifies the
 * bundled Lena fixture; --cuda selects TensorRT; --benchmark repeats inference.
 */
#include "humanoid_vision/detection_postprocess.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

/** @brief Decode one image, write its overlay, and optionally check a reference.
 * @return 0 for success, 1 for no detection/reference mismatch, 2 for setup/IO
 * errors or exceptions. Benchmark time includes preprocessing and decoding.
 */
int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " MODEL INPUT_IMAGE OUTPUT_IMAGE [--reference-face] [--cuda] [--benchmark]\n";
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
    bool reference_face = false, benchmark = false;
    for (int i = 4; i < argc; ++i) {
      const std::string flag(argv[i]);
      if (flag == "--cuda") config.use_cuda = true;
      else if (flag == "--reference-face") reference_face = true;
      else if (flag == "--benchmark") benchmark = true;
      else throw std::invalid_argument("Unknown flag: " + flag);
    }

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

    // Only enable these numeric assertions for the matching model/Lena fixture;
    // arbitrary user images are valid smoke inputs without this flag.
    if (reference_face) {
      if (detections.size() != 1) {
        return 1;
      }
      const auto &face = detections.front();
      // Reference from the original graph: catches incorrect box decoding.
      if (std::abs(face.confidence - 0.833417F) > 0.01F ||
          std::abs(face.box.x - 212) > 2 || std::abs(face.box.y - 186) > 2 ||
          std::abs(face.box.width - 144) > 2 ||
          std::abs(face.box.height - 203) > 2) {
        std::cerr << "Face differs from the reference detection\n";
        return 1;
      }
    }
    // Warm up backend allocations/caches before timing repeated identical inputs.
    if (benchmark) {
      for (int i = 0; i < 5; ++i) detector.detect(image);
      const auto started = std::chrono::steady_clock::now();
      constexpr int iterations = 30;
      for (int i = 0; i < iterations; ++i) detector.detect(image);
      const double ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count() / iterations;
      std::cout << "backend=" << (config.use_cuda ? "TensorRT CUDA FP16" : "OpenCV CPU")
                << " mean_ms=" << ms << " throughput_fps=" << 1000.0 / ms << '\n';
    }
    return detections.empty() ? 1 : 0;
  } catch (const std::exception &error) {
    std::cerr << "Face detection failed: " << error.what() << '\n';
    return 2;
  }
}
