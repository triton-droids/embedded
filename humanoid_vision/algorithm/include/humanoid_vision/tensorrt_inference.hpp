/** @file tensorrt_inference.hpp
 * @brief Opaque TensorRT 10 backend with reusable CUDA input/output buffers.
 */
#pragma once

#include <memory>
#include <string>
#include <opencv2/core.hpp>

/** @brief Fixed-shape, batch-one engine with one FP32 device input and output.
 * CUDA details stay in Impl so CPU builds can include this header. Engines are
 * platform-dependent artifacts and should be generated on the deployment device.
 * This object owns mutable buffers and is not intended for concurrent inference.
 */
class TensorRtInference {
public:
  /** @brief Deserialize an engine whose input is [1,3,height,width].
   * @throws std::runtime_error For incompatible tensors, CUDA errors, bad engine
   * files, or a build without TensorRT support.
   */
  TensorRtInference(const std::string &engine_path, int width, int height);
  /** @brief Wait for pending stream work, then release context and owned storage. */
  ~TensorRtInference();
  /** @brief Execute a contiguous FP32 NCHW blob and synchronize before returning.
   * @return A shallow cv::Mat view of reusable host output; the next call overwrites
   * its contents. Clone it if the output must survive another inference call.
   */
  cv::Mat forward(const cv::Mat &blob);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
