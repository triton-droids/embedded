/** @file tensorrt_inference.hpp
 * @brief Opaque TensorRT 10 backend with reusable CUDA input/output buffers.
 */
#pragma once

#include <memory>
#include <string>
#include <opencv2/core.hpp>

/** @brief Fixed-shape, batch-one engine with one device input and one FP32 output.
 * The input is either an FP32 NCHW blob [1,3,height,width] or, for engines whose
 * ONNX carries its own preprocessing (scripts/make_u8_input_onnx.py), a uint8
 * letterboxed BGR image [1,height,width,3] that is normalized on the GPU.
 * CUDA details stay in Impl so CPU builds can include this header. Engines are
 * platform-dependent artifacts and should be generated on the deployment device.
 * This object owns mutable buffers and is not intended for concurrent inference.
 */
class TensorRtInference {
public:
  /** @brief Deserialize an engine whose input is [1,3,height,width] FP32 or
   * [1,height,width,3] uint8.
   * @throws std::runtime_error For incompatible tensors, CUDA errors, bad engine
   * files, or a build without TensorRT support.
   */
  TensorRtInference(const std::string &engine_path, int width, int height);
  /** @brief Wait for pending stream work, then release context and owned storage. */
  ~TensorRtInference();
  /** @brief True when the engine takes the letterboxed BGR uint8 image directly. */
  bool takes_bgr_u8() const;
  /** @brief Execute one input and synchronize before returning.
   * @param input A contiguous FP32 NCHW blob, or a CV_8UC3 height x width BGR image
   * when takes_bgr_u8() is true.
   * @return A shallow cv::Mat view of reusable host output; the next call overwrites
   * its contents. Clone it if the output must survive another inference call.
   */
  cv::Mat forward(const cv::Mat &input);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
