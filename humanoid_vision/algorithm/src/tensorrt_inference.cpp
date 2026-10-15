#include "humanoid_vision/tensorrt_inference.hpp"

#ifdef HUMANOID_VISION_HAS_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>

static_assert(NV_TENSORRT_MAJOR >= 10, "TensorRT 10 or newer is required");

namespace {
void checkCuda(cudaError_t status) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA: ") + cudaGetErrorString(status));
  }
}
struct Logger : nvinfer1::ILogger {
  void log(Severity severity, const char *message) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::cerr << "TensorRT: " << message << '\n';
    }
  }
};
struct DeviceBuffer {
  void *data = nullptr;
  ~DeviceBuffer() { if (data) cudaFree(data); }
};
struct Stream {
  cudaStream_t handle = nullptr;
  ~Stream() { if (handle) cudaStreamDestroy(handle); }
};
}

struct TensorRtInference::Impl {
  ~Impl() { if (stream.handle) cudaStreamSynchronize(stream.handle); }
  Logger logger;
  std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;
  // Keep device storage and the stream alive until the context is destroyed.
  DeviceBuffer input, output;
  Stream stream;
  std::unique_ptr<nvinfer1::IExecutionContext> context;
  std::vector<int> input_shape;
  cv::Mat host_output;
  size_t input_bytes = 0;
  size_t output_bytes = 0;
};

TensorRtInference::TensorRtInference(const std::string &engine_path,
                                   int width, int height)
    : impl_(std::make_unique<Impl>()) {
  auto &state = *impl_;
  std::ifstream file(engine_path, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot open TensorRT engine: " + engine_path);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(file), {}};
  if (bytes.empty()) throw std::runtime_error("Empty TensorRT engine");
  state.runtime.reset(nvinfer1::createInferRuntime(state.logger));
  if (!state.runtime) throw std::runtime_error("Cannot create TensorRT runtime");
  state.engine.reset(state.runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
  if (!state.engine) throw std::runtime_error("Cannot load TensorRT engine; rebuild it on this Jetson");
  if (state.engine->getNbIOTensors() != 2) {
    throw std::runtime_error("Expected one TensorRT input and one output");
  }
  state.context.reset(state.engine->createExecutionContext());
  if (!state.context) throw std::runtime_error("Cannot create TensorRT context");
  checkCuda(cudaStreamCreate(&state.stream.handle));
  bool found_input = false, found_output = false;
  for (int i = 0; i < state.engine->getNbIOTensors(); ++i) {
    const char *name = state.engine->getIOTensorName(i);
    const auto shape = state.engine->getTensorShape(name);
    if (state.engine->getTensorDataType(name) != nvinfer1::DataType::kFLOAT ||
        state.engine->getTensorLocation(name) != nvinfer1::TensorLocation::kDEVICE) {
      throw std::runtime_error("TensorRT IO must use FP32 device tensors");
    }
    std::vector<int> dims;
    for (int j = 0; j < shape.nbDims; ++j) {
      if (shape.d[j] <= 0) throw std::runtime_error("Dynamic TensorRT shapes are unsupported");
      dims.push_back(shape.d[j]);
    }
    if (state.engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
      if (found_input || dims != std::vector<int>{1, 3, height, width}) {
        throw std::runtime_error("TensorRT input must match [1,3,height,width]");
      }
      found_input = true;
      state.input_shape = dims;
      state.input_bytes = static_cast<size_t>(3) * height * width * sizeof(float);
      checkCuda(cudaMalloc(&state.input.data, state.input_bytes));
      if (!state.context->setTensorAddress(name, state.input.data)) {
        throw std::runtime_error("Cannot bind TensorRT input");
      }
    } else {
      if (found_output || dims.size() != 3 || dims[0] != 1 ||
          dims[1] < 5 || dims[1] >= dims[2]) {
        throw std::runtime_error("Expected TensorRT YOLO output [1,4+classes,anchors]");
      }
      found_output = true;
      state.host_output.create(static_cast<int>(dims.size()), dims.data(), CV_32F);
      state.output_bytes = state.host_output.total() * sizeof(float);
      checkCuda(cudaMalloc(&state.output.data, state.output_bytes));
      if (!state.context->setTensorAddress(name, state.output.data)) {
        throw std::runtime_error("Cannot bind TensorRT output");
      }
    }
  }
  if (!found_input || !found_output) throw std::runtime_error("Incomplete TensorRT IO");
}

TensorRtInference::~TensorRtInference() = default;

cv::Mat TensorRtInference::forward(const cv::Mat &blob) {
  auto &state = *impl_;
  if (blob.type() != CV_32F || !blob.isContinuous() || blob.dims != 4 ||
      std::vector<int>(blob.size.p, blob.size.p + blob.dims) != state.input_shape) {
    throw std::runtime_error("Invalid TensorRT input blob");
  }
  checkCuda(cudaMemcpyAsync(state.input.data, blob.data, state.input_bytes,
                            cudaMemcpyHostToDevice, state.stream.handle));
  if (!state.context->enqueueV3(state.stream.handle)) {
    cudaStreamSynchronize(state.stream.handle);
    throw std::runtime_error("TensorRT CUDA inference failed");
  }
  checkCuda(cudaMemcpyAsync(state.host_output.data, state.output.data,
                            state.output_bytes, cudaMemcpyDeviceToHost,
                            state.stream.handle));
  checkCuda(cudaStreamSynchronize(state.stream.handle));
  return state.host_output;
}

#else
#include <stdexcept>
struct TensorRtInference::Impl {};
TensorRtInference::TensorRtInference(const std::string &, int, int) {
  throw std::runtime_error("TensorRT support is unavailable");
}
TensorRtInference::~TensorRtInference() = default;
cv::Mat TensorRtInference::forward(const cv::Mat &) {
  throw std::runtime_error("TensorRT support is unavailable");
}
#endif
