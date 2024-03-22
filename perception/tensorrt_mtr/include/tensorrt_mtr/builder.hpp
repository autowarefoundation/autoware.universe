// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TENSORRT_MTR__BUILDER_HPP_
#define TENSORRT_MTR__BUILDER_HPP_

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <filesystem>
namespace fs = ::std::filesystem;

#include <tensorrt_common/logger.hpp>

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace trt_mtr
{

template <typename T>
struct TrtDeleter
{
  void operator()(T * obj) const
  {
    if (obj) {
#if NV_TENSORRT_MAJOR >= 8
      delete obj;
#else
      obj->destroy();
#endif
    }
  }
};  // struct TrtDeleter

template <typename T>
using TrtUniquePtr = std::unique_ptr<T, TrtDeleter<T>>;
using BatchConfig = std::array<int32_t, 3>;

struct BuildConfig
{
  // type for calibration
  std::string calib_type_str;

  // DLA core ID that the process uses
  int dla_core_id;

  // flag for partial quantization in first layer
  bool quantize_first_layer;  // For partial quantization

  // flag for partial quantization in last layer
  bool quantize_last_layer;  // For partial quantization

  // flag for per-layer profiler using IProfiler
  bool profile_per_layer;

  // clip value for implicit quantization
  double clip_value;  // For implicit quantization

  // Supported calibration type
  const std::array<std::string, 4> valid_calib_type = {"Entropy", "Legacy", "Percentile", "MinMax"};

  /**
   * @brief Construct a new instance with default configurations.
   *
   */
  BuildConfig()
  : calib_type_str("MinMax"),
    dla_core_id(-1),
    quantize_first_layer(false),
    quantize_last_layer(false),
    profile_per_layer(false),
    clip_value(0.0)
  {
  }

  /**
   * @brief Construct a new instance with custom configurations.
   *
   * @param calib_type_str The name of calibration type which must be selected from [Entropy,
   * MinMax].
   * @param dla_core_id DLA core ID used by the process.
   * @param quantize_first_layer The flag whether to quantize first layer.
   * @param quantize_last_layer The flag whether to quantize last layer.
   * @param profile_per_layer The flag to profile per-layer in IProfiler.
   * @param clip_value The value to be clipped in quantization implicitly.
   */
  explicit BuildConfig(
    const std::string & calib_type_str, const int dla_core_id = -1,
    const bool quantize_first_layer = false, const bool quantize_last_layer = false,
    const bool profile_per_layer = false, const double clip_value = 0.0)
  : calib_type_str(calib_type_str),
    dla_core_id(dla_core_id),
    quantize_first_layer(quantize_first_layer),
    quantize_last_layer(quantize_last_layer),
    profile_per_layer(profile_per_layer),
    clip_value(clip_value)
  {
    if (
      std::find(valid_calib_type.begin(), valid_calib_type.end(), calib_type_str) ==
      valid_calib_type.end()) {
      std::stringstream message;
      message << "Invalid calibration type was specified: " << calib_type_str << std::endl
              << "Valid value is one of: [Entropy, (Legacy | Percentile), MinMax]" << std::endl
              << "Default calibration type will be used: MinMax" << std::endl;
      std::cerr << message.str();
    }
  }
};  // struct BuildConfig

class MTRBuilder
{
public:
  /**
   * @brief Construct a new instance.
   *
   * @param model_path Path to engine or onnx file.
   * @param precision The name of precision type.
   * @param batch_config The configuration of min/opt/max batch.
   * @param max_workspace_size The max workspace size.
   * @param build_config The configuration of build.
   */
  MTRBuilder(
    const std::string & model_path, const std::string & precision,
    const BatchConfig & batch_config = {1, 1, 1}, const size_t max_workspace_size = (1ULL << 30),
    const BuildConfig & build_config = BuildConfig());

  /**
   * @brief Destroy the instance.
   */
  ~MTRBuilder();

  /**
   * @brief Setup engine for inference. After finishing setup successfully, `isInitialized` must
   * return `true`.
   */
  void setup();

  /**
   * @brief Check whether engine was initialized successfully.
   *
   * @return True if plugins were initialized successfully.
   */
  bool isInitialized() const;

  /**
   * @brief A wrapper of `nvinfer1::IExecuteContext::enqueueV2`.
   *
   * @param bindings An array of pointers to input and output buffers for the network.
   * @param stream A cuda stream on which the inference kernels will be enqueued.
   * @param inputConsumed An optional event which will be signaled when the input buffers can be
   * refilled with new data.
   * @return True If the kernels were enqueued successfully.
   */
  bool enqueueV2(void ** bindings, cudaStream_t stream, cudaEvent_t * inputConsumed);

private:
  /**
   * @brief Load engin file.
   *
   * @param filepath Engine file path.
   * @return True if the engine were loaded successfully.
   */
  bool loadEngine(const std::string & filepath);

  /**
   * @brief Build engine from onnx file.
   *
   * @param filepath Onnx file path.
   * @param output_engine_filepath Output engine file path.
   * @return True if the engine were built successfully.
   */
  bool buildEngineFromOnnx(
    const std::string & filepath, const std::string & output_engine_filepath);

  tensorrt_common::Logger logger_;
  TrtUniquePtr<nvinfer1::IRuntime> runtime_;
  TrtUniquePtr<nvinfer1::ICudaEngine> engine_;
  TrtUniquePtr<nvinfer1::IExecutionContext> context_;

  fs::path model_filepath_;
  std::string precision_;
  BatchConfig batch_config_;
  size_t max_workspace_size_;
  std::unique_ptr<const BuildConfig> build_config_;

  bool is_initialized_{false};
};  // class MTRBuilder
}  // namespace trt_mtr
#endif  // TENSORRT_MTR__BUILDER_HPP_
