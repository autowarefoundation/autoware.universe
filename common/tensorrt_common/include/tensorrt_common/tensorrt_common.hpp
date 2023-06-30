// Copyright 2023 TIER IV, Inc.
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

#ifndef TENSORRT_COMMON__TENSORRT_COMMON_HPP_
#define TENSORRT_COMMON__TENSORRT_COMMON_HPP_

#include <rclcpp/rclcpp.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>

#if (defined(_MSC_VER) or (defined(__GNUC__) and (7 <= __GNUC_MAJOR__)))
#include <filesystem>
namespace fs = ::std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = ::std::experimental::filesystem;
#endif

#include <tensorrt_common/logger.hpp>
#include <tensorrt_common/simple_profiler.hpp>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tensorrt_common
{
/**
 * @struct BuildConfig
 * @brief Configuration to provide fine control regarding TensorRT builder
 */
struct BuildConfig
{
  nvinfer1::CalibrationAlgoType calibType;
  int dla;
  bool first;  // For partial quantization
  bool last;   // For partial quantization
  bool prof;
  double clip;  // For implicit quantization
};

nvinfer1::Dims get_input_dims(const std::string & onnx_file_path);
template <typename T>
struct InferDeleter  // NOLINT
{
  void operator()(T * obj) const
  {
    if (obj) {
#if TENSORRT_VERSION_MAJOR >= 8
      delete obj;
#else
      obj->destroy();
#endif
    }
  }
};

template <typename T>
using TrtUniquePtr = std::unique_ptr<T, InferDeleter<T>>;

using BatchConfig = std::array<int32_t, 3>;

/**
 * @class TrtCommon
 * @brief TensorRT common library
 */
class TrtCommon  // NOLINT
{
public:
  /**
   * @brief Construct TrtCommon.
   * @param[in] mode_path ONNX model_path
   * @param[in] precision precision for inference
   * @param[in] calibrator pointer for any type of INT8 calibrator
   * @param[in] batch_config configuration for batched execution
   * @param[in] max_workspace_size maximum workspace for building TensorRT engine
   * @param[in] buildConfig configuration including precision, calibration method, dla, remaining
   * fp16 for first layer,  remaining fp16 for last layer and profiler for builder
   * @param[in] plugin_paths path for custom plugin
   */
  TrtCommon(
    const std::string & model_path, const std::string & precision,
    std::unique_ptr<nvinfer1::IInt8Calibrator> calibrator,
    const BatchConfig & batch_config = {1, 1, 1}, const size_t max_workspace_size = (16 << 20),
    const BuildConfig buildConfig =
      {nvinfer1::CalibrationAlgoType::kMINMAX_CALIBRATION, -1, false, false},
    const std::vector<std::string> & plugin_paths = {});

  /**
   * @brief Deconstruct TrtCommon
   */
  ~TrtCommon();

  /**
   * @brief Load TensorRT engine
   * @param[in] engine_file_path path for a engine file
   * @return flg for whether loading are succedded or failed
   */
  bool loadEngine(const std::string & engine_file_path);

  /**
   * @brief build TensorRT engine from ONNX
   * @param[in] onnx_file_path path for a onnx file
   * @param[in] output_engine_file_path path for a engine file
   */
  bool buildEngineFromOnnx(
    const std::string & onnx_file_path, const std::string & output_engine_file_path);

  /**
   * @brief setup for TensorRT execution including building and loading engine
   */
  void setup();

  bool isInitialized();

  nvinfer1::Dims getBindingDimensions(const int32_t index) const;
  int32_t getNbBindings();
  bool setBindingDimensions(const int32_t index, const nvinfer1::Dims & dimensions) const;
  bool enqueueV2(void ** bindings, cudaStream_t stream, cudaEvent_t * input_consumed);

/**
 * @brief get per-layer information for trt-engine-profiler
 */
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8200
  std::string getLayerInformation(nvinfer1::LayerInformationFormat format);
#endif

private:
  Logger logger_;
  fs::path model_file_path_;
  TrtUniquePtr<nvinfer1::IRuntime> runtime_;
  TrtUniquePtr<nvinfer1::ICudaEngine> engine_;
  TrtUniquePtr<nvinfer1::IExecutionContext> context_;
  std::unique_ptr<nvinfer1::IInt8Calibrator> calibrator_;
  nvinfer1::Dims input_dims_;
  nvinfer1::Dims output_dims_;
  std::string precision_;
  BatchConfig batch_config_;
  size_t max_workspace_size_;
  bool is_initialized_{false};
  // profiler for per-layer
  SimpleProfiler m_model_profiler;
  // profiler for whole model
  SimpleProfiler m_host_profiler;
  // type for calibration
  nvinfer1::CalibrationAlgoType m_calibType;
  // flg for dla
  int m_dla;
  // flg for partial quanitzation in first layer
  bool m_first;
  // flg for partial quanitzation in last layer
  bool m_last;
  // clip value for implicit quantization
  double m_clip;
  // flg for per-layer profiler using IProfiler
  bool m_prof;
};

}  // namespace tensorrt_common

#endif  // TENSORRT_COMMON__TENSORRT_COMMON_HPP_
