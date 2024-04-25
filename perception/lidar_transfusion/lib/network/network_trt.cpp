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

#include "lidar_transfusion/network/network_trt.hpp"

#include <NvOnnxParser.h>

#include <fstream>
#include <memory>
#include <string>

namespace lidar_transfusion
{

NetworkTRT::NetworkTRT(const TransfusionConfig & config) : config_(config)
{
}

NetworkTRT::~NetworkTRT()
{
  context.reset();
  runtime_.reset();
  plan_.reset();
  engine.reset();
}

bool NetworkTRT::init(
  const std::string & onnx_path, const std::string & engine_path, const std::string & precision)
{
  runtime_ =
    tensorrt_common::TrtUniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create runtime" << std::endl;
    return false;
  }

  bool success;
  std::ifstream engine_file(engine_path);
  if (engine_file.is_open()) {
    success = loadEngine(engine_path);
  } else {
    auto log_thread = logger_.log_throttle(
      nvinfer1::ILogger::Severity::kINFO,
      "Applying optimizations and building TRT CUDA engine. Please wait a minutes...", 5);
    success = parseONNX(onnx_path, engine_path, precision);
    logger_.stop_throttle(log_thread);
  }
  success &= createContext();

  return success;
}

bool NetworkTRT::setProfile(
  nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
  nvinfer1::IBuilderConfig & config)
{
  auto profile = builder.createOptimizationProfile();

  auto voxels_name = network.getInput(NetworkIO::voxels)->getName();
  auto voxels_min_dims = nvinfer1::Dims3(
    config_.min_voxel_size_, config_.min_point_in_voxel_size_, config_.min_network_feature_size_);
  auto voxels_opt_dims = nvinfer1::Dims3(
    config_.opt_voxel_size_, config_.opt_point_in_voxel_size_, config_.opt_network_feature_size_);
  auto voxels_max_dims = nvinfer1::Dims3(
    config_.max_voxel_size_, config_.max_point_in_voxel_size_, config_.max_network_feature_size_);

  auto num_points_name = network.getInput(NetworkIO::num_points)->getName();
  auto num_points_min_dims = nvinfer1::Dims{1, {static_cast<int32_t>(config_.min_points_size_)}};
  auto num_points_opt_dims = nvinfer1::Dims{1, {static_cast<int32_t>(config_.opt_points_size_)}};
  auto num_points_max_dims = nvinfer1::Dims{1, {static_cast<int32_t>(config_.max_points_size_)}};

  auto coors_name = network.getInput(NetworkIO::coors)->getName();
  auto coors_min_dims = nvinfer1::Dims2(config_.min_coors_size_, config_.min_coors_dim_size_);
  auto coors_opt_dims = nvinfer1::Dims2(config_.opt_coors_size_, config_.opt_coors_dim_size_);
  auto coors_max_dims = nvinfer1::Dims2(config_.max_coors_size_, config_.max_coors_dim_size_);

  profile->setDimensions(voxels_name, nvinfer1::OptProfileSelector::kMIN, voxels_min_dims);
  profile->setDimensions(voxels_name, nvinfer1::OptProfileSelector::kOPT, voxels_opt_dims);
  profile->setDimensions(voxels_name, nvinfer1::OptProfileSelector::kMAX, voxels_max_dims);

  profile->setDimensions(num_points_name, nvinfer1::OptProfileSelector::kMIN, num_points_min_dims);
  profile->setDimensions(num_points_name, nvinfer1::OptProfileSelector::kOPT, num_points_opt_dims);
  profile->setDimensions(num_points_name, nvinfer1::OptProfileSelector::kMAX, num_points_max_dims);

  profile->setDimensions(coors_name, nvinfer1::OptProfileSelector::kMIN, coors_min_dims);
  profile->setDimensions(coors_name, nvinfer1::OptProfileSelector::kOPT, coors_opt_dims);
  profile->setDimensions(coors_name, nvinfer1::OptProfileSelector::kMAX, coors_max_dims);

  config.addOptimizationProfile(profile);
  return true;
}

bool NetworkTRT::createContext()
{
  if (!engine) {
    tensorrt_common::LOG_ERROR(logger_)
      << "Failed to create context: Engine was not created" << std::endl;
    return false;
  }

  context =
    tensorrt_common::TrtUniquePtr<nvinfer1::IExecutionContext>(engine->createExecutionContext());
  if (!context) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create context" << std::endl;
    return false;
  }

  return true;
}

bool NetworkTRT::parseONNX(
  const std::string & onnx_path, const std::string & engine_path, const std::string & precision,
  const size_t workspace_size)
{
  auto builder =
    tensorrt_common::TrtUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
  if (!builder) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create builder" << std::endl;
    return false;
  }

  auto config =
    tensorrt_common::TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create config" << std::endl;
    return false;
  }
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspace_size);
#else
  config->setMaxWorkspaceSize(workspace_size);
#endif
  if (precision == "fp16") {
    if (builder->platformHasFastFp16()) {
      tensorrt_common::LOG_INFO(logger_) << "Using TensorRT FP16 Inference" << std::endl;
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    } else {
      tensorrt_common::LOG_INFO(logger_)
        << "TensorRT FP16 Inference isn't supported in this environment" << std::endl;
    }
  }

  const auto flag =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network =
    tensorrt_common::TrtUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(flag));
  if (!network) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create network" << std::endl;
    return false;
  }

  auto parser = tensorrt_common::TrtUniquePtr<nvonnxparser::IParser>(
    nvonnxparser::createParser(*network, logger_));
  parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR));

  if (!setProfile(*builder, *network, *config)) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to set profile" << std::endl;
    return false;
  }

  plan_ = tensorrt_common::TrtUniquePtr<nvinfer1::IHostMemory>(
    builder->buildSerializedNetwork(*network, *config));
  if (!plan_) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create serialized network" << std::endl;
    return false;
  }
  engine = tensorrt_common::TrtUniquePtr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(plan_->data(), plan_->size()));
  if (!engine) {
    tensorrt_common::LOG_ERROR(logger_) << "Failed to create engine" << std::endl;
    return false;
  }

  return saveEngine(engine_path);
}

bool NetworkTRT::saveEngine(const std::string & engine_path)
{
  tensorrt_common::LOG_INFO(logger_) << "Writing to " << engine_path << std::endl;
  std::ofstream file(engine_path, std::ios::out | std::ios::binary);
  file.write(reinterpret_cast<const char *>(plan_->data()), plan_->size());
  return validateNetworkIO();
}

bool NetworkTRT::loadEngine(const std::string & engine_path)
{
  std::ifstream engine_file(engine_path);
  std::stringstream engine_buffer;
  engine_buffer << engine_file.rdbuf();
  std::string engine_str = engine_buffer.str();
  engine = tensorrt_common::TrtUniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(
    reinterpret_cast<const void *>(engine_str.data()), engine_str.size()));
  tensorrt_common::LOG_INFO(logger_) << "Loaded engine from " << engine_path << std::endl;
  return validateNetworkIO();
}

bool NetworkTRT::validateNetworkIO()
{
  if (engine->getNbIOTensors() != NetworkIO::ENUM_SIZE) {
    tensorrt_common::LOG_ERROR(logger_)
      << "Invalid network IO. Expected size: " << NetworkIO::ENUM_SIZE
      << ". Actual size: " << engine->getNbIOTensors() << "." << std::endl;
    throw std::runtime_error("Failed to initialize TRT network.");
  }
  for (int i = 0; i < NetworkIO::ENUM_SIZE; ++i) {
    tensors_names_.push_back(engine->getIOTensorName(i));
  }
  std::string tensors = std::accumulate(
    tensors_names_.begin(), tensors_names_.end(), std::string(),
    [](const std::string & a, const std::string & b) -> std::string { return a + b + " "; });
  tensorrt_common::LOG_INFO(logger_) << "Network IO: " << tensors << std::endl;
  return true;
}

const char * NetworkTRT::getTensorName(NetworkIO name)
{
  return tensors_names_.at(name);
}

}  // namespace lidar_transfusion
