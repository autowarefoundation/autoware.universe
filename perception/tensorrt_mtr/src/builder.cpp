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

#include "tensorrt_mtr/builder.hpp"

#include <fstream>
#include <sstream>

namespace trt_mtr
{
MTRBuilder::MTRBuilder(
  const std::string & model_filepath, const std::string & precision,
  const BatchConfig & batch_config, const size_t max_workspace_size,
  const BuildConfig & build_config)
: model_filepath_(model_filepath),
  precision_(precision),
  batch_config_(batch_config),
  max_workspace_size_(max_workspace_size)
{
  build_config_ = std::make_unique<const BuildConfig>(build_config);
  runtime_ = TrtUniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
}

MTRBuilder::~MTRBuilder()
{
}

void MTRBuilder::setup()
{
  if (!fs::exists(model_filepath_)) {
    is_initialized_ = false;
    return;
  }
  std::string engine_path = model_filepath_;
  if (model_filepath_.extension() == ".engine") {
    std::cout << "Loading... " << model_filepath_ << std::endl;
    loadEngine(model_filepath_);
  } else if (model_filepath_.extension() == ".onnx") {
    fs::path cache_engine_path{model_filepath_};
    std::string ext;
    std::string calib_name = "";
    if (precision_ == "int8") {
      if (build_config_->calib_type_str == "Entropy") {
        calib_name = "EntropyV2-";
      } else if (
        build_config_->calib_type_str == "Legacy" ||
        build_config_->calib_type_str == "Percentile") {
        calib_name = "Legacy-";
      } else {
        calib_name = "MinMax-";
      }
    }
    if (build_config_->dla_core_id != -1) {
      ext = "DLA" + std::to_string(build_config_->dla_core_id) + "-" + calib_name + precision_;
      if (build_config_->quantize_first_layer) {
        ext += "-firstFP16";
      }
      if (build_config_->quantize_last_layer) {
        ext += "-lastFP16";
      }
      ext += "-batch" + std::to_string(batch_config_[0]) + ".engine";
    } else {
      ext = calib_name + precision_;
      if (build_config_->quantize_first_layer) {
        ext += "-firstFP16";
      }
      if (build_config_->quantize_last_layer) {
        ext += "-lastFP16";
      }
      ext += "-batch" + std::to_string(batch_config_[0]) + ".engine";
    }
    cache_engine_path.replace_extension(ext);
    if (fs::exists(cache_engine_path)) {
      std::cout << "Loading cached engine... " << cache_engine_path << std::endl;
      if (!loadEngine(cache_engine_path)) {
        std::cerr << "Fail to load engine" << std::endl;
        is_initialized_ = false;
        return;
      }
    } else {
      std::cout << "Building... " << cache_engine_path << std::endl;
      logger_.log(nvinfer1::ILogger::Severity::kINFO, "start build engine");
      if (!buildEngineFromOnnx(model_filepath_, cache_engine_path)) {
        std::cerr << "Fail to build engine from onnx" << std::endl;
        is_initialized_ = false;
        return;
      }
      logger_.log(nvinfer1::ILogger::Severity::kINFO, "End build engine");
    }
    engine_path = cache_engine_path;
  } else {
    is_initialized_ = false;
    return;
  }

  context_ = TrtUniquePtr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (!context_) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create context");
    is_initialized_ = false;
    return;
  }

  is_initialized_ = true;
}

bool MTRBuilder::loadEngine(const std::string & filepath)
{
  try {
    std::ifstream engine_file(filepath);
    std::stringstream buffer;
    buffer << engine_file.rdbuf();
    std::string engine_str = buffer.str();
    engine_ = TrtUniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(
      reinterpret_cast<const void *>(engine_str.data()), engine_str.size()));
    return true;
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

bool MTRBuilder::buildEngineFromOnnx(
  const std::string & filepath, const std::string & output_engine_filepath)
{
  auto builder = TrtUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(logger_));
  if (!builder) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create builder");
    return false;
  }

  const auto explicit_batch =
    1U << static_cast<int32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);

  auto network =
    TrtUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicit_batch));
  if (!network) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create network");
    return false;
  }

  auto config = TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create builder config");
    return false;
  }

  int num_available_dla = builder->getNbDLACores();
  if (build_config_->dla_core_id != -1) {
    if (num_available_dla > 0) {
      std::cout << "###" << num_available_dla << " DLAs are supported! ###" << std::endl;
    } else {
      std::cout << "###Warning : "
                << "No DLA is supported! ###" << std::endl;
    }
    config->setDefaultDeviceType(nvinfer1::DeviceType::kDLA);
    config->setDLACore(build_config_->dla_core_id);
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8200
    config->setFlag(nvinfer1::BuilderFlag::kPREFER_PRECISION_CONSTRAINTS);
#else
    config->setFlag(nvinfer1::BuilderFlag::kSTRICT_TYPES);
#endif
    config->setFlag(nvinfer1::BuilderFlag::kGPU_FALLBACK);
  }
  if (precision_ == "fp16" || precision_ == "int8") {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, max_workspace_size_);
#else
  config->setMaxWorkspaceSize(max_workspace_size_);
#endif

  auto parser = TrtUniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger_));
  if (!parser->parseFromFile(
        filepath.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR))) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to parse onnx file");
    return false;
  }

  const auto input0 = network->getInput(0);
  const auto input0_dims = input0->getDimensions();
  const auto num_targets = input0_dims.d[0];
  const auto num_agents = input0_dims.d[1];
  const auto num_past_frames = input0_dims.d[2];
  const auto num_agent_dims = input0_dims.d[3];

  const auto input2 = network->getInput(2);
  const auto input2_dims = input2->getDimensions();
  const auto num_polylines = input2_dims.d[1];
  const auto num_points = input2_dims.d[2];
  const auto num_polyline_dims = input2_dims.d[3];

  if (num_targets > 1) {
    batch_config_[0] = num_targets;
  }

  if (build_config_->profile_per_layer) {
    auto profile = builder->createOptimizationProfile();
    // trajectory
    const auto input0_name = input0->getName();
    profile->setDimensions(
      input0_name, nvinfer1::OptProfileSelector::kMIN,
      nvinfer1::Dims4{batch_config_.at(0), num_agents, num_past_frames, num_agent_dims});
    profile->setDimensions(
      input0_name, nvinfer1::OptProfileSelector::kOPT,
      nvinfer1::Dims4{batch_config_.at(1), num_agents, num_past_frames, num_agent_dims});
    profile->setDimensions(
      input0_name, nvinfer1::OptProfileSelector::kMAX,
      nvinfer1::Dims4{batch_config_.at(2), num_agents, num_past_frames, num_agent_dims});
    // trajectory mask
    const auto input1_name = network->getInput(1)->getName();
    profile->setDimensions(
      input1_name, nvinfer1::OptProfileSelector::kMIN,
      nvinfer1::Dims3{batch_config_.at(0), num_agents, num_past_frames});
    profile->setDimensions(
      input1_name, nvinfer1::OptProfileSelector::kOPT,
      nvinfer1::Dims3{batch_config_.at(1), num_agents, num_past_frames});
    profile->setDimensions(
      input1_name, nvinfer1::OptProfileSelector::kMAX,
      nvinfer1::Dims3{batch_config_.at(2), num_agents, num_past_frames});
    // polyline
    const auto input2_name = input2->getName();
    profile->setDimensions(
      input2_name, nvinfer1::OptProfileSelector::kMIN,
      nvinfer1::Dims4{batch_config_.at(0), num_polylines, num_points, num_polyline_dims});
    profile->setDimensions(
      input2_name, nvinfer1::OptProfileSelector::kOPT,
      nvinfer1::Dims4{batch_config_.at(1), num_polylines, num_points, num_polyline_dims});
    profile->setDimensions(
      input2_name, nvinfer1::OptProfileSelector::kMAX,
      nvinfer1::Dims4{batch_config_.at(2), num_polylines, num_points, num_polyline_dims});
    // polyline mask
    const auto input3_name = network->getInput(3)->getName();
    profile->setDimensions(
      input3_name, nvinfer1::OptProfileSelector::kMIN,
      nvinfer1::Dims3{batch_config_.at(0), num_polylines, num_points});
    profile->setDimensions(
      input3_name, nvinfer1::OptProfileSelector::kOPT,
      nvinfer1::Dims3{batch_config_.at(1), num_polylines, num_points});
    profile->setDimensions(
      input3_name, nvinfer1::OptProfileSelector::kMAX,
      nvinfer1::Dims3{batch_config_.at(2), num_polylines, num_points});
    // polyline center
    const auto input4_name = network->getInput(4)->getName();
    profile->setDimensions(
      input4_name, nvinfer1::OptProfileSelector::kMIN,
      nvinfer1::Dims3{batch_config_.at(0), num_polylines, 3});
    profile->setDimensions(
      input4_name, nvinfer1::OptProfileSelector::kOPT,
      nvinfer1::Dims3{batch_config_.at(1), num_polylines, 3});
    profile->setDimensions(
      input4_name, nvinfer1::OptProfileSelector::kMAX,
      nvinfer1::Dims3{batch_config_.at(2), num_polylines, 3});
    // last pos
    const auto input5_name = network->getInput(5)->getName();
    profile->setDimensions(
      input5_name, nvinfer1::OptProfileSelector::kMIN,
      nvinfer1::Dims3{batch_config_.at(0), num_agents, 3});
    profile->setDimensions(
      input5_name, nvinfer1::OptProfileSelector::kOPT,
      nvinfer1::Dims3{batch_config_.at(1), num_agents, 3});
    profile->setDimensions(
      input5_name, nvinfer1::OptProfileSelector::kMAX,
      nvinfer1::Dims3{batch_config_.at(2), num_agents, 3});
    // track index & label index is skipped because of 1D
    config->addOptimizationProfile(profile);
  }

  if (build_config_->profile_per_layer) {
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8200
    config->setProfilingVerbosity(nvinfer1::ProfilingVerbosity::kDETAILED);
#else
    config->setProfilingVerbosity(nvinfer1::ProfilingVerbosity::kVERBOSE);
#endif
  }

#if TENSORRT_VERSION_MAJOR >= 8
  auto plan =
    TrtUniquePtr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
  if (!plan) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create host memory");
    return false;
  }
  engine_ = TrtUniquePtr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(plan->data(), plan->size()));
#else
  engine_ = TrtUniquePtr<nvinfer1::ICudaEngine>(builder->buildEngineWithConfig(*network, *config));
#endif

  if (!engine_) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "Fail to create engine");
    return false;
  }

// save engine
#if TENSORRT_VERSION_MAJOR < 8
  auto data = TrtUniquePtr<nvinfer1::IHostMemory>(engine_->serialize());
#endif
  std::ofstream file;
  file.open(output_engine_filepath, std::ios::binary | std::ios::out);
  if (!file.is_open()) {
    return false;
  }
#if TENSORRT_VERSION_MAJOR < 8
  file.write(reinterpret_cast<const char *>(data->data()), data->size());
#else
  file.write(reinterpret_cast<const char *>(plan->data()), plan->size());
#endif

  file.close();

  return true;
}

bool MTRBuilder::isInitialized() const
{
  return is_initialized_;
}

bool MTRBuilder::enqueueV2(void ** bindings, cudaStream_t stream, cudaEvent_t * inputConsumed)
{
  return context_->enqueueV2(bindings, stream, inputConsumed);
}

}  // namespace trt_mtr
