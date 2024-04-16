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

#include "tensorrt_mtr/trt_mtr.hpp"

#include "postprocess/postprocess_kernel.cuh"
#include "preprocess/agent_preprocess_kernel.cuh"
#include "preprocess/polyline_preprocess_kernel.cuh"

namespace trt_mtr
{
TrtMTR::TrtMTR(
  const std::string & model_path, const std::string & precision, const MTRConfig & config,
  const BatchConfig & batch_config, const size_t max_workspace_size,
  const BuildConfig & build_config)
: config_(config),
  intention_point_(config_.intention_point_filepath, config_.num_intention_point_cluster)
{
  builder_ = std::make_unique<MTRBuilder>(
    model_path, precision, batch_config, max_workspace_size, build_config);
  builder_->setup();

  if (!builder_->isInitialized()) {
    return;
  }

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

bool TrtMTR::doInference(
  AgentData & agent_data, PolylineData & polyline_data,
  std::vector<PredictedTrajectory> & trajectories)
{
  initCudaPtr(agent_data, polyline_data);

  if (!preProcess(agent_data, polyline_data)) {
    std::cerr << "Fail to preprocess" << std::endl;
    return false;
  }

  std::vector<void *> buffer = {d_in_trajectory_.get(),      d_in_trajectory_mask_.get(),
                                d_in_polyline_.get(),        d_in_polyline_mask_.get(),
                                d_in_polyline_center_.get(), d_in_last_pos_.get(),
                                d_target_index_.get(),       d_intention_points_.get(),
                                d_out_score_.get(),          d_out_trajectory_.get()};

  if (!builder_->enqueueV2(buffer.data(), stream_, nullptr)) {
    std::cerr << "Fail to do inference" << std::endl;
    return false;
  }

  if (!postProcess(agent_data, trajectories)) {
    std::cerr << "Fail to preprocess" << std::endl;
    return false;
  }

  return true;
}

void TrtMTR::initCudaPtr(AgentData & agent_data, PolylineData & polyline_data)
{
  // source data
  d_target_index_ = cuda::make_unique<int[]>(agent_data.num_target());
  d_label_index_ = cuda::make_unique<int[]>(agent_data.num_agent());
  d_timestamps_ = cuda::make_unique<float[]>(agent_data.time_length());
  d_trajectory_ = cuda::make_unique<float[]>(agent_data.size());
  d_target_state_ = cuda::make_unique<float[]>(agent_data.num_target() * agent_data.state_dim());
  d_intention_points_ =
    cuda::make_unique<float[]>(agent_data.num_target() * intention_point_.size());
  d_polyline_ = cuda::make_unique<float[]>(polyline_data.size());
  d_topk_index_ = cuda::make_unique<int[]>(config_.max_num_polyline);

  // preprocessed input
  // TODO(ktro2828): define this in global
  d_in_trajectory_ = cuda::make_unique<float[]>(
    agent_data.num_target() * agent_data.num_agent() * agent_data.time_length() *
    agent_data.input_dim());
  d_in_trajectory_mask_ = cuda::make_unique<bool[]>(
    agent_data.num_target() * agent_data.num_agent() * agent_data.time_length());
  d_in_last_pos_ = cuda::make_unique<float[]>(agent_data.num_target() * agent_data.num_agent() * 3);
  d_in_polyline_ = cuda::make_unique<float[]>(
    agent_data.num_target() * config_.max_num_polyline * polyline_data.num_point() *
    (polyline_data.state_dim() + 2));
  d_in_polyline_mask_ = cuda::make_unique<bool[]>(
    agent_data.num_target() * config_.max_num_polyline * polyline_data.num_point());
  d_in_polyline_center_ =
    cuda::make_unique<float[]>(agent_data.num_target() * config_.max_num_polyline * 3);

  // outputs
  d_out_score_ = cuda::make_unique<float[]>(agent_data.num_target() * config_.num_mode);
  d_out_trajectory_ = cuda::make_unique<float[]>(
    agent_data.num_target() * config_.num_mode * config_.num_future * PredictedStateDim);
}

bool TrtMTR::preProcess(AgentData & agent_data, PolylineData & polyline_data)
{
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_target_index_.get(), agent_data.target_index().data(), sizeof(int) * agent_data.num_target(),
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_label_index_.get(), agent_data.label_index().data(), sizeof(int) * agent_data.num_agent(),
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_timestamps_.get(), agent_data.timestamps().data(), sizeof(float) * agent_data.time_length(),
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_trajectory_.get(), agent_data.data_ptr(),
    sizeof(float) * agent_data.num_agent() * agent_data.time_length() * agent_data.state_dim(),
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_target_state_.get(), agent_data.target_data_ptr(),
    sizeof(float) * agent_data.num_target() * agent_data.state_dim(), cudaMemcpyHostToDevice,
    stream_));

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_polyline_.get(), polyline_data.data_ptr(),
    sizeof(float) * polyline_data.num_polyline() * polyline_data.num_point() *
      polyline_data.state_dim(),
    cudaMemcpyHostToDevice, stream_));

  const auto target_label_names = getLabelNames(agent_data.target_label_index());
  const auto intention_points = intention_point_.get_points(target_label_names);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_intention_points_.get(), intention_points.data(),
    sizeof(float) * agent_data.num_target() * intention_point_.size(), cudaMemcpyHostToDevice,
    stream_));

  // Preprocess
  CHECK_CUDA_ERROR(agentPreprocessLauncher(
    agent_data.num_target(), agent_data.num_agent(), agent_data.time_length(),
    agent_data.state_dim(), agent_data.num_class(), agent_data.sdc_index(), d_target_index_.get(),
    d_label_index_.get(), d_timestamps_.get(), d_trajectory_.get(), d_in_trajectory_.get(),
    d_in_trajectory_mask_.get(), d_in_last_pos_.get(), stream_));

  // TODO(ktro2828)
  if (config_.max_num_polyline < polyline_data.num_polyline()) {
    CHECK_CUDA_ERROR(polylinePreprocessWithTopkLauncher(
      polyline_data.num_polyline(), config_.max_num_polyline, polyline_data.num_point(),
      polyline_data.state_dim(), d_polyline_.get(), agent_data.num_target(), agent_data.state_dim(),
      d_target_state_.get(), config_.offset_xy[0], config_.offset_xy[1], d_topk_index_.get(),
      d_in_polyline_.get(), d_in_polyline_mask_.get(), d_in_polyline_center_.get(), stream_));
  } else {
    assert(config_.max_num_polyline == polyline_data.num_polyline());
    CHECK_CUDA_ERROR(polylinePreprocessLauncher(
      polyline_data.num_polyline(), polyline_data.num_point(), polyline_data.state_dim(),
      d_polyline_.get(), agent_data.num_target(), agent_data.state_dim(), d_target_state_.get(),
      d_in_polyline_.get(), d_in_polyline_mask_.get(), d_in_polyline_center_.get(), stream_));
  }
  return true;
}

bool TrtMTR::postProcess(AgentData & agent_data, std::vector<PredictedTrajectory> & trajectories)
{
  // Postprocess
  CHECK_CUDA_ERROR(postprocessLauncher(
    agent_data.num_target(), config_.num_mode, config_.num_future, agent_data.state_dim(),
    d_target_state_.get(), PredictedStateDim, d_out_score_.get(), d_out_trajectory_.get(),
    stream_));

  trajectories.reserve(agent_data.num_target());
  for (size_t b = 0; b < agent_data.num_target(); ++b) {
    const auto score_ptr = d_out_score_.get() + b * config_.num_mode;
    const auto trajectory_ptr =
      d_out_trajectory_.get() + b * config_.num_mode * config_.num_future * PredictedStateDim;
    trajectories.emplace_back(score_ptr, trajectory_ptr, config_.num_mode, config_.num_future);
  }
  return true;
}
}  // namespace trt_mtr
