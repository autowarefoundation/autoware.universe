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

#ifndef TENSORRT_MTR__TRT_MTR_HPP_
#define TENSORRT_MTR__TRT_MTR_HPP_

#include "attention/trt_attn_value_computation.hpp"
#include "attention/trt_attn_weight_computation.hpp"
#include "knn/trt_knn_batch.hpp"
#include "knn/trt_knn_batch_mlogk_kernel.hpp"
#include "tensorrt_mtr/agent.hpp"
#include "tensorrt_mtr/builder.hpp"
#include "tensorrt_mtr/cuda_helper.hpp"
#include "tensorrt_mtr/intention_point.hpp"
#include "tensorrt_mtr/polyline.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace trt_mtr
{
/**
 * @brief A configuration of MTR.
 */
struct MtrConfig
{
  /**
   * @brief Construct a new Mtr Config object
   *
   * @param target_labels An array of target label names.
   * @param num_mode The number of modes.
   * @param num_future The number of future time step length predicted by MTR.
   * @param num_predict_dim The number of state dimensions.
   * @param max_num_polyline The max number of polylines which can be contained in a single input.
   * @param offset_xy The offset value used in input pre-process.
   * @param intention_point_filepath The path to intention points file.
   * @param num_intention_point_cluster The number of clusters for intension points.
   */
  MtrConfig(
    const std::vector<std::string> & target_labels = {"VEHICLE", "PEDESTRIAN", "CYCLIST"},
    const size_t num_mode = 6, const size_t num_future = 80, const size_t num_predict_dim = 7,
    const size_t max_num_polyline = 768, const std::array<float, 2> offset_xy = {30.0f, 0.0f},
    const std::string & intention_point_filepath = "./data/waymo64.csv",
    const size_t num_intention_point_cluster = 64)
  : target_labels(target_labels),
    num_class(target_labels.size()),
    num_mode(num_mode),
    num_future(num_future),
    num_predict_dim(num_predict_dim),
    max_num_polyline(max_num_polyline),
    offset_xy(offset_xy),
    intention_point_filepath(intention_point_filepath),
    num_intention_point_cluster(num_intention_point_cluster)
  {
  }

  std::vector<std::string> target_labels;
  size_t num_class;
  size_t num_mode;
  size_t num_future;
  size_t num_predict_dim;
  size_t max_num_polyline;
  std::array<float, 2> offset_xy;
  std::string intention_point_filepath;
  size_t num_intention_point_cluster;
};

/**
 * @brief A class to inference with MTR.
 */
class TrtMTR
{
public:
  /**
   * @brief Construct a new instance.
   *
   * @param model_path The path to engine or onnx file.
   * @param precision The precision type.
   * @param config The configuration of model.
   * @param batch_config The configuration of batch.
   * @param max_workspace_size The max size of workspace.
   * @param build_config The configuration of build.
   */
  TrtMTR(
    const std::string & model_path, const std::string & precision,
    const MtrConfig & config = MtrConfig(), const BatchConfig & batch_config = {1, 1, 1},
    const size_t max_workspace_size = (1ULL << 30),
    const BuildConfig & build_config = BuildConfig());

  /**
   * @brief Execute inference.
   *
   * @param agent_data The agent data to input.
   * @param polyline_data The polyline data to input.
   * @return True if the inference finishes successfully.
   */
  bool doInference(AgentData & agent_data, PolylineData & polyline_data);

  /**
   * @brief Return the model configuration.
   *
   * @return const MtrConfig& The model configuration which can not be updated.
   */
  const MtrConfig & config() const { return config_; }

private:
  /**
   * @brief Initialize pointers of cuda containers.
   *
   * @param agent_data The input agent data.
   * @param polyline_data The input polyline data.
   */
  void initCudaPtr(AgentData & agent_data, PolylineData & polyline_data);

  /**
   * @brief Execute pre-process.
   *
   * @param agent_data The input agent data.
   * @param polyline_data The input polyline data.
   * @return True if the pre-process finishes successfully.
   */
  bool preProcess(AgentData & agent_data, PolylineData & polyline_data);

  /**
   * @brief Execute post-process.
   *
   * @param agent_data The input agent data.
   * @return True if the post-process finishes successfully.
   */
  bool postProcess(AgentData & agent_data);

  // model parameters
  MtrConfig config_;

  std::unique_ptr<MTRBuilder> builder_;
  cudaStream_t stream_{nullptr};

  IntentionPoint intention_point_;

  // source data
  cuda::unique_ptr<int[]> d_target_index_{nullptr};
  cuda::unique_ptr<int[]> d_label_index_{nullptr};
  cuda::unique_ptr<float[]> d_timestamps_{nullptr};
  cuda::unique_ptr<float[]> d_trajectory_{nullptr};
  cuda::unique_ptr<float[]> d_target_state_{nullptr};
  cuda::unique_ptr<float[]> d_intention_points_{nullptr};
  cuda::unique_ptr<float[]> d_polyline_{nullptr};
  cuda::unique_ptr<int[]> d_topk_index_{nullptr};

  // preprocessed inputs
  cuda::unique_ptr<float[]> d_in_trajectory_{nullptr};
  cuda::unique_ptr<bool[]> d_in_trajectory_mask_{nullptr};
  cuda::unique_ptr<float[]> d_in_last_pos_{nullptr};
  cuda::unique_ptr<float[]> d_in_polyline_{nullptr};
  cuda::unique_ptr<bool[]> d_in_polyline_mask_{nullptr};
  cuda::unique_ptr<float[]> d_in_polyline_center_{nullptr};

  // outputs
  cuda::unique_ptr<float[]> d_out_score_{nullptr};
  cuda::unique_ptr<float[]> d_out_trajectory_{nullptr};

  // debug
  cuda::EventDebugger event_debugger_;
  std::unique_ptr<float[]> h_debug_in_trajectory_{nullptr};
  std::unique_ptr<bool[]> h_debug_in_trajectory_mask_{nullptr};
  std::unique_ptr<float[]> h_debug_in_last_pos_{nullptr};
  std::unique_ptr<float[]> h_debug_in_polyline_{nullptr};
  std::unique_ptr<bool[]> h_debug_in_polyline_mask_{nullptr};
  std::unique_ptr<float[]> h_debug_in_polyline_center_{nullptr};
  std::unique_ptr<float[]> h_debug_out_score_{nullptr};
  std::unique_ptr<float[]> h_debug_out_trajectory_{nullptr};

  /**
   * @brief Display input data after finishing pre-process.
   *
   * @param agent_data The input agent data.
   * @param polyline_data The input polyline data.
   */
  void debugPreprocess(const AgentData & agent_data, const PolylineData & polyline_data);

  /**
   * @brief Display output data after finishing post-process.
   *
   * @param agent_data The input agent data.
   */
  void debugPostprocess(const AgentData & agent_data);
};  // class TrtMTR
}  // namespace trt_mtr
#endif  // TENSORRT_MTR__TRT_MTR_HPP_
