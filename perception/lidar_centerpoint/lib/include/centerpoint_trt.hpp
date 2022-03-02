// Copyright 2021 Tier IV, Inc.
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

#ifndef CENTERPOINT_TRT_HPP_
#define CENTERPOINT_TRT_HPP_

#include <config.hpp>
#include <cuda_utils.hpp>
#include <network_trt.hpp>
#include <postprocess_kernel.hpp>
#include <voxel_generator.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace centerpoint
{
class NetworkParam
{
public:
  NetworkParam(std::string onnx_path, std::string engine_path, std::string trt_precision)
  : onnx_path_(std::move(onnx_path)),
    engine_path_(std::move(engine_path)),
    trt_precision_(std::move(trt_precision))
  {
  }

  std::string onnx_path() const { return onnx_path_; }
  std::string engine_path() const { return engine_path_; }
  std::string trt_precision() const { return trt_precision_; }

private:
  std::string onnx_path_;
  std::string engine_path_;
  std::string trt_precision_;
};

class CenterPointTRT
{
public:
  explicit CenterPointTRT(
    const int num_class, const NetworkParam & encoder_param, const NetworkParam & head_param,
    const DensificationParam & densification_param);

  ~CenterPointTRT();

  bool detect(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer,
    std::vector<Box3D> & det_boxes3d);

private:
  void initPtr();

  bool preprocess(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer);

  void inference();

  void postProcess(std::vector<Box3D> & det_boxes3d);

  std::unique_ptr<VoxelGeneratorTemplate> vg_ptr_{nullptr};
  std::unique_ptr<VoxelEncoderTRT> encoder_trt_ptr_{nullptr};
  std::unique_ptr<HeadTRT> head_trt_ptr_{nullptr};
  std::unique_ptr<PostProcessCUDA> post_proc_ptr{nullptr};
  cudaStream_t stream_{nullptr};

  bool verbose_{false};
  int num_class_{0};
  size_t num_voxels_{0};
  const int size_of_voxels_ =
    Config::max_num_voxels * Config::max_num_points_per_voxel * Config::num_point_features;
  const int size_of_coordinates_ = Config::max_num_voxels * Config::num_point_dims;
  const size_t size_of_num_points_per_voxel_ = Config::max_num_voxels;
  const size_t input_features_size_ =
    Config::max_num_voxels * Config::max_num_points_per_voxel * Config::num_encoder_input_features;
  const size_t pillar_features_size_ = Config::max_num_voxels * Config::num_encoder_output_features;
  const size_t spatial_features_size_ =
    Config::grid_size_x * Config::grid_size_y * Config::num_encoder_output_features;
  cuda::unique_ptr<float[]> voxels_d_{nullptr};
  cuda::unique_ptr<int[]> coordinates_d_{nullptr};
  cuda::unique_ptr<float[]> num_points_per_voxel_d_{nullptr};
  cuda::unique_ptr<float[]> input_features_d_{nullptr};
  cuda::unique_ptr<float[]> pillar_features_d_{nullptr};
  cuda::unique_ptr<float[]> spatial_features_d_{nullptr};
  cuda::unique_ptr<float[]> output_heatmap_d_{nullptr};
  cuda::unique_ptr<float[]> output_offset_d_{nullptr};
  cuda::unique_ptr<float[]> output_z_d_{nullptr};
  cuda::unique_ptr<float[]> output_dim_d_{nullptr};
  cuda::unique_ptr<float[]> output_rot_d_{nullptr};
  cuda::unique_ptr<float[]> output_vel_d_{nullptr};
};

}  // namespace centerpoint

#endif  // CENTERPOINT_TRT_HPP_
