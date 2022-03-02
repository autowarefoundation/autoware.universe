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

#include <centerpoint_trt.hpp>
#include <postprocess_kernel.hpp>
#include <preprocess_kernel.hpp>
#include <scatter_kernel.hpp>
#include <tier4_autoware_utils/math/constants.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace centerpoint
{
CenterPointTRT::CenterPointTRT(
  const int num_class, const NetworkParam & encoder_param, const NetworkParam & head_param,
  const DensificationParam & densification_param)
: num_class_(num_class)
{
  vg_ptr_ = std::make_unique<VoxelGenerator>(densification_param);
  post_proc_ptr = std::make_unique<PostProcessCUDA>(num_class);

  // encoder
  encoder_trt_ptr_ = std::make_unique<VoxelEncoderTRT>(verbose_);
  encoder_trt_ptr_->init(
    encoder_param.onnx_path(), encoder_param.engine_path(), encoder_param.trt_precision());
  encoder_trt_ptr_->context_->setBindingDimensions(
    0, nvinfer1::Dims3(
         Config::max_num_voxels, Config::max_num_points_per_voxel,
         Config::num_encoder_input_features));

  // head
  head_trt_ptr_ = std::make_unique<HeadTRT>(num_class, verbose_);
  head_trt_ptr_->init(head_param.onnx_path(), head_param.engine_path(), head_param.trt_precision());
  head_trt_ptr_->context_->setBindingDimensions(
    0, nvinfer1::Dims4(
         1, Config::num_encoder_output_features, Config::grid_size_y, Config::grid_size_x));

  initPtr();

  cudaStreamCreate(&stream_);
}

CenterPointTRT::~CenterPointTRT()
{
  if (stream_) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
  }
}

void CenterPointTRT::initPtr()
{
  voxels_d_ = cuda::make_unique<float[]>(size_of_voxels_);
  coordinates_d_ = cuda::make_unique<int[]>(size_of_coordinates_);
  num_points_per_voxel_d_ = cuda::make_unique<float[]>(size_of_num_points_per_voxel_);
  input_features_d_ = cuda::make_unique<float[]>(input_features_size_);
  pillar_features_d_ = cuda::make_unique<float[]>(pillar_features_size_);
  spatial_features_d_ = cuda::make_unique<float[]>(spatial_features_size_);

  const int downsample_grid_x =
    static_cast<int>(static_cast<float>(Config::grid_size_x) / Config::downsample_factor);
  const int downsample_grid_y =
    static_cast<int>(static_cast<float>(Config::grid_size_y) / Config::downsample_factor);
  const int grid_xy = downsample_grid_x * downsample_grid_y;
  output_heatmap_d_ = cuda::make_unique<float[]>(grid_xy * num_class_);
  output_offset_d_ = cuda::make_unique<float[]>(grid_xy * Config::num_output_offset_features);
  output_z_d_ = cuda::make_unique<float[]>(grid_xy * Config::num_output_z_features);
  output_dim_d_ = cuda::make_unique<float[]>(grid_xy * Config::num_output_dim_features);
  output_rot_d_ = cuda::make_unique<float[]>(grid_xy * Config::num_output_rot_features);
  output_vel_d_ = cuda::make_unique<float[]>(grid_xy * Config::num_output_vel_features);
}

bool CenterPointTRT::detect(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer,
  std::vector<Box3D> & det_boxes3d)
{
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(input_features_d_.get(), 0, input_features_size_ * sizeof(float), stream_));
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(spatial_features_d_.get(), 0, spatial_features_size_ * sizeof(float), stream_));

  if (!preprocess(input_pointcloud_msg, tf_buffer)) {
    std::cout << "fail preprocess" << std::endl;
    return false;
  }

  inference();

  postProcess(det_boxes3d);

  return true;
}

bool CenterPointTRT::preprocess(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer)
{
  std::vector<float> voxels(size_of_voxels_, 0);
  std::vector<int> coordinates(size_of_coordinates_, -1);
  std::vector<float> num_points_per_voxel(size_of_num_points_per_voxel_, 0);

  vg_ptr_->pd_ptr_->enqueuePointCloud(input_pointcloud_msg, tf_buffer);
  num_voxels_ = vg_ptr_->pointsToVoxels(voxels, coordinates, num_points_per_voxel);
  if (num_voxels_ == 0) {
    return false;
  }

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    voxels_d_.get(), voxels.data(), voxels.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    coordinates_d_.get(), coordinates.data(), coordinates.size() * sizeof(int),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    num_points_per_voxel_d_.get(), num_points_per_voxel.data(),
    num_points_per_voxel.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  CHECK_CUDA_ERROR(generateFeatures_launch(
    voxels_d_.get(), num_points_per_voxel_d_.get(), coordinates_d_.get(), num_voxels_,
    Config::max_num_voxels, Config::voxel_size_x, Config::voxel_size_y, Config::voxel_size_z,
    Config::range_min_x, Config::range_min_y, Config::range_min_z, input_features_d_.get(),
    stream_));

  return true;
}

void CenterPointTRT::inference()
{
  if (!encoder_trt_ptr_->context_ || !head_trt_ptr_->context_) {
    throw std::runtime_error("Fail to create context");
  }

  std::vector<void *> encoder_buffers{input_features_d_.get(), pillar_features_d_.get()};
  encoder_trt_ptr_->context_->enqueueV2(encoder_buffers.data(), stream_, nullptr);

  CHECK_CUDA_ERROR(scatterFeatures_launch(
    pillar_features_d_.get(), coordinates_d_.get(), num_voxels_, Config::max_num_voxels,
    Config::num_encoder_output_features, Config::grid_size_x, Config::grid_size_y,
    spatial_features_d_.get(), stream_));

  std::vector<void *> head_buffers = {
    spatial_features_d_.get(), output_heatmap_d_.get(), output_offset_d_.get(), output_z_d_.get(),
    output_dim_d_.get(),       output_rot_d_.get(),     output_vel_d_.get()};
  head_trt_ptr_->context_->enqueueV2(head_buffers.data(), stream_, nullptr);
}

void CenterPointTRT::postProcess(std::vector<Box3D> & det_boxes3d)
{
  CHECK_CUDA_ERROR(post_proc_ptr->generateDetectedBoxes3D_launch(
    output_heatmap_d_.get(), output_offset_d_.get(), output_z_d_.get(), output_dim_d_.get(),
    output_rot_d_.get(), output_vel_d_.get(), det_boxes3d, stream_));
}

}  // namespace centerpoint
