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
/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES.
 * All rights reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lidar_transfusion/cuda_utils.hpp"
#include "lidar_transfusion/preprocess/preprocess_kernel.hpp"

namespace lidar_transfusion
{

PreprocessCuda::PreprocessCuda(const TransfusionConfig & config, cudaStream_t & stream)
: stream_(stream), config_(config)
{
  mask_size_ =
    config_.grid_z_size_ * config_.grid_y_size_ * config_.grid_x_size_ * sizeof(unsigned int);
  voxels_size_ = config_.grid_z_size_ * config_.grid_y_size_ * config_.grid_x_size_ *
                             config_.max_num_points_per_pillar_ * config_.num_point_feature_size_ *
                             sizeof(float);
  mask_ = cuda::make_unique<unsigned int[]>(mask_size_);
  voxels_ = cuda::make_unique<float[]>(voxels_size_);
}

void PreprocessCuda::generateVoxels(
  float * points, unsigned int points_size, unsigned int * pillar_num, float * voxel_features,
  unsigned int * voxel_num, unsigned int * voxel_idxs)
{
  cuda::clear_async(mask_.get(), mask_size_, stream_);
  cuda::clear_async(voxels_.get(), voxels_size_, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  CHECK_CUDA_ERROR(generateVoxels_random_launch(points, points_size, mask_.get(), voxels_.get()));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  CHECK_CUDA_ERROR(generateBaseFeatures_launch(
    mask_.get(), voxels_.get(), pillar_num, voxel_features, voxel_num, voxel_idxs));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

__global__ void generateVoxels_random_kernel(
  float * points, unsigned int points_size, float min_x_range, float max_x_range, float min_y_range,
  float max_y_range, float min_z_range, float max_z_range, float pillar_x_size, float pillar_y_size,
  float pillar_z_size, int grid_y_size, int grid_x_size, int points_per_voxel, unsigned int * mask,
  float * voxels)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= points_size) return;

  float x = points[point_idx * 5];
  float y = points[point_idx * 5 + 1];
  float z = points[point_idx * 5 + 2];
  float w = points[point_idx * 5 + 3];
  float t = points[point_idx * 5 + 4];

  if (
    x < min_x_range || x >= max_x_range || y < min_y_range || y >= max_y_range || z < min_z_range ||
    z >= max_z_range)
    return;

  int voxel_idx = floorf((x - min_x_range) / pillar_x_size);
  int voxel_idy = floorf((y - min_y_range) / pillar_y_size);
  unsigned int voxel_index = voxel_idy * grid_x_size + voxel_idx;

  unsigned int point_id = atomicAdd(&(mask[voxel_index]), 1);

  if (point_id >= points_per_voxel) return;
  float * address = voxels + (voxel_index * points_per_voxel + point_id) * 5;
  atomicExch(address + 0, x);
  atomicExch(address + 1, y);
  atomicExch(address + 2, z);
  atomicExch(address + 3, w);
  atomicExch(address + 4, t);
}

cudaError_t PreprocessCuda::generateVoxels_random_launch(
  float * points, unsigned int points_size, unsigned int * mask, float * voxels)
{
  int threadNum = config_.threads_for_voxel_;
  dim3 blocks((points_size + threadNum - 1) / threadNum);
  dim3 threads(threadNum);
  generateVoxels_random_kernel<<<blocks, threads, 0, stream_>>>(
    points, points_size, config_.min_x_range_, config_.max_x_range_, config_.min_y_range_,
    config_.max_y_range_, config_.min_z_range_, config_.max_z_range_, config_.voxel_x_size_,
    config_.voxel_y_size_, config_.voxel_z_size_, config_.grid_y_size_, config_.grid_x_size_,
    config_.points_per_voxel_, mask, voxels);
  cudaError_t err = cudaGetLastError();
  return err;
}

__global__ void generateBaseFeatures_kernel(
  unsigned int * mask, float * voxels, int grid_y_size, int grid_x_size, float points_per_voxel,
  float max_voxels, unsigned int * pillar_num, float * voxel_features, unsigned int * voxel_num,
  unsigned int * voxel_idxs)
{
  unsigned int voxel_idx = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int voxel_idy = blockIdx.y * blockDim.y + threadIdx.y;

  if (voxel_idx >= grid_x_size || voxel_idy >= grid_y_size) return;

  unsigned int voxel_index = voxel_idy * grid_x_size + voxel_idx;
  unsigned int count = mask[voxel_index];
  if (!(count > 0)) return;
  count = count < points_per_voxel ? count : points_per_voxel;

  unsigned int current_pillarId = 0;
  current_pillarId = atomicAdd(pillar_num, 1);
  if (current_pillarId >= max_voxels) return;

  voxel_num[current_pillarId] = count;

  uint4 idx = {0, 0, voxel_idy, voxel_idx};
  ((uint4 *)voxel_idxs)[current_pillarId] = idx;

  for (int i = 0; i < count; i++) {
    int inIndex = voxel_index * points_per_voxel + i;
    int outIndex = current_pillarId * points_per_voxel + i;
    voxel_features[outIndex * 5] = voxels[inIndex * 5];
    voxel_features[outIndex * 5 + 1] = voxels[inIndex * 5 + 1];
    voxel_features[outIndex * 5 + 2] = voxels[inIndex * 5 + 2];
    voxel_features[outIndex * 5 + 3] = voxels[inIndex * 5 + 3];
    voxel_features[outIndex * 5 + 4] = voxels[inIndex * 5 + 4];
  }

  // clear buffer for next infer
  atomicExch(mask + voxel_index, 0);
}

// create 4 channels
cudaError_t PreprocessCuda::generateBaseFeatures_launch(
  unsigned int * mask, float * voxels, unsigned int * pillar_num, float * voxel_features,
  unsigned int * voxel_num, unsigned int * voxel_idxs)
{
  dim3 threads = {32, 32};
  dim3 blocks = {divup(config_.grid_x_size_, threads.x), divup(config_.grid_y_size_, threads.y)};

  generateBaseFeatures_kernel<<<blocks, threads, 0, stream_>>>(
    mask, voxels, config_.grid_y_size_, config_.grid_x_size_, config_.points_per_voxel_,
    config_.max_voxels_, pillar_num, voxel_features, voxel_num, voxel_idxs);
  cudaError_t err = cudaGetLastError();
  return err;
}

__device__ int decode_field(
  unsigned char * cloud_data, unsigned long point_offset, unsigned int field_offset,
  unsigned char field_size, bool is_bigendian)
{
  int value = 0;
  for (int i = 0; i < field_size; i++) {
    int step = is_bigendian ? field_size - i - 1 : i;
    value |= cloud_data[point_offset + field_offset + i] << (step * 8);
  }
  return value;
}

__device__ float cast_field(int & field_raw, unsigned char & field_type)
{
  switch (field_type) {
    case 1:  // INT8
      return static_cast<float>(reinterpret_cast<char &>(field_raw));
    case 2:  // UINT8
      return static_cast<float>(reinterpret_cast<unsigned char &>(field_raw));
    case 3:  // INT16
      return static_cast<float>(reinterpret_cast<short &>(field_raw));
    case 4:  // UINT16
      return static_cast<float>(reinterpret_cast<unsigned short &>(field_raw));
    case 5:  // INT32
      return static_cast<float>(reinterpret_cast<int &>(field_raw));
    case 6:  // UINT32
      return static_cast<float>(reinterpret_cast<unsigned int &>(field_raw));
    case 7:  // FLOAT32
      return reinterpret_cast<float &>(field_raw);
    case 8:  // FLOAT64
      return static_cast<float>(reinterpret_cast<double &>(field_raw));
    default:
      return 0.0;
  }
}

__global__ void generateVoxelsInput_kernel(
  unsigned char * cloud_data, unsigned int x_offset, unsigned int y_offset, unsigned int z_offset,
  unsigned int intensity_offset, unsigned char x_datatype, unsigned char y_datatype,
  unsigned char z_datatype, unsigned char intensity_datatype, unsigned char x_size,
  unsigned char y_size, unsigned char z_size, unsigned char intensity_size, unsigned int point_step,
  bool is_bigendian, unsigned int cloud_capacity, unsigned int points_agg, unsigned int points_size,
  float time_lag, float * affine_past2current, float * points)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= points_size || points_agg + point_idx >= cloud_capacity) return;

  unsigned long point_offset = point_idx * point_step;

  int x_raw = decode_field(cloud_data, point_offset, x_offset, x_size, is_bigendian);
  int y_raw = decode_field(cloud_data, point_offset, y_offset, y_size, is_bigendian);
  int z_raw = decode_field(cloud_data, point_offset, z_offset, z_size, is_bigendian);
  int intensity_raw =
    decode_field(cloud_data, point_offset, intensity_offset, intensity_size, is_bigendian);

  float x = cast_field(x_raw, x_datatype);
  float y = cast_field(y_raw, y_datatype);
  float z = cast_field(z_raw, z_datatype);
  float intensity = cast_field(intensity_raw, intensity_datatype);

  float x_affined = affine_past2current[0] * x + affine_past2current[1] * y +
                    affine_past2current[2] * z + affine_past2current[3];
  float y_affined = affine_past2current[4] * x + affine_past2current[5] * y +
                    affine_past2current[6] * z + affine_past2current[7];
  float z_affined = affine_past2current[8] * x + affine_past2current[9] * y +
                    affine_past2current[10] * z + affine_past2current[11];

  points[(points_agg + point_idx) * 5] = x_affined;
  points[(points_agg + point_idx) * 5 + 1] = y_affined;
  points[(points_agg + point_idx) * 5 + 2] = z_affined;
  points[(points_agg + point_idx) * 5 + 3] = intensity;
  points[(points_agg + point_idx) * 5 + 4] = time_lag;
}

// extract data from cloud byte array
cudaError_t PreprocessCuda::generateVoxelsInput_launch(
  uint8_t * cloud_data, CloudInfo & cloud_info, unsigned int points_agg, unsigned int points_size,
  float time_lag, float * affine_past2current, float * points)
{
  dim3 threads = {1024};
  dim3 blocks = {divup(points_size, threads.x * threads.y)};
  generateVoxelsInput_kernel<<<blocks, threads, 0, stream_>>>(
    cloud_data, cloud_info.x_offset, cloud_info.y_offset, cloud_info.z_offset,
    cloud_info.intensity_offset, cloud_info.x_datatype, cloud_info.y_datatype,
    cloud_info.z_datatype, cloud_info.intensity_datatype, cloud_info.x_size, cloud_info.y_size,
    cloud_info.z_size, cloud_info.intensity_size, cloud_info.point_step, cloud_info.is_bigendian,
    config_.cloud_capacity_, points_agg, points_size, time_lag, affine_past2current, points);
  cudaError_t err = cudaGetLastError();
  return err;
}

}  // namespace lidar_transfusion
