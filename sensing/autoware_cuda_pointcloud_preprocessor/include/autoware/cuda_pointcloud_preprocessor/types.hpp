// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__TYPES_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__TYPES_HPP_

#include <cuda_runtime.h>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace autoware::cuda_pointcloud_preprocessor
{

struct TwistStruct2D
{
  float cum_x;
  float cum_y;
  float cum_theta;
  float cum_cos_theta;
  float cum_sin_theta;
  std::uint32_t last_stamp_nsec;  // relative to the start of the pointcloud
  std::uint32_t stamp_nsec;       // relative to the start of the pointcloud
  float v_x;
  float v_theta;
};

struct TwistStruct3D
{
  float cum_transform_buffer[16];
  std::uint32_t last_stamp_nsec;  // relative to the start of the pointcloud
  std::uint32_t stamp_nsec;       // relative to the start of the pointcloud
  float v[3];
  float w[3];
};

struct TransformStruct
{
  float x;
  float y;
  float z;
  float m11;
  float m12;
  float m13;
  float m21;
  float m22;
  float m23;
  float m31;
  float m32;
  float m33;
};

struct CropBoxParameters
{
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_z;
  float max_z;
};

struct RingOutlierFilterParameters
{
  float distance_ratio{std::nanf("")};
  float object_length_threshold{std::nanf("")};
  std::size_t num_points_threshold{0};
};

template <typename T>
class MemoryPoolAllocator
{
public:
  using value_type = T;
  MemoryPoolAllocator(cudaMemPool_t pool) : m_pool(pool) {}

  T * allocate(std::size_t n)
  {
    void * ptr = nullptr;
    cudaMallocFromPoolAsync(&ptr, n * sizeof(T), m_pool, cudaStreamDefault);
    return static_cast<T *>(ptr);
  }

  void deallocate(T * ptr, std::size_t) { cudaFreeAsync(ptr, cudaStreamDefault); }

protected:
  cudaMemPool_t m_pool;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__TYPES_HPP_
