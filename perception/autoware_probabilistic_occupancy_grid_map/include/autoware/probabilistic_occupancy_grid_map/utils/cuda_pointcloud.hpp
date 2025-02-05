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

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__UTILS__CUDA_POINTCLOUD_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__UTILS__CUDA_POINTCLOUD_HPP_

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cuda_runtime.h>

class CudaPointCloud2 : public sensor_msgs::msg::PointCloud2
{
public:
  void fromROSMsgAsync(const sensor_msgs::msg::PointCloud2 & msg)
  {
    // cSpell: ignore knzo25
    // NOTE(knzo25): replace this with the cuda blackboard later
    header = msg.header;
    fields = msg.fields;
    height = msg.height;
    width = msg.width;
    is_bigendian = msg.is_bigendian;
    point_step = msg.point_step;
    row_step = msg.row_step;
    is_dense = msg.is_dense;

    if (!data || capacity_ < msg.data.size()) {
      capacity_ = 1024 * (msg.data.size() / 1024 + 1);
      data = autoware::cuda_utils::make_unique<std::uint8_t[]>(capacity_);
    }

    cudaMemcpyAsync(data.get(), msg.data.data(), msg.data.size(), cudaMemcpyHostToDevice, stream);
  }

  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> data;
  cudaStream_t stream;

private:
  std::size_t capacity_{0};
};

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__UTILS__CUDA_POINTCLOUD_HPP_
