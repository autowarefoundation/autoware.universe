// Copyright 2025 TIER IV, Inc.
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

#pragma once

#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/cuda_traits.hpp"

#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::pointcloud_preprocessor
{

class CudaPointCloudConcatenateDataSynchronizerComponent
: public PointCloudConcatenateDataSynchronizerComponentTemplated<CudaPointCloud2Traits>
{
public:
  explicit CudaPointCloudConcatenateDataSynchronizerComponent(
    const rclcpp::NodeOptions & node_options)
  : PointCloudConcatenateDataSynchronizerComponentTemplated<CudaPointCloud2Traits>(node_options)
  {
  }
  ~CudaPointCloudConcatenateDataSynchronizerComponent() override = default;
};

}  // namespace autoware::pointcloud_preprocessor
