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

#ifndef AUTOWARE__LIDAR_BEVFUSION__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_
#define AUTOWARE__LIDAR_BEVFUSION__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_

#include "autoware/lidar_bevfusion/preprocess/point_type.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstddef>
#include <list>
#include <string>
#include <utility>

namespace autoware::lidar_bevfusion
{

using autoware::cuda_utils::CudaUniquePtr;

class DensificationParam
{
public:
  DensificationParam(const std::string & world_frame_id, const unsigned int num_past_frames)
  : world_frame_id_(std::move(world_frame_id)),
    pointcloud_cache_size_(num_past_frames + /*current frame*/ 1)
  {
  }

  std::string world_frame_id() const { return world_frame_id_; }
  unsigned int pointcloud_cache_size() const { return pointcloud_cache_size_; }

private:
  std::string world_frame_id_;
  unsigned int pointcloud_cache_size_{1};
};

struct PointCloudWithTransform
{
  CudaUniquePtr<InputPointType[]> data_d{nullptr};
  std_msgs::msg::Header header;
  std::size_t num_points{0};
  Eigen::Affine3f affine_past2world;
};

class PointCloudDensification
{
public:
  explicit PointCloudDensification(const DensificationParam & param, cudaStream_t & stream);

  bool enqueuePointCloud(
    const sensor_msgs::msg::PointCloud2 & msg, const tf2_ros::Buffer & tf_buffer);

  double getCurrentTimestamp() const { return current_timestamp_; }
  Eigen::Affine3f getAffineWorldToCurrent() const { return affine_world2current_; }
  std::list<PointCloudWithTransform>::iterator getPointCloudCacheIter()
  {
    return pointcloud_cache_.begin();
  }
  bool isCacheEnd(std::list<PointCloudWithTransform>::iterator iter)
  {
    return iter == pointcloud_cache_.end();
  }
  std::size_t getIdx(std::list<PointCloudWithTransform>::iterator iter)
  {
    return std::distance(pointcloud_cache_.begin(), iter);
  }
  std::size_t getCacheSize()
  {
    return std::distance(pointcloud_cache_.begin(), pointcloud_cache_.end());
  }
  unsigned int pointcloud_cache_size() const { return param_.pointcloud_cache_size(); }

private:
  void enqueue(const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Affine3f & affine);
  void dequeue();

  DensificationParam param_;
  double current_timestamp_{0.0};
  Eigen::Affine3f affine_world2current_;
  std::list<PointCloudWithTransform> pointcloud_cache_;
  cudaStream_t stream_;
};

}  // namespace autoware::lidar_bevfusion

#endif  // AUTOWARE__LIDAR_BEVFUSION__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_
