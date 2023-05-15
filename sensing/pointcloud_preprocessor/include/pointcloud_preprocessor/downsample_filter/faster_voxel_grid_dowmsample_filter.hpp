// Copyright 2020 Tier IV, Inc.
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

#include "pointcloud_preprocessor/filter.hpp"

#include <pcl/filters/voxel_grid.h>

#include <vector>

namespace pointcloud_preprocessor
{
class FasterVoxelGridDownsampleFilter
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  using TransformInfo = pointcloud_preprocessor::Filter::TransformInfo;

  FasterVoxelGridDownsampleFilter();
  void set_voxel_size(float voxel_size_x_, float voxel_size_y_, float voxel_size_z_);
  void filter(
    const PointCloud2ConstPtr & input, PointCloud2 & output, const TransformInfo & transform_info,
    const rclcpp::Logger & logger);

private:
  struct Centroid
  {
    float x;
    struct TransformInfo
    {
      TransformInfo()
      {
        eigen_transform = Eigen::Matrix4f::Identity(4, 4);
        need_transform = false;
      }

      Eigen::Matrix4f eigen_transform;
      bool need_transform;
    };
    float y;
    float z;
    uint32_t point_count_;

    Centroid() : x(0), y(0), z(0) {}
    Centroid(float _x, float _y, float _z) : x(_x), y(_y), z(_z) { this->point_count_ = 1; }

    void add_point(float _x, float _y, float _z)
    {
      this->x += _x;
      this->y += _y;
      this->z += _z;
      this->point_count_++;
    }

    void calc_centroid()
    {
      this->x /= this->point_count_;
      this->y /= this->point_count_;
      this->z /= this->point_count_;
    }
  };

  Eigen::Vector3f inverse_voxel_size_;
  std::vector<pcl::PCLPointField> xyz_fields_;
};
}  // namespace pointcloud_preprocessor
