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

#ifndef VOXEL_BASED_APPROXIMATE_COMPARE_MAP_FILTER__NODE_HPP_  // NOLINT
#define VOXEL_BASED_APPROXIMATE_COMPARE_MAP_FILTER__NODE_HPP_  // NOLINT

#include "../voxel_grid_map_loader/voxel_grid_map_loader.hpp"
#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>

#include <memory>
#include <string>

namespace autoware::compare_map_segmentation
{

class VoxelBasedApproximateStaticMapLoader : public VoxelGridStaticMapLoader
{
public:
  explicit VoxelBasedApproximateStaticMapLoader(
    rclcpp::Node * node, double leaf_size, double downsize_ratio_z_axis,
    std::string * tf_map_input_frame)
  : VoxelGridStaticMapLoader(node, leaf_size, downsize_ratio_z_axis, tf_map_input_frame)
  {
    RCLCPP_INFO(logger_, "VoxelBasedApproximateStaticMapLoader initialized.\n");
  }
  bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold) override;
};

class VoxelBasedApproximateDynamicMapLoader : public VoxelGridDynamicMapLoader
{
public:
  VoxelBasedApproximateDynamicMapLoader(
    rclcpp::Node * node, double leaf_size, double downsize_ratio_z_axis,
    std::string * tf_map_input_frame, rclcpp::CallbackGroup::SharedPtr main_callback_group)
  : VoxelGridDynamicMapLoader(
      node, leaf_size, downsize_ratio_z_axis, tf_map_input_frame, main_callback_group)
  {
    RCLCPP_INFO(logger_, "VoxelBasedApproximateDynamicMapLoader initialized.\n");
  }
  bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold) override;
};

class VoxelBasedApproximateCompareMapFilterComponent
: public autoware::pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

private:
  double distance_threshold_;
  std::unique_ptr<VoxelGridMapLoader> voxel_based_approximate_map_loader_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VoxelBasedApproximateCompareMapFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::compare_map_segmentation

// clang-format off
#endif  // VOXEL_BASED_APPROXIMATE_COMPARE_MAP_FILTER__NODE_HPP_  // NOLINT
// clang-format on
