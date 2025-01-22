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

#ifndef VOXEL_DISTANCE_BASED_COMPARE_MAP_FILTER__NODE_HPP_  // NOLINT
#define VOXEL_DISTANCE_BASED_COMPARE_MAP_FILTER__NODE_HPP_  // NOLINT

#include "../voxel_grid_map_loader/voxel_grid_map_loader.hpp"
#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>

#include <map>
#include <memory>
#include <string>
#include <utility>

namespace autoware::compare_map_segmentation
{

using FilteredPointCloud = typename pcl::Filter<pcl::PointXYZ>::PointCloud;
using FilteredPointCloudPtr = typename FilteredPointCloud::Ptr;
using FilteredPointCloudConstPtr = typename FilteredPointCloud::ConstPtr;

class VoxelDistanceBasedStaticMapLoader : public VoxelGridStaticMapLoader
{
private:
  FilteredPointCloudConstPtr map_ptr_;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree_;

public:
  explicit VoxelDistanceBasedStaticMapLoader(
    rclcpp::Node * node, double leaf_size, double downsize_ratio_z_axis,
    std::string * tf_map_input_frame)
  : VoxelGridStaticMapLoader(node, leaf_size, downsize_ratio_z_axis, tf_map_input_frame)
  {
    RCLCPP_INFO(logger_, "VoxelDistanceBasedStaticMapLoader initialized.\n");
  }
  bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold) override;
  void onMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr map) override;
};

class VoxelDistanceBasedDynamicMapLoader : public VoxelGridDynamicMapLoader
{
protected:
private:
  FilteredPointCloudConstPtr map_ptr_;
  /* data */
public:
  explicit VoxelDistanceBasedDynamicMapLoader(
    rclcpp::Node * node, double leaf_size, double downsize_ratio_z_axis,
    std::string * tf_map_input_frame, rclcpp::CallbackGroup::SharedPtr main_callback_group)
  : VoxelGridDynamicMapLoader(
      node, leaf_size, downsize_ratio_z_axis, tf_map_input_frame, main_callback_group)
  {
    RCLCPP_INFO(logger_, "VoxelDistanceBasedDynamicMapLoader initialized.\n");
  }
  bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold) override;

  inline void addMapCellAndFilter(
    const autoware_map_msgs::msg::PointCloudMapCellWithID & map_cell_to_add) override
  {
    map_grid_size_x_ = map_cell_to_add.metadata.max_x - map_cell_to_add.metadata.min_x;
    map_grid_size_y_ = map_cell_to_add.metadata.max_y - map_cell_to_add.metadata.min_y;

    pcl::PointCloud<pcl::PointXYZ> map_cell_pc_tmp;
    pcl::fromROSMsg(map_cell_to_add.pointcloud, map_cell_pc_tmp);

    VoxelGridPointXYZ map_cell_voxel_grid_tmp;
    FilteredPointCloudPtr map_cell_downsampled_pc_ptr_tmp;

    auto map_cell_voxel_input_tmp_ptr =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_cell_pc_tmp);
    map_cell_voxel_grid_tmp.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    map_cell_downsampled_pc_ptr_tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_cell_voxel_grid_tmp.setInputCloud(map_cell_voxel_input_tmp_ptr);
    map_cell_voxel_grid_tmp.setSaveLeafLayout(true);
    map_cell_voxel_grid_tmp.filter(*map_cell_downsampled_pc_ptr_tmp);

    MapGridVoxelInfo current_voxel_grid_list_item;
    current_voxel_grid_list_item.min_b_x = map_cell_to_add.metadata.min_x;
    current_voxel_grid_list_item.min_b_y = map_cell_to_add.metadata.min_y;
    current_voxel_grid_list_item.max_b_x = map_cell_to_add.metadata.max_x;
    current_voxel_grid_list_item.max_b_y = map_cell_to_add.metadata.max_y;

    current_voxel_grid_list_item.map_cell_voxel_grid.set_voxel_grid(
      &(map_cell_voxel_grid_tmp.leaf_layout_), map_cell_voxel_grid_tmp.get_min_b(),
      map_cell_voxel_grid_tmp.get_max_b(), map_cell_voxel_grid_tmp.get_div_b(),
      map_cell_voxel_grid_tmp.get_divb_mul(), map_cell_voxel_grid_tmp.get_inverse_leaf_size());

    current_voxel_grid_list_item.map_cell_pc_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_voxel_grid_list_item.map_cell_pc_ptr = std::move(map_cell_downsampled_pc_ptr_tmp);

    // add kdtree
    pcl::search::Search<pcl::PointXYZ>::Ptr tree_tmp;
    if (!tree_tmp) {
      if (map_cell_voxel_input_tmp_ptr->isOrganized()) {
        tree_tmp.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
      } else {
        tree_tmp.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
      }
    }
    tree_tmp->setInputCloud(map_cell_voxel_input_tmp_ptr);
    current_voxel_grid_list_item.map_cell_kdtree = tree_tmp;

    // add
    std::lock_guard<std::mutex> lock(dynamic_map_loader_mutex_);
    current_voxel_grid_dict_.insert({map_cell_to_add.cell_id, current_voxel_grid_list_item});
  }
};

class VoxelDistanceBasedCompareMapFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  void input_target_callback(const PointCloud2ConstPtr map);

private:
  std::unique_ptr<VoxelGridMapLoader> voxel_distance_based_map_loader_;
  double distance_threshold_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VoxelDistanceBasedCompareMapFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::compare_map_segmentation

// clang-format off
#endif  // VOXEL_DISTANCE_BASED_COMPARE_MAP_FILTER__NODE_HPP_  // NOLINT
// clang-format on
