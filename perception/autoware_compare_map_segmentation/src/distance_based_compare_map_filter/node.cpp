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

#include "node.hpp"

#include "autoware/universe_utils/ros/debug_publisher.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <memory>
#include <vector>

namespace autoware::compare_map_segmentation
{

void DistanceBasedStaticMapLoader::onMapCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr map)
{
  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::fromROSMsg<pcl::PointXYZ>(*map, map_pcl);
  const auto map_pcl_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_pcl);
  map_ptr_ = map_pcl_ptr;
  *tf_map_input_frame_ = map_ptr_->header.frame_id;
  if (!tree_) {
    if (map_ptr_->isOrganized()) {
      tree_.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
    } else {
      tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
    }
  }
  tree_->setInputCloud(map_ptr_);
  is_initialized_.store(true, std::memory_order_release);
}

bool DistanceBasedStaticMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, const double distance_threshold)
{
  if (!is_initialized_.load(std::memory_order_acquire)) {
    return false;
  }
  if (map_ptr_ == nullptr) {
    return false;
  }
  if (tree_ == nullptr) {
    return false;
  }

  std::vector<int> nn_indices(1);
  std::vector<float> nn_sqr_distances(1);
  if (!isFinite(point)) {
    return false;
  }
  if (!tree_->nearestKSearch(point, 1, nn_indices, nn_sqr_distances)) {
    return false;
  }
  if (nn_sqr_distances[0] > distance_threshold * distance_threshold) {
    return false;
  }
  return true;
}

bool DistanceBasedDynamicMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, const double distance_threshold)
{
  if (current_voxel_grid_dict_.size() == 0) {
    return false;
  }
  if (!isFinite(point)) {
    return false;
  }

  const int map_grid_index = static_cast<int>(
    std::floor((point.x - origin_x_) / map_grid_size_x_) +
    map_grids_x_ * std::floor((point.y - origin_y_) / map_grid_size_y_));

  if (static_cast<size_t>(map_grid_index) >= current_voxel_grid_array_.size()) {
    return false;
  }
  if (current_voxel_grid_array_.at(map_grid_index) != nullptr) {
    if (current_voxel_grid_array_.at(map_grid_index)->map_cell_kdtree == nullptr) {
      return false;
    }
    std::vector<int> nn_indices(1);
    std::vector<float> nn_distances(1);
    if (!current_voxel_grid_array_.at(map_grid_index)
           ->map_cell_kdtree->nearestKSearch(point, 1, nn_indices, nn_distances)) {
      return false;
    }
    if (nn_distances[0] <= distance_threshold) {
      return true;
    }
  }
  return false;
}

DistanceBasedCompareMapFilterComponent::DistanceBasedCompareMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("DistanceBasedCompareMapFilter", options)
{
  // initialize debug tool
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "distance_based_compare_map_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  distance_threshold_ = declare_parameter<double>("distance_threshold");
  bool use_dynamic_map_loading = declare_parameter<bool>("use_dynamic_map_loading");
  if (use_dynamic_map_loading) {
    rclcpp::CallbackGroup::SharedPtr main_callback_group;
    main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    distance_based_map_loader_ = std::make_unique<DistanceBasedDynamicMapLoader>(
      this, distance_threshold_, &tf_input_frame_, main_callback_group);
  } else {
    distance_based_map_loader_ =
      std::make_unique<DistanceBasedStaticMapLoader>(this, distance_threshold_, &tf_input_frame_);
  }
}

void DistanceBasedCompareMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);

  int point_step = input->point_step;
  int offset_x = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  int offset_y = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  int offset_z = input->fields[pcl::getFieldIndex(*input, "z")].offset;

  output.data.resize(input->data.size());
  output.point_step = point_step;
  size_t output_size = 0;
  for (size_t global_offset = 0; global_offset < input->data.size(); global_offset += point_step) {
    pcl::PointXYZ point{};
    std::memcpy(&point.x, &input->data[global_offset + offset_x], sizeof(float));
    std::memcpy(&point.y, &input->data[global_offset + offset_y], sizeof(float));
    std::memcpy(&point.z, &input->data[global_offset + offset_z], sizeof(float));
    if (distance_based_map_loader_->is_close_to_map(point, distance_threshold_)) {
      continue;
    }
    std::memcpy(&output.data[output_size], &input->data[global_offset], point_step);
    output_size += point_step;
  }
  output.header = input->header;
  output.fields = input->fields;
  output.data.resize(output_size);
  output.height = input->height;
  output.width = output_size / point_step / output.height;
  output.row_step = output_size / output.height;
  output.is_bigendian = input->is_bigendian;
  output.is_dense = input->is_dense;

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

}  // namespace autoware::compare_map_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::compare_map_segmentation::DistanceBasedCompareMapFilterComponent)
