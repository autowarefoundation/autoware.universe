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

#include "compare_map_segmentation/voxel_distance_based_compare_map_filter_nodelet.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <vector>

namespace compare_map_segmentation
{
using pointcloud_preprocessor::get_param;

VoxelDistanceBasedCompareMapFilterComponent::VoxelDistanceBasedCompareMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("VoxelDistanceBasedCompareMapFilter", options)
{
  distance_threshold_ = static_cast<double>(declare_parameter("distance_threshold", 0.3));

  using std::placeholders::_1;
  sub_map_ = this->create_subscription<PointCloud2>(
    "map", rclcpp::QoS{1}.transient_local(),
    std::bind(&VoxelDistanceBasedCompareMapFilterComponent::input_target_callback, this, _1));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&VoxelDistanceBasedCompareMapFilterComponent::paramCallback, this, _1));
}

void VoxelDistanceBasedCompareMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (voxel_map_ptr_ == NULL || map_ptr_ == NULL || tree_ == NULL) {
    output = *input;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    const int index = voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(
      pcl_input->points.at(i).x, pcl_input->points.at(i).y, pcl_input->points.at(i).z));
    if (index == -1) {                 // empty voxel
      std::vector<int> nn_indices(1);  // nn means nearest neighbor
      std::vector<float> nn_distances(1);
      if (
        tree_->radiusSearch(
          pcl_input->points.at(i), distance_threshold_, nn_indices, nn_distances, 1) == 0) {
        pcl_output->points.push_back(pcl_input->points.at(i));
      }
    }
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

//******************************* Static map loader *****************************
void VoxelDistanceBasedStaticMapLoader::onMapCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr map)
{
  RCLCPP_INFO(logger_, "Voxelization for Static Map Loader ");
  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::fromROSMsg<pcl::PointXYZ>(*map, map_pcl);
  const auto map_pcl_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_pcl);

  std::scoped_lock lock(mutex_);
  tf_input_frame_ = map_pcl_ptr->header.frame_id;
  // voxel
  voxel_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(distance_threshold_, distance_threshold_, distance_threshold_);
  voxel_grid_.setInputCloud(map_pcl_ptr);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr_);
  // kdtree
  map_ptr_ = map_pcl_ptr;

  RCLCPP_INFO(logger_, "Create tree for Static Map Loader ");
  if (!tree_) {
    if (map_ptr_->isOrganized()) {
      tree_.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
    } else {
      tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
    }
  }
  tree_->setInputCloud(map_ptr_);
}

bool VoxelDistanceBasedStaticMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, const double distance_threshold)
{
  if (voxel_map_ptr_ == NULL) {
    return false;
  }
  if (map_ptr_ == NULL) {
    return false;
  }
  if (tree_ == NULL) {
    return false;
  }
  if (is_close_to_neighbor_voxels(point, distance_threshold, voxel_grid_, tree_)) {
    return true;
  }
  return false;
}

//******************************* Dynamic map loader ***********************************
bool VoxelDistanceBasedDynamicMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, const double distance_threshold)
{
  if (current_voxel_grid_dict_.size() == 0) {
    return false;
  }

  const int map_grid_index = static_cast<int>(
    std::floor((point.x - origin_x_) / map_grid_size_x_) +
    map_grids_x_ * std::floor((point.y - origin_y_) / map_grid_size_y_));

  if (static_cast<size_t>(map_grid_index) >= current_voxel_grid_array_.size()) {
    return false;
  }
  if (
    current_voxel_grid_array_.at(map_grid_index) != NULL &&
    is_close_to_neighbor_voxels(
      point, distance_threshold, current_voxel_grid_array_.at(map_grid_index)->map_cell_voxel_grid,
      current_voxel_grid_array_.at(map_grid_index)->map_cell_kdtree)) {
    return true;
  }
  return false;
}

// *************************************************************************************
VoxelDistanceBasedCompareMapFilterComponent::VoxelDistanceBasedCompareMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("VoxelDistanceBasedCompareMapFilter", options)
{
  // initilize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ =
      std::make_unique<DebugPublisher>(this, "voxel_distance_based_compare_map_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  distance_threshold_ = static_cast<double>(declare_parameter("distance_threshold", 0.3));
  bool use_dynamic_map_loading = declare_parameter<bool>("use_dynamic_map_loading");
  if (use_dynamic_map_loading) {
    rclcpp::CallbackGroup::SharedPtr main_callback_group;
    main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    voxel_distance_based_map_loader_ = std::make_unique<VoxelDistanceBasedDynamicMapLoader>(
      this, distance_threshold_, &tf_input_frame_, &mutex_, main_callback_group);
  } else {
    voxel_distance_based_map_loader_ = std::make_unique<VoxelDistanceBasedStaticMapLoader>(
      this, distance_threshold_, &tf_input_frame_, &mutex_);
  }
}

void VoxelDistanceBasedCompareMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    if (voxel_distance_based_map_loader_->is_close_to_map(
          pcl_input->points.at(i), distance_threshold_)) {
      continue;
    }
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", distance_threshold_);
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}
}  // namespace compare_map_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  compare_map_segmentation::VoxelDistanceBasedCompareMapFilterComponent)
