// Copyright 2023 Autoware Foundation
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

#include "compare_map_segmentation/voxel_grid_map_loader.hpp"

VoxelGridMapLoader::VoxelGridMapLoader(
  rclcpp::Node * node, double leaf_size, std::string * tf_map_input_frame, std::mutex * mutex)
: logger_(node->get_logger()), voxel_leaf_size_(leaf_size)
{
  tf_map_input_frame_ = tf_map_input_frame;
  mutex_ptr_ = mutex;

  downsampled_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/downsampled_map/pointcloud", rclcpp::QoS{1}.transient_local());
}

pcl::PointCloud<pcl::PointXYZ> VoxelGridMapLoader::getNeighborVoxelPoints(
  pcl::PointXYZ point, double voxel_size)
{
  pcl::PointCloud<pcl::PointXYZ> output;
  output.push_back(pcl::PointXYZ(point.x, point.y, point.z));
  output.push_back(pcl::PointXYZ(point.x, point.y - voxel_size, point.z - voxel_size));
  output.push_back(pcl::PointXYZ(point.x, point.y - voxel_size, point.z));
  output.push_back(pcl::PointXYZ(point.x, point.y - voxel_size, point.z + voxel_size));
  output.push_back(pcl::PointXYZ(point.x, point.y, point.z - voxel_size));
  output.push_back(pcl::PointXYZ(point.x, point.y, point.z + voxel_size));
  output.push_back(pcl::PointXYZ(point.x, point.y + voxel_size, point.z - voxel_size));
  output.push_back(pcl::PointXYZ(point.x, point.y + voxel_size, point.z));
  output.push_back(pcl::PointXYZ(point.x, point.y + voxel_size, point.z + voxel_size));

  output.push_back(pcl::PointXYZ(point.x - voxel_size, point.y - voxel_size, point.z - voxel_size));
  output.push_back(pcl::PointXYZ(point.x - voxel_size, point.y - voxel_size, point.z));
  output.push_back(pcl::PointXYZ(point.x - voxel_size, point.y - voxel_size, point.z + voxel_size));
  output.push_back(pcl::PointXYZ(point.x - voxel_size, point.y, point.z - voxel_size));
  output.push_back(pcl::PointXYZ(point.x - voxel_size, point.y, point.z));
  output.push_back(pcl::PointXYZ(point.x - voxel_size, point.y, point.z + voxel_size));
  output.push_back(pcl::PointXYZ(point.x - voxel_size, point.y + voxel_size, point.z - voxel_size));
  output.push_back(pcl::PointXYZ(point.x - voxel_size, point.y + voxel_size, point.z));
  output.push_back(pcl::PointXYZ(point.x - voxel_size, point.y + voxel_size, point.z + voxel_size));

  output.push_back(pcl::PointXYZ(point.x + voxel_size, point.y - voxel_size, point.z - voxel_size));
  output.push_back(pcl::PointXYZ(point.x + voxel_size, point.y - voxel_size, point.z));
  output.push_back(pcl::PointXYZ(point.x + voxel_size, point.y - voxel_size, point.z + voxel_size));
  output.push_back(pcl::PointXYZ(point.x + voxel_size, point.y, point.z - voxel_size));
  output.push_back(pcl::PointXYZ(point.x + voxel_size, point.y, point.z));
  output.push_back(pcl::PointXYZ(point.x + voxel_size, point.y, point.z + voxel_size));
  output.push_back(pcl::PointXYZ(point.x + voxel_size, point.y + voxel_size, point.z - voxel_size));
  output.push_back(pcl::PointXYZ(point.x + voxel_size, point.y + voxel_size, point.z));
  output.push_back(pcl::PointXYZ(point.x + voxel_size, point.y + voxel_size, point.z + voxel_size));
  return output;
}

bool VoxelGridMapLoader::is_close_points(
  const pcl::PointXYZ point, const pcl::PointXYZ target_point,
  const double distance_threshold) const
{
  if (distance3D(point, target_point) < distance_threshold * distance_threshold) {
    return true;
  }
  return false;
}

void VoxelGridMapLoader::publish_downsampled_map(
  const pcl::PointCloud<pcl::PointXYZ> & downsampled_pc)
{
  sensor_msgs::msg::PointCloud2 downsampled_map_msg;
  pcl::toROSMsg(downsampled_pc, downsampled_map_msg);
  downsampled_map_msg.header.frame_id = "map";
  downsampled_map_pub_->publish(downsampled_map_msg);
}

//*************************** for Static Map loader Voxel Grid Filter *************

VoxelGridStaticMapLoader::VoxelGridStaticMapLoader(
  rclcpp::Node * node, double leaf_size, std::string * tf_map_input_frame, std::mutex * mutex)
: VoxelGridMapLoader(node, leaf_size, tf_map_input_frame, mutex)
{
  sub_map_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS{1}.transient_local(),
    std::bind(&VoxelGridStaticMapLoader::onMapCallback, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "VoxelGridStaticMapLoader initialized.\n");
}

void VoxelGridStaticMapLoader::onMapCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr map)
{
  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::fromROSMsg<pcl::PointXYZ>(*map, map_pcl);
  const auto map_pcl_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_pcl);
  *tf_map_input_frame_ = map_pcl_ptr->header.frame_id;
  (*mutex_ptr_).lock();
  voxel_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_grid_.setInputCloud(map_pcl_ptr);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr_);
  (*mutex_ptr_).unlock();
  publish_downsampled_map(*voxel_map_ptr_);
}
bool VoxelGridStaticMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, const double distance_threshold)
{
  // check map downsampled pc
  if (voxel_map_ptr_ == NULL) {
    return false;
  }
  auto neighbor_voxel_points = getNeighborVoxelPoints(point, distance_threshold);
  int voxel_index;
  for (size_t j = 0; j < neighbor_voxel_points.size(); ++j) {
    voxel_index = voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(
      neighbor_voxel_points[j].x, neighbor_voxel_points[j].y, neighbor_voxel_points[j].z));

    if (voxel_index == -1) {
      continue;
    }
    if (is_close_points(voxel_map_ptr_->points.at(voxel_index), point, distance_threshold)) {
      return true;
    }
  }
  return false;
}
//*************** for Dynamic and Differential Map loader Voxel Grid Filter *************

VoxelGridDynamicMapLoader::VoxelGridDynamicMapLoader(
  rclcpp::Node * node, double leaf_size, std::string * tf_map_input_frame, std::mutex * mutex,
  rclcpp::CallbackGroup::SharedPtr main_callback_group)
: VoxelGridMapLoader(node, leaf_size, tf_map_input_frame, mutex)
{
  auto timer_interval_ms = node->declare_parameter<int>("timer_interval_ms");
  map_update_distance_threshold_ = node->declare_parameter<double>("map_update_distance_threshold");
  map_loader_radius_ = node->declare_parameter<double>("map_loader_radius");
  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  sub_estimated_pose_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_with_covariance", rclcpp::QoS{1},
    std::bind(&VoxelGridDynamicMapLoader::onEstimatedPoseCallback, this, std::placeholders::_1),
    main_sub_opt);
  RCLCPP_INFO(logger_, "VoxelGridDynamicMapLoader initialized.\n");

  client_callback_group_ =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_update_client_ = node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>(
    "map_loader_service", rmw_qos_profile_services_default, client_callback_group_);

  while (!map_update_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(logger_, "service not available, waiting again ...");
  }

  timer_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_update_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(timer_interval_ms),
    std::bind(&VoxelGridDynamicMapLoader::timer_callback, this), timer_callback_group_);
}

void VoxelGridDynamicMapLoader::onEstimatedPoseCallback(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  current_position_ = msg->pose.pose.position;
}

bool VoxelGridDynamicMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, const double distance_threshold)
{
  if (current_voxel_grid_dict_.size() == 0) {
    return false;
  }
  for (const auto & kv : current_voxel_grid_dict_) {
    // TODO(badai-nguyen): add neighboor map cells checking for points on boundary when map loader
    // I/F is updated
    if (point.x < kv.second.min_b_x) {
      continue;
    }
    if (point.y < kv.second.min_b_y) {
      continue;
    }
    if (point.x > kv.second.max_b_x) {
      continue;
    }
    if (point.y > kv.second.max_b_y) {
      continue;
    }
    // the map cell is found
    if (kv.second.map_cell_pc_ptr == NULL) {
      return false;
    }
    auto neighbor_voxel_points = getNeighborVoxelPoints(point, distance_threshold);
    int voxel_index;
    for (size_t j = 0; j < neighbor_voxel_points.size(); ++j) {
      voxel_index = kv.second.map_cell_voxel_grid.getCentroidIndexAt(
        kv.second.map_cell_voxel_grid.getGridCoordinates(
          neighbor_voxel_points[j].x, neighbor_voxel_points[j].y, neighbor_voxel_points[j].z));
      if (voxel_index == -1) {
        continue;
      }
      if (is_close_points(
            kv.second.map_cell_pc_ptr->points.at(voxel_index), point, distance_threshold)) {
        return true;
      }
    }
    return false;
  }
  return false;
}
void VoxelGridDynamicMapLoader::timer_callback()
{
  if (current_position_ == std::nullopt) {
    return;
  }
  if (last_updated_position_ == std::nullopt) {
    request_update_map(current_position_.value());
    last_updated_position_ = current_position_;
    return;
  }

  if (should_update_map()) {
    RCLCPP_INFO(logger_, "Vehicle moved. Update current map!.\n");
    last_updated_position_ = current_position_;
    request_update_map((current_position_.value()));
    last_updated_position_ = current_position_;
  }
}

bool VoxelGridDynamicMapLoader::should_update_map() const
{
  if (
    distance2D(current_position_.value(), last_updated_position_.value()) >
    map_update_distance_threshold_) {
    return true;
  }
  return false;
}

void VoxelGridDynamicMapLoader::request_update_map(const geometry_msgs::msg::Point & position)
{
  auto exe_start_time = std::chrono::system_clock::now();
  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();
  request->area.center = position;
  request->area.radius = map_loader_radius_;
  request->cached_ids = getCurrentMapIDs();

  auto result{map_update_client_->async_send_request(
    request,
    [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    RCLCPP_INFO(logger_, "Waiting for response...\n");
    if (!rclcpp::ok()) {
      return;
    }
    status = result.wait_for(std::chrono::seconds(1));
  }

  //
  if (status == std::future_status::ready) {
    if (
      result.get()->new_pointcloud_with_ids.size() > 0 || result.get()->ids_to_remove.size() > 0) {
      updateDifferentialMapCells(
        result.get()->new_pointcloud_with_ids, result.get()->ids_to_remove);
    }
  }

  auto exe_stop_time = std::chrono::system_clock::now();
  const double exe_run_time =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_stop_time - exe_start_time).count() /
    1000.0;
  RCLCPP_INFO(logger_, "Current map updating time: %lf [ms]", exe_run_time);

  publish_downsampled_map(getCurrentDownsampledMapPc());
}
