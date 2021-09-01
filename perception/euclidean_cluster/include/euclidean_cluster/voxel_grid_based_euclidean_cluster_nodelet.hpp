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
#include "rclcpp/rclcpp.hpp"

// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/message_filter.h"
// #include "message_filters/subscriber.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"

namespace euclidean_cluster
{
class VoxelGridBasedEuclideanClusterNodelet : public rclcpp::Node
{
public:
  VoxelGridBasedEuclideanClusterNodelet(const rclcpp::NodeOptions & options);

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  geometry_msgs::msg::Point getCentroid(const sensor_msgs::msg::PointCloud2 & pointcloud);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr
    cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;

  // ROS Parameters
  std::string target_frame_;
  int min_cluster_size_;
  int max_cluster_size_;
  float tolerance_;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  float voxel_leaf_size_;
  int min_points_number_per_voxel_;
};

}  // namespace euclidean_cluster
