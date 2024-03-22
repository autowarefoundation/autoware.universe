// Copyright 2024 Autoware Foundation. All rights reserved.
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

#ifndef CARLA_POINTCLOUD_INTERFACE__CARLA_POINTCLOUD_INTERFACE_NODE_HPP_
#define CARLA_POINTCLOUD_INTERFACE__CARLA_POINTCLOUD_INTERFACE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

class PointCloudInterface : public rclcpp::Node
{
public:
  explicit PointCloudInterface(const rclcpp::NodeOptions & node_options);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_raw;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr carla_cloud_;
  std::string tf_output;
  void processScan(const sensor_msgs::msg::PointCloud2::SharedPtr scanMsg);
  void setupTF();
};

#endif  // CARLA_POINTCLOUD_INTERFACE__CARLA_POINTCLOUD_INTERFACE_NODE_HPP_
