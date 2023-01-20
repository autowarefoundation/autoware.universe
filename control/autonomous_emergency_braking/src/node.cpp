// Copyright 2022 TIER IV, Inc.
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

#include "autonomous_emergency_braking/node.hpp"

#include <pcl/filters/voxel_grid.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::motion::control::autonomous_emergency_braking
{
AEB::AEB(const rclcpp::NodeOptions & node_options) : Node("AEB", node_options)
{
  // Subscribers
  sub_point_cloud_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&AEB::onPointCloud, this, std::placeholders::_1));

  sub_velocity_ = this->create_subscription<VelocityReport>(
    "~/input/velocity", rclcpp::QoS{1}, std::bind(&AEB::onVelocity, this, std::placeholders::_1));

  sub_odometry_ = this->create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&AEB::onOdometry, this, std::placeholders::_1));

  sub_imu_ = this->create_subscription<Imu>(
    "~/input/imu", rclcpp::QoS{1}, std::bind(&AEB::onImu, this, std::placeholders::_1));

  // Publisher
  pub_obstacle_pointcloud_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/obstacle_pointcloud", 1);
}

void AEB::onVelocity(const VelocityReport::ConstSharedPtr input_msg)
{
  current_velocity_ptr_ = input_msg;
}

void AEB::onOdometry(const Odometry::ConstSharedPtr input_msg) { odometry_ptr_ = input_msg; }

void AEB::onImu(const Imu::ConstSharedPtr input_msg) { imu_ptr_ = input_msg; }

void AEB::onPointCloud(const PointCloud2::ConstSharedPtr input_msg)
{
  PointCloud::Ptr pointcloud_ptr(new PointCloud);
  pcl::fromROSMsg(*input_msg, *pointcloud_ptr);

  if (input_msg->header.frame_id != "base_link") {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[AEB]: Input point cloud frame is not base_link and it is " << input_msg->header.frame_id);
    // transform pointcloud
    geometry_msgs::msg::TransformStamped transform_stamped{};
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        "base_link", input_msg->header.frame_id, input_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "[AEB] Failed to look up transform from base_link to" << input_msg->header.frame_id);
      return;
    }

    // transform by using eigen matrix
    PointCloud2 transformed_points{};
    const Eigen::Matrix4f affine_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(affine_matrix, *input_msg, transformed_points);
    pcl::fromROSMsg(transformed_points, *pointcloud_ptr);
  }

  constexpr double voxel_grid_x = 0.05;
  constexpr double voxel_grid_y = 0.05;
  constexpr double voxel_grid_z = 100000.0;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  PointCloud::Ptr no_height_filtered_pointcloud_ptr(new PointCloud);
  filter.setInputCloud(pointcloud_ptr);
  filter.setLeafSize(voxel_grid_x, voxel_grid_y, voxel_grid_z);
  filter.filter(*no_height_filtered_pointcloud_ptr);

  obstacle_ros_pointcloud_ptr_ = std::make_shared<PointCloud2>();
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  obstacle_ros_pointcloud_ptr_->header = input_msg->header;
  pub_obstacle_pointcloud_->publish(*obstacle_ros_pointcloud_ptr_);
}

}  // namespace autoware::motion::control::autonomous_emergency_braking

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::autonomous_emergency_braking::AEB)
