// Copyright 2022 Tier IV, Inc.
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

#ifndef SAFE_VELOCITY_ADJUSTOR__POINTCLOUD_PROCESSING_HPP_
#define SAFE_VELOCITY_ADJUSTOR__POINTCLOUD_PROCESSING_HPP_

#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <Eigen/Core>
#include <pcl_ros/transforms.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

namespace safe_velocity_adjustor
{

/// @brief return the pointcloud msg transformed and converted to pcl format
inline pcl::PointCloud<pcl::PointXYZ> getTransformedPointCloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud_msg,
  const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  sensor_msgs::msg::PointCloud2 transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);

  return transformed_pointcloud;
}

/// @brief returns the pointcloud with only points within a given distance to the trajectory
inline pcl::PointCloud<pcl::PointXYZ> filterPointCloudByTrajectory(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud,
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double duration,
  const double distance)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_pointcloud;
  for (const auto & point : pointcloud.points) {
    for (const auto & trajectory_point : trajectory.points) {
      const auto radius = trajectory_point.longitudinal_velocity_mps * duration + distance;
      const double dx = trajectory_point.pose.position.x - point.x;
      const double dy = trajectory_point.pose.position.y - point.y;
      if (std::hypot(dx, dy) < radius) {
        filtered_pointcloud.points.push_back(point);
        break;
      }
    }
  }
  return filtered_pointcloud;
}

/// @brief returns the pointcloud transformed to the trajectory frame and in PCL format with only
/// points that are within range of the trajectory
inline pcl::PointCloud<pcl::PointXYZ> transformAndFilterPointCloud(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory,
  const sensor_msgs::msg::PointCloud2 & pointcloud,
  tier4_autoware_utils::TransformListener & transform_listener, const double duration,
  const double distance)
{
  // TODO(Maxime CLEMENT): we may need to remove dynamic obstacles from the point cloud
  const auto & header = pointcloud.header;
  const auto transform = transform_listener.getTransform(
    trajectory.header.frame_id, header.frame_id, header.stamp,
    rclcpp::Duration::from_nanoseconds(0));
  const auto obstacle_pointcloud = getTransformedPointCloud(pointcloud, transform->transform);
  return filterPointCloudByTrajectory(obstacle_pointcloud, trajectory, duration, distance);
}

}  // namespace safe_velocity_adjustor

#endif  // SAFE_VELOCITY_ADJUSTOR__POINTCLOUD_PROCESSING_HPP_
