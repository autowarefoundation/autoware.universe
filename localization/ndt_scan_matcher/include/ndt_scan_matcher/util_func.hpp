// Copyright 2015-2019 Autoware Foundation
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

#include <algorithm>
#include <cmath>
#include <deque>
#include <random>

#include "tf2_eigen/tf2_eigen.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// ref by http://takacity.blog.fc2.com/blog-entry-69.html
std_msgs::msg::ColorRGBA ExchangeColorCrc(double x);

double calcDiffForRadian(const double lhs_rad, const double rhs_rad);

// x: roll, y: pitch, z: yaw
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseStamped & pose);
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

// geometry_msgs::msg::Twist calcTwist(const geometry_msgs::msg::PoseStamped &pose_a,
//                                const geometry_msgs::msg::PoseStamped &pose_b)
// {
//   const double dt = (pose_a.header.stamp - pose_b.header.stamp).toSec();
//
//   if (dt == 0) {
//     return geometry_msgs::msg::Twist();
//   }
//
//   const auto pose_a_rpy = getRPY(pose_a);
//   const auto pose_b_rpy = getRPY(pose_b);
//
//   geometry_msgs::msg::Vector3 diff_xyz;
//   geometry_msgs::msg::Vector3 diff_rpy;
//
//   diff_xyz.x = pose_a.pose.position.x - pose_b.pose.position.x;
//   diff_xyz.y = pose_a.pose.position.y - pose_b.pose.position.y;
//   diff_xyz.z = pose_a.pose.position.z - pose_b.pose.position.z;
//   diff_rpy.x = calcDiffForRadian(pose_a_rpy.x, pose_b_rpy.x);
//   diff_rpy.y = calcDiffForRadian(pose_a_rpy.y, pose_b_rpy.y);
//   diff_rpy.z = calcDiffForRadian(pose_a_rpy.z, pose_b_rpy.z);
//
//   geometry_msgs::msg::Twist twist;
//   twist.linear.x = diff_xyz.x / dt;
//   twist.linear.y = diff_xyz.y / dt;
//   twist.linear.z = diff_xyz.z / dt;
//   twist.angular.x = diff_rpy.x / dt;
//   twist.angular.y = diff_rpy.y / dt;
//   twist.angular.z = diff_rpy.z / dt;
//
//   return twist;
// }

geometry_msgs::msg::Twist calcTwist(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b);

void getNearestTimeStampPose(
  const std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> &
  pose_cov_msg_ptr_array,
  const rclcpp::Time & time_stamp,
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & output_old_pose_cov_msg_ptr,
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & output_new_pose_cov_msg_ptr);

geometry_msgs::msg::PoseStamped interpolatePose(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b,
  const rclcpp::Time & time_stamp);

geometry_msgs::msg::PoseStamped interpolatePose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_b,
  const rclcpp::Time & time_stamp);

void popOldPose(
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> &
  pose_cov_msg_ptr_array,
  const rclcpp::Time & time_stamp);

geometry_msgs::msg::PoseArray createRandomPoseArray(
  const geometry_msgs::msg::PoseWithCovarianceStamped & base_pose_with_cov,
  const size_t particle_num);
