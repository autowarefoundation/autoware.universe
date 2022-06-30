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

#ifndef LOCALIZATION_EVALUATOR__LOCALIZATION_EVALUATOR_CORE_HPP_
#define LOCALIZATION_EVALUATOR__LOCALIZATION_EVALUATOR_CORE_HPP_

#include <eigen3/Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class LocalizationEvaluator : public rclcpp::Node
{
public:
  LocalizationEvaluator();
  ~LocalizationEvaluator();

private:
  void callbackVehicleOdometry(geometry_msgs::msg::PoseStamped::ConstSharedPtr odom_msg_ptr);
  void callbackGroundTruthOdometry(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr odom_ground_truth_msg_ptr);
  void calculateError(
    Eigen::Affine3d & vehicle_trans, Eigen::Affine3d & ground_truth_trans,
    Eigen::Affine3d & error_trans);
  void publishError(Eigen::Affine3d & error_trans);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_error_pub_;

  Eigen::Affine3d last_vehicle_trans_;
  Eigen::Affine3d last_ground_truth_trans_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr last_ground_truth_pose_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr curr_ground_truth_pose_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr prev_vehicle_pose_;
  bool has_ground_truth_;
};

#endif  // LOCALIZATION_EVALUATOR__LOCALIZATION_EVALUATOR_CORE_HPP_
