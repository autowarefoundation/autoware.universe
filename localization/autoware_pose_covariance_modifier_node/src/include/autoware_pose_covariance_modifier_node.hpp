// Copyright 2024 The Autoware Foundation
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
#ifndef AUTOWARE_POSE_COVARIANCE_MODIFIER_NODE_HPP_
#define AUTOWARE_POSE_COVARIANCE_MODIFIER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

class AutowarePoseCovarianceModifierNode : public rclcpp::Node
{
public:
  AutowarePoseCovarianceModifierNode();

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    trusted_pose_with_cov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ndt_pose_with_cov_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    new_pose_estimator_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_source_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr out_ndt_position_rmse_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr out_gnss_position_rmse_pub_;

  void trusted_pose_with_cov_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void ndt_pose_with_cov_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  std::array<double, 36> ndt_covariance_modifier(std::array<double, 36> & in_ndt_covariance);
  geometry_msgs::msg::PoseWithCovarianceStamped trusted_source_pose_with_cov;

  void checkTrustedPoseTimeout();

private:
  void selectPositionSource(
    double trusted_pose_average_rmse_xy, double trusted_pose_yaw_rmse_in_degrees);

  enum class PoseSource {
    GNSS = 0,
    GNSS_NDT = 1,
    NDT = 2,
  };
  void publishPoseSource();
  int pose_source_;
  rclcpp::Time trustedPoseCallbackTime;

  double gnss_error_reliable_max_, gnss_error_unreliable_min_, yaw_error_deg_threshold_;
  bool debug_;
};

#endif  // AUTOWARE_POSE_COVARIANCE_MODIFIER_NODE_HPP_
