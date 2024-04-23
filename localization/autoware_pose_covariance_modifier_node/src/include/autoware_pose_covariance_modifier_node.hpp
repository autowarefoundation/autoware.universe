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
    gnss_pose_with_cov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ndt_pose_with_cov_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    new_pose_estimator_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_source_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr out_ndt_position_rmse_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr out_gnss_position_rmse_pub_;

  void gnss_pose_with_cov_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void ndt_pose_with_cov_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  std::array<double, 36> ndt_covariance_modifier(std::array<double, 36> & in_ndt_covariance);
  geometry_msgs::msg::PoseWithCovarianceStamped gnss_source_pose_with_cov;

  void check_gnss_pose_timeout();

private:
  void update_pose_source_based_on_rmse(
    double gnss_pose_average_rmse_xy, double gnss_pose_rmse_z,
    double gnss_pose_yaw_rmse_in_degrees);

  enum class PoseSource {
    GNSS = 0,
    GNSS_NDT = 1,
    NDT = 2,
  };

  rclcpp::Time gnssPoseLastReceivedTime_;

  PoseSource pose_source_;

  double gnss_error_reliable_max_, gnss_error_unreliable_min_, yaw_error_deg_threshold_;
  double gnss_pose_timeout_sec_;
  bool debug_;

  // covariance matrix indexes
  const int X_POS_IDX_ = 0;
  const int Y_POS_IDX_ = 7;
  const int Z_POS_IDX_ = 14;
  const int YAW_POS_IDX_ = 35;
};

#endif  // AUTOWARE_POSE_COVARIANCE_MODIFIER_NODE_HPP_
