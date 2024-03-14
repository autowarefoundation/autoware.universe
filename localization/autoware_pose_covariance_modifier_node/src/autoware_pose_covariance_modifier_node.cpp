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

#include "include/autoware_pose_covariance_modifier_node.hpp"

#include <rclcpp/rclcpp.hpp>

AutowarePoseCovarianceModifierNode::AutowarePoseCovarianceModifierNode()
: Node("AutowarePoseCovarianceModifierNode")
{
  gnss_error_reliable_max_ =
    this->declare_parameter<double>("error_thresholds.gnss_error_reliable_max", 0.10);
  gnss_error_unreliable_min_ =
    this->declare_parameter<double>("error_thresholds.gnss_error_unreliable_min", 0.25);
  yaw_error_deg_threshold_ =
    this->declare_parameter<double>("error_thresholds.yaw_error_deg_threshold", 0.3);
  debug_ = this->declare_parameter<bool>("debug.enable_debug_topics", false);

  trusted_pose_with_cov_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "input_trusted_pose_with_cov_topic", 10000,
      std::bind(
        &AutowarePoseCovarianceModifierNode::trusted_pose_with_cov_callback, this,
        std::placeholders::_1));

  ndt_pose_with_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input_ndt_pose_with_cov_topic", 10000,
    std::bind(
      &AutowarePoseCovarianceModifierNode::ndt_pose_with_cov_callback, this,
      std::placeholders::_1));
  new_pose_estimator_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "output_pose_with_covariance_topic", 10);

  pose_source_pub_ = this->create_publisher<std_msgs::msg::String>(
    "autoware_pose_covariance_modifier/selected_pose_type", 10);
  if (debug_) {
    out_ndt_position_rmse_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/autoware_pose_covariance_modifier/output/ndt_position_rmse", 10);
    out_gnss_position_rmse_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/autoware_pose_covariance_modifier/output/gnss_position_rmse", 10);
  }
}
void AutowarePoseCovarianceModifierNode::checkTrustedPoseTimeout()
{
  auto timeDiff = this->now() - trustedPoseCallbackTime;
  if (timeDiff.seconds() > 1.0) {
    RCLCPP_WARN(this->get_logger(), "Trusted Pose Timeout");
    pose_source_ = static_cast<int>(AutowarePoseCovarianceModifierNode::PoseSource::NDT);
    std_msgs::msg::String selected_pose_type;
    selected_pose_type.data = "NDT";
    pose_source_pub_->publish(selected_pose_type);
  }
}

std::array<double, 36> AutowarePoseCovarianceModifierNode::ndt_covariance_modifier(
  std::array<double, 36> & in_ndt_covariance)
{
  std::array<double, 36> ndt_covariance = in_ndt_covariance;

  /*
   * ndt_min_rmse_meters = in_ndt_rmse
   * ndt_max_rmse_meters = in_ndt_rmse  * 2
   * ndt_rmse = ndt_max_rmse_meters - (gnss_rmse * (ndt_max_rmse_meters - ndt_min_rmse_meters) /
   * normalization_coeff)
   */
  double normalization_coeff = 0.1;
  ndt_covariance[0] = std::pow(
    ((std::sqrt(in_ndt_covariance[0]) * 2) -
     std::sqrt(trusted_source_pose_with_cov.pose.covariance[0])) *
      (std::sqrt(in_ndt_covariance[0])) / normalization_coeff,
    2);
  ndt_covariance[7] = std::pow(
    ((std::sqrt(in_ndt_covariance[7]) * 2) -
     std::sqrt(trusted_source_pose_with_cov.pose.covariance[7])) *
      (std::sqrt(in_ndt_covariance[7])) / normalization_coeff,
    2);
  ndt_covariance[14] = std::pow(
    ((std::sqrt(in_ndt_covariance[14]) * 2) -
     std::sqrt(trusted_source_pose_with_cov.pose.covariance[14])) *
      (std::sqrt(in_ndt_covariance[14])) / normalization_coeff,
    2);

  ndt_covariance[0] = std::max(ndt_covariance[0], in_ndt_covariance[0]);
  ndt_covariance[7] = std::max(ndt_covariance[7], in_ndt_covariance[7]);
  ndt_covariance[14] = std::max(ndt_covariance[14], in_ndt_covariance[14]);

  return ndt_covariance;
}
void AutowarePoseCovarianceModifierNode::ndt_pose_with_cov_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  AutowarePoseCovarianceModifierNode::checkTrustedPoseTimeout();

  std::array<double, 36> ndt_covariance_in_ = msg->pose.covariance;
  std::array<double, 36> out_ndt_covariance_ =
    pose_source_ == 1 ? ndt_covariance_modifier(ndt_covariance_in_) : ndt_covariance_in_;

  geometry_msgs::msg::PoseWithCovarianceStamped ndt_pose_with_covariance = *msg;
  ndt_pose_with_covariance.pose.covariance = out_ndt_covariance_;
  if (pose_source_ != 0) {
    new_pose_estimator_pub_->publish(ndt_pose_with_covariance);
    if (debug_) {
      std_msgs::msg::Float32 out_ndt_rmse;
      out_ndt_rmse.data = (std::sqrt(ndt_pose_with_covariance.pose.covariance[0]) +
                           std::sqrt(ndt_pose_with_covariance.pose.covariance[7])) /
                          2;
      out_ndt_position_rmse_pub_->publish(out_ndt_rmse);
    }
  }
}
void AutowarePoseCovarianceModifierNode::trusted_pose_with_cov_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  trustedPoseCallbackTime = this->now();
  trusted_source_pose_with_cov = *msg;

  double trusted_pose_average_rmse_xy;
  double trusted_pose_yaw_rmse_in_degrees;
  trusted_pose_average_rmse_xy = (std::sqrt(trusted_source_pose_with_cov.pose.covariance[0]) +
                                  std::sqrt(trusted_source_pose_with_cov.pose.covariance[7])) /
                                 2;

  trusted_pose_yaw_rmse_in_degrees =
    std::sqrt(trusted_source_pose_with_cov.pose.covariance[35]) * 180 / M_PI;

  selectPositionSource(trusted_pose_average_rmse_xy, trusted_pose_yaw_rmse_in_degrees);

  if (pose_source_ != 2) {
    new_pose_estimator_pub_->publish(trusted_source_pose_with_cov);
    if (debug_) {
      std_msgs::msg::Float32 out_gnss_rmse;
      out_gnss_rmse.data = trusted_pose_average_rmse_xy;
      out_gnss_position_rmse_pub_->publish(out_gnss_rmse);
    }
  }
}

void AutowarePoseCovarianceModifierNode::selectPositionSource(
  double trusted_pose_average_rmse_xy, double trusted_pose_yaw_rmse_in_degrees)
{
  std_msgs::msg::String selected_pose_type;
  if (
    trusted_pose_average_rmse_xy <= gnss_error_reliable_max_ &&
    trusted_pose_yaw_rmse_in_degrees < yaw_error_deg_threshold_) {
    pose_source_ = static_cast<int>(AutowarePoseCovarianceModifierNode::PoseSource::GNSS);
    selected_pose_type.data = "GNSS";
  } else if (trusted_pose_average_rmse_xy <= gnss_error_unreliable_min_) {
    pose_source_ = static_cast<int>(AutowarePoseCovarianceModifierNode::PoseSource::GNSS_NDT);
    selected_pose_type.data = "GNSS + NDT";
  } else {
    pose_source_ = static_cast<int>(AutowarePoseCovarianceModifierNode::PoseSource::NDT);
    selected_pose_type.data = "NDT";
  }
  pose_source_pub_->publish(selected_pose_type);
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutowarePoseCovarianceModifierNode>());
  rclcpp::shutdown();
  return 0;
}
