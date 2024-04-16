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
: Node("AutowarePoseCovarianceModifierNode"),
  trustedPoseLastReceivedTime_(this->now()),
  pose_source_(AutowarePoseCovarianceModifierNode::PoseSource::NDT)
{
  gnss_error_reliable_max_ =
    this->declare_parameter<double>("error_thresholds.gnss_error_reliable_max", 0.10);
  gnss_error_unreliable_min_ =
    this->declare_parameter<double>("error_thresholds.gnss_error_unreliable_min", 0.25);
  yaw_error_deg_threshold_ =
    this->declare_parameter<double>("error_thresholds.yaw_error_deg_threshold", 0.3);
  trusted_pose_timeout_sec_ = this->declare_parameter<double>("trusted_pose_timeout_sec", 1.0);
  debug_ = this->declare_parameter<bool>("debug.enable_debug_topics", false);

  trusted_pose_with_cov_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "input_trusted_pose_with_cov_topic", 10,
      std::bind(
        &AutowarePoseCovarianceModifierNode::trusted_pose_with_cov_callback, this,
        std::placeholders::_1));

  ndt_pose_with_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input_ndt_pose_with_cov_topic", 10,
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
void AutowarePoseCovarianceModifierNode::check_trusted_pose_timeout()
{
  auto time_diff = this->now() - trustedPoseLastReceivedTime_;
  if (time_diff.seconds() > trusted_pose_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "Trusted Pose Timeout");
    pose_source_ = AutowarePoseCovarianceModifierNode::PoseSource::NDT;
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
   * ndt_rmse = (ndt_max_rmse_meters - gnss_rmse) * (ndt_min_rmse_meters) / 0.1
   */

  ndt_covariance[X_POS_IDX_] = std::pow(
    ((std::sqrt(in_ndt_covariance[X_POS_IDX_]) * 2) -
     std::sqrt(trusted_source_pose_with_cov.pose.covariance[X_POS_IDX_])) *
      (std::sqrt(in_ndt_covariance[X_POS_IDX_])) / 0.1,
    2);
  ndt_covariance[Y_POS_IDX_] = std::pow(
    ((std::sqrt(in_ndt_covariance[Y_POS_IDX_]) * 2) -
     std::sqrt(trusted_source_pose_with_cov.pose.covariance[Y_POS_IDX_])) *
      (std::sqrt(in_ndt_covariance[Y_POS_IDX_])) / 0.1,
    2);
  ndt_covariance[Z_POS_IDX_] = std::pow(
    ((std::sqrt(in_ndt_covariance[Z_POS_IDX_]) * 2) -
     std::sqrt(trusted_source_pose_with_cov.pose.covariance[Z_POS_IDX_])) *
      (std::sqrt(in_ndt_covariance[Z_POS_IDX_])) / 0.1,
    2);

  ndt_covariance[X_POS_IDX_] = std::max(ndt_covariance[X_POS_IDX_], in_ndt_covariance[X_POS_IDX_]);
  ndt_covariance[Y_POS_IDX_] = std::max(ndt_covariance[Y_POS_IDX_], in_ndt_covariance[Y_POS_IDX_]);
  ndt_covariance[Z_POS_IDX_] = std::max(ndt_covariance[Z_POS_IDX_], in_ndt_covariance[Z_POS_IDX_]);

  return ndt_covariance;
}
void AutowarePoseCovarianceModifierNode::ndt_pose_with_cov_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  AutowarePoseCovarianceModifierNode::check_trusted_pose_timeout();

  std::array<double, 36> ndt_covariance_in = msg->pose.covariance;
  std::array<double, 36> out_ndt_covariance =
    pose_source_ == AutowarePoseCovarianceModifierNode::PoseSource::GNSS_NDT
      ? ndt_covariance_modifier(ndt_covariance_in)
      : ndt_covariance_in;

  geometry_msgs::msg::PoseWithCovarianceStamped ndt_pose_with_covariance = *msg;
  ndt_pose_with_covariance.pose.covariance = out_ndt_covariance;
  if (pose_source_ != AutowarePoseCovarianceModifierNode::PoseSource::GNSS) {
    new_pose_estimator_pub_->publish(ndt_pose_with_covariance);
    if (debug_) {
      std_msgs::msg::Float32 out_ndt_rmse;

      out_ndt_rmse.data = static_cast<float>(
        (std::sqrt(ndt_pose_with_covariance.pose.covariance[X_POS_IDX_]) +
         std::sqrt(ndt_pose_with_covariance.pose.covariance[Y_POS_IDX_])) /
        2);
      out_ndt_position_rmse_pub_->publish(out_ndt_rmse);
    }
  }
}
void AutowarePoseCovarianceModifierNode::trusted_pose_with_cov_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  trustedPoseLastReceivedTime_ = this->now();
  trusted_source_pose_with_cov = *msg;

  double trusted_pose_average_rmse_xy =
    (std::sqrt(trusted_source_pose_with_cov.pose.covariance[X_POS_IDX_]) +
     std::sqrt(trusted_source_pose_with_cov.pose.covariance[Y_POS_IDX_])) /
    2;

  double trusted_pose_rmse_z = std::sqrt(trusted_source_pose_with_cov.pose.covariance[Z_POS_IDX_]);
  double trusted_pose_yaw_rmse_in_degrees =
    std::sqrt(trusted_source_pose_with_cov.pose.covariance[YAW_POS_IDX_]) * 180 / M_PI;

  update_pose_source_based_on_rmse(
    trusted_pose_average_rmse_xy, trusted_pose_rmse_z, trusted_pose_yaw_rmse_in_degrees);

  if (pose_source_ != AutowarePoseCovarianceModifierNode::PoseSource::NDT) {
    new_pose_estimator_pub_->publish(trusted_source_pose_with_cov);
    if (debug_) {
      std_msgs::msg::Float32 out_gnss_rmse;
      out_gnss_rmse.data = static_cast<float>(trusted_pose_average_rmse_xy);
      out_gnss_position_rmse_pub_->publish(out_gnss_rmse);
    }
  }
}

void AutowarePoseCovarianceModifierNode::update_pose_source_based_on_rmse(
  double trusted_pose_average_rmse_xy, double trusted_pose_rmse_z,
  double trusted_pose_yaw_rmse_in_degrees)
{
  std_msgs::msg::String selected_pose_type;
  if (
    trusted_pose_average_rmse_xy <= gnss_error_reliable_max_ &&
    trusted_pose_yaw_rmse_in_degrees < yaw_error_deg_threshold_ &&
    trusted_pose_rmse_z < gnss_error_reliable_max_) {
    pose_source_ = AutowarePoseCovarianceModifierNode::PoseSource::GNSS;
    selected_pose_type.data = "GNSS";
  } else if (
    trusted_pose_average_rmse_xy <= gnss_error_unreliable_min_ &&
    trusted_pose_rmse_z < gnss_error_reliable_max_) {
    pose_source_ = AutowarePoseCovarianceModifierNode::PoseSource::GNSS_NDT;
    selected_pose_type.data = "GNSS + NDT";
  } else {
    pose_source_ = AutowarePoseCovarianceModifierNode::PoseSource::NDT;
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
