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
  gnss_pose_last_received_time_(this->now()),
  pose_source_(AutowarePoseCovarianceModifierNode::PoseSource::NDT)
{
  try {
    gnss_error_reliable_max_ =
      this->declare_parameter<double>("error_thresholds.gnss_error_reliable_max");
    gnss_error_unreliable_min_ =
      this->declare_parameter<double>("error_thresholds.gnss_error_unreliable_min");
    yaw_error_deg_threshold_ =
      this->declare_parameter<double>("error_thresholds.yaw_error_deg_threshold");
    gnss_pose_timeout_sec_ = this->declare_parameter<double>("gnss_pose_timeout_sec");
    debug_ = this->declare_parameter<bool>("debug.enable_debug_topics");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to declare parameters: %s", e.what());
    throw;
  }
  gnss_pose_with_cov_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "input_gnss_pose_with_cov_topic", 10,
      std::bind(
        &AutowarePoseCovarianceModifierNode::gnss_pose_with_cov_callback, this,
        std::placeholders::_1));

  ndt_pose_with_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input_ndt_pose_with_cov_topic", 10,
    std::bind(
      &AutowarePoseCovarianceModifierNode::ndt_pose_with_cov_callback, this,
      std::placeholders::_1));
  output_pose_with_covariance_stamped_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
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
void AutowarePoseCovarianceModifierNode::check_gnss_pose_timeout()
{
  auto time_diff = this->now() - gnss_pose_last_received_time_;
  if (time_diff.seconds() > gnss_pose_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "GNSS Pose Timeout");
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
   * In the README.md file, "How does the "Interpolate GNSS and NDT pose" part work?" It is
   * explained in the section
   * (https://github.com/meliketanrikulu/autoware.universe/tree/feat/add_autoware_pose_covariance_modifier_node/localization/autoware_pose_covariance_modifier_node#how-does-the-interpolate-gnss-and-ndt-pose-part-work-)
   */
  auto modify_covariance = [&](int idx) {
    // calculate NDT covariance value based on gnss covariance
    double calculated_covariance = std::pow(
      ((std::sqrt(in_ndt_covariance[idx]) * 2) -
       ((std::sqrt(in_ndt_covariance[idx]) * 2) - (std::sqrt(in_ndt_covariance[idx]))) *
         ((std::sqrt(gnss_source_pose_with_cov.pose.covariance[idx]) - gnss_error_reliable_max_) /
          (gnss_error_unreliable_min_ - gnss_error_reliable_max_))),
      2);
    // Make sure the ndt covariance is not below the input ndt covariance value and return
    return (std::max(calculated_covariance, in_ndt_covariance[idx]));
  };
  ndt_covariance[X_POS_IDX_] = modify_covariance(X_POS_IDX_);
  ndt_covariance[Y_POS_IDX_] = modify_covariance(Y_POS_IDX_);
  ndt_covariance[Z_POS_IDX_] = modify_covariance(Z_POS_IDX_);

  return ndt_covariance;
}
void AutowarePoseCovarianceModifierNode::ndt_pose_with_cov_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  AutowarePoseCovarianceModifierNode::check_gnss_pose_timeout();

  std::array<double, 36> ndt_covariance_in = msg->pose.covariance;
  std::array<double, 36> out_ndt_covariance =
    pose_source_ == AutowarePoseCovarianceModifierNode::PoseSource::GNSS_NDT
      ? ndt_covariance_modifier(ndt_covariance_in)
      : ndt_covariance_in;

  geometry_msgs::msg::PoseWithCovarianceStamped ndt_pose_with_covariance = *msg;
  ndt_pose_with_covariance.pose.covariance = out_ndt_covariance;

  if (pose_source_ == AutowarePoseCovarianceModifierNode::PoseSource::GNSS) {
    return;
  }
  output_pose_with_covariance_stamped_pub_->publish(ndt_pose_with_covariance);
  if (debug_) {
    std_msgs::msg::Float32 out_ndt_rmse;
    out_ndt_rmse.data = static_cast<float>(
      (std::sqrt(ndt_pose_with_covariance.pose.covariance[X_POS_IDX_]) +
       std::sqrt(ndt_pose_with_covariance.pose.covariance[Y_POS_IDX_])) /
      2);
    out_ndt_position_rmse_pub_->publish(out_ndt_rmse);
  }
}
void AutowarePoseCovarianceModifierNode::gnss_pose_with_cov_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  gnss_pose_last_received_time_ = this->now();
  gnss_source_pose_with_cov = *msg;

  double gnss_pose_average_rmse_xy =
    (std::sqrt(gnss_source_pose_with_cov.pose.covariance[X_POS_IDX_]) +
     std::sqrt(gnss_source_pose_with_cov.pose.covariance[Y_POS_IDX_])) /
    2;

  double gnss_pose_rmse_z = std::sqrt(gnss_source_pose_with_cov.pose.covariance[Z_POS_IDX_]);
  double gnss_pose_yaw_rmse_in_degrees =
    std::sqrt(gnss_source_pose_with_cov.pose.covariance[YAW_POS_IDX_]) * 180 / M_PI;

  update_pose_source_based_on_rmse(
    gnss_pose_average_rmse_xy, gnss_pose_rmse_z, gnss_pose_yaw_rmse_in_degrees);

  if (pose_source_ == AutowarePoseCovarianceModifierNode::PoseSource::NDT) {
    return;
  }
  output_pose_with_covariance_stamped_pub_->publish(gnss_source_pose_with_cov);
  if (debug_) {
    std_msgs::msg::Float32 out_gnss_rmse;
    out_gnss_rmse.data = static_cast<float>(gnss_pose_average_rmse_xy);
    out_gnss_position_rmse_pub_->publish(out_gnss_rmse);
  }
}

void AutowarePoseCovarianceModifierNode::update_pose_source_based_on_rmse(
  double gnss_pose_average_rmse_xy, double gnss_pose_rmse_z, double gnss_pose_yaw_rmse_in_degrees)
{
  std_msgs::msg::String selected_pose_type;
  if (
    gnss_pose_yaw_rmse_in_degrees <= yaw_error_deg_threshold_ &&
    gnss_pose_rmse_z <= gnss_error_reliable_max_) {
    if (gnss_pose_average_rmse_xy <= gnss_error_reliable_max_) {
      pose_source_ = AutowarePoseCovarianceModifierNode::PoseSource::GNSS;
      selected_pose_type.data = "GNSS";
    } else if (gnss_pose_average_rmse_xy <= gnss_error_unreliable_min_) {
      pose_source_ = AutowarePoseCovarianceModifierNode::PoseSource::GNSS_NDT;
      selected_pose_type.data = "GNSS + NDT";
    } else {
      pose_source_ = AutowarePoseCovarianceModifierNode::PoseSource::NDT;
      selected_pose_type.data = "NDT";
    }
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
