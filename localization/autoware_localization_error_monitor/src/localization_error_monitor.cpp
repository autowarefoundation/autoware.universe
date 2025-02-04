// Copyright 2020 Tier IV, Inc.
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

#include "localization_error_monitor.hpp"

#include "diagnostics_helper.hpp"

#include <Eigen/Dense>

#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::localization_error_monitor
{
LocalizationErrorMonitor::LocalizationErrorMonitor(const rclcpp::NodeOptions & options)
: Node("localization_error_monitor", options)
{
  scale_ = this->declare_parameter<double>("scale");
  error_ellipse_size_ = this->declare_parameter<double>("error_ellipse_size");
  warn_ellipse_size_ = this->declare_parameter<double>("warn_ellipse_size");

  error_ellipse_size_lateral_direction_ =
    this->declare_parameter<double>("error_ellipse_size_lateral_direction");
  warn_ellipse_size_lateral_direction_ =
    this->declare_parameter<double>("warn_ellipse_size_lateral_direction");

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&LocalizationErrorMonitor::on_odom, this, std::placeholders::_1));

  // QoS setup
  rclcpp::QoS durable_qos(1);
  durable_qos.transient_local();  // option for latching
  ellipse_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("debug/ellipse_marker", durable_qos);

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);

  diagnostics_error_monitor_ =
    std::make_unique<autoware::universe_utils::DiagnosticsInterface>(this, "ellipse_error_status");
}

void LocalizationErrorMonitor::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr input_msg)
{
  diagnostics_error_monitor_->clear();

  ellipse_ = autoware::localization_util::calculate_xy_ellipse(input_msg->pose, scale_);

  const auto ellipse_marker = autoware::localization_util::create_ellipse_marker(
    ellipse_, input_msg->header, input_msg->pose);
  ellipse_marker_pub_->publish(ellipse_marker);

  // update localization accuracy diagnostics
  const auto accuracy_status =
    check_localization_accuracy(ellipse_.long_radius, warn_ellipse_size_, error_ellipse_size_);
  diagnostics_error_monitor_->add_key_value("localization_error_ellipse", ellipse_.long_radius);
  diagnostics_error_monitor_->update_level_and_message(
    accuracy_status.level, accuracy_status.message);

  // update lateral direction error diagnostics
  const auto lateral_direction_status = check_localization_accuracy_lateral_direction(
    ellipse_.size_lateral_direction, warn_ellipse_size_lateral_direction_,
    error_ellipse_size_lateral_direction_);
  diagnostics_error_monitor_->add_key_value(
    "localization_error_ellipse_lateral_direction", ellipse_.size_lateral_direction);
  diagnostics_error_monitor_->update_level_and_message(
    lateral_direction_status.level, lateral_direction_status.message);

  diagnostics_error_monitor_->publish(this->now());
}
}  // namespace autoware::localization_error_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::localization_error_monitor::LocalizationErrorMonitor)
