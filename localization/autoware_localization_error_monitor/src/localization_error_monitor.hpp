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

#ifndef LOCALIZATION_ERROR_MONITOR_HPP_
#define LOCALIZATION_ERROR_MONITOR_HPP_

#include "autoware/localization_util/covariance_ellipse.hpp"
#include "autoware/universe_utils/ros/diagnostics_interface.hpp"

#include <Eigen/Dense>
#include <autoware/universe_utils/ros/logger_level_configure.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

namespace autoware::localization_error_monitor
{
class LocalizationErrorMonitor : public rclcpp::Node
{
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ellipse_marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware::universe_utils::DiagnosticsInterface> diagnostics_error_monitor_;

  double scale_;
  double error_ellipse_size_;
  double warn_ellipse_size_;
  double error_ellipse_size_lateral_direction_;
  double warn_ellipse_size_lateral_direction_;
  autoware::localization_util::Ellipse ellipse_;

  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr input_msg);

public:
  explicit LocalizationErrorMonitor(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::localization_error_monitor

#endif  // LOCALIZATION_ERROR_MONITOR_HPP_
