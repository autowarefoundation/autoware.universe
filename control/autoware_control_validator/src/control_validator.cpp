// Copyright 2023 TIER IV, Inc.
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

#include "autoware/control_validator/control_validator.hpp"

#include "autoware/control_validator/utils.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <autoware/universe_utils/ros/polling_subscriber.hpp>

#include <nav_msgs/msg/detail/odometry__struct.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace autoware::control_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

ControlValidator::ControlValidator(const rclcpp::NodeOptions & options)
: Node("control_validator", options), validation_params_(), vehicle_info_()
{
  using std::placeholders::_1;

  sub_predicted_traj_ = create_subscription<Trajectory>(
    "~/input/predicted_trajectory", 1,
    std::bind(&ControlValidator::on_predicted_trajectory, this, _1));
  sub_kinematics_ =
    universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>::create_subscription(
      this, "~/input/kinematics", 1);
  sub_reference_traj_ =
    autoware::universe_utils::InterProcessPollingSubscriber<Trajectory>::create_subscription(
      this, "~/input/reference_trajectory", 1);

  pub_status_ = create_publisher<ControlValidatorStatus>("~/output/validation_status", 1);

  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);

  debug_pose_publisher_ = std::make_shared<ControlValidatorDebugMarkerPublisher>(this);

  setup_parameters();

  setup_diag();
}

void ControlValidator::setup_parameters()
{
  diag_error_count_threshold_ = declare_parameter<int64_t>("diag_error_count_threshold");
  display_on_terminal_ = declare_parameter<bool>("display_on_terminal");

  {
    auto & p = validation_params_;
    const std::string t = "thresholds.";
    p.max_distance_deviation_threshold = declare_parameter<double>(t + "max_distance_deviation");
  }

  try {
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  } catch (...) {
    vehicle_info_.front_overhang_m = 0.5;
    vehicle_info_.wheel_base_m = 4.0;
    RCLCPP_ERROR(
      get_logger(),
      "failed to get vehicle info. use default value. vehicle_info_.front_overhang_m: %f, "
      "vehicle_info_.wheel_base_m: %f",
      vehicle_info_.front_overhang_m, vehicle_info_.wheel_base_m);
  }
}

void ControlValidator::set_status(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
  } else if (validation_status_.invalid_count < diag_error_count_threshold_) {
    const auto warn_msg = msg + " (invalid count is less than error threshold: " +
                          std::to_string(validation_status_.invalid_count) + " < " +
                          std::to_string(diag_error_count_threshold_) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
  } else {
    stat.summary(DiagnosticStatus::ERROR, msg);
  }
}

void ControlValidator::setup_diag()
{
  auto & d = diag_updater_;
  d.setHardwareID("control_validator");

  std::string ns = "control_validation_";
  d.add(ns + "max_distance_deviation", [&](auto & stat) {
    set_status(
      stat, validation_status_.is_valid_max_distance_deviation,
      "control output is deviated from trajectory");
  });
}

bool ControlValidator::is_data_ready()
{
  const auto waiting = [this](const auto s) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", s);
    return false;
  };

  // TODO(hisaki): change message
  if (!current_kinematics_) {
    return waiting("current_kinematics_");
  }
  if (!current_reference_trajectory_) {
    return waiting("current_reference_trajectory_");
  }
  if (!current_predicted_trajectory_) {
    return waiting("current_predicted_trajectory_");
  }
  return true;
}

void ControlValidator::on_predicted_trajectory(const Trajectory::ConstSharedPtr msg)
{
  current_predicted_trajectory_ = msg;
  current_reference_trajectory_ = sub_reference_traj_->takeData();
  current_kinematics_ = sub_kinematics_->takeData();

  if (!is_data_ready()) return;

  debug_pose_publisher_->clear_markers();

  validate(*current_predicted_trajectory_);

  diag_updater_.force_update();

  // for debug
  publish_debug_info();
  display_status();
}

void ControlValidator::publish_debug_info()
{
  validation_status_.stamp = get_clock()->now();
  pub_status_->publish(validation_status_);

  if (!is_all_valid(validation_status_)) {
    geometry_msgs::msg::Pose front_pose = current_kinematics_->pose.pose;
    shift_pose(front_pose, vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m);
    debug_pose_publisher_->push_virtual_wall(front_pose);
    debug_pose_publisher_->push_warning_msg(front_pose, "INVALID CONTROL");
  }
  debug_pose_publisher_->publish();
}

void ControlValidator::validate(const Trajectory & predicted_trajectory)
{
  if (predicted_trajectory.points.size() < 2) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "predicted_trajectory size is less than 2. Cannot validate.");
    return;
  }

  auto & s = validation_status_;

  s.is_valid_max_distance_deviation =
    check_valid_max_distance_deviation(predicted_trajectory, *current_reference_trajectory_);

  s.invalid_count = is_all_valid(s) ? 0 : s.invalid_count + 1;
}

bool ControlValidator::check_valid_max_distance_deviation(
  const Trajectory & predicted_trajectory, const Trajectory & reference_trajectory)
{
  validation_status_.max_distance_deviation =
    calc_max_lateral_distance(reference_trajectory, predicted_trajectory);
  return validation_status_.max_distance_deviation <=
         validation_params_.max_distance_deviation_threshold;
}

bool ControlValidator::is_all_valid(const ControlValidatorStatus & s)
{
  return s.is_valid_max_distance_deviation;
}

void ControlValidator::display_status()
{
  if (!display_on_terminal_) return;
  rclcpp::Clock clock{RCL_ROS_TIME};

  const auto warn = [this, &clock](const bool status, const std::string & msg) {
    if (!status) {
      RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "%s", msg.c_str());
    }
  };

  const auto & s = validation_status_;

  warn(
    s.is_valid_max_distance_deviation,
    "predicted trajectory is too far from planning trajectory!!");
}

}  // namespace autoware::control_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control_validator::ControlValidator)
