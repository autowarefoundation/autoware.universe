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
#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <nav_msgs/msg/odometry.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

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

  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);

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
    p.rolling_back_velocity = declare_parameter<double>(t + "rolling_back_velocity");
    p.over_velocity_offset = declare_parameter<double>(t + "over_velocity_offset");
    p.over_velocity_ratio = declare_parameter<double>(t + "over_velocity_ratio");
  }
  const auto lpf_gain = declare_parameter<double>("vel_lpf_gain");
  vehicle_vel_.setGain(lpf_gain);
  target_vel_.setGain(lpf_gain);

  hold_velocity_error_until_stop_ = declare_parameter<bool>("hold_velocity_error_until_stop");

  try {
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  } catch (...) {
    vehicle_info_.front_overhang_m = 0.5;
    vehicle_info_.wheel_base_m = 4.0;
    RCLCPP_ERROR(
      get_logger(),
      "failed to get vehicle info. use default value. vehicle_info_.front_overhang_m: %.2f, "
      "vehicle_info_.wheel_base_m: %.2f",
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
  d.add(ns + "rolling_back", [&](auto & stat) {
    set_status(
      stat, !validation_status_.is_rolling_back,
      "The vehicle is rolling back. The velocity has the opposite sign to the target.");
  });
  d.add(ns + "over_velocity", [&](auto & stat) {
    set_status(
      stat, !validation_status_.is_over_velocity,
      "The vehicle is over-speeding against the target.");
  });
}

bool ControlValidator::is_data_ready()
{
  const auto waiting = [this](const auto topic_name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", topic_name);
    return false;
  };

  if (!current_kinematics_) {
    return waiting(sub_kinematics_->subscriber()->get_topic_name());
  }
  if (!current_reference_trajectory_) {
    return waiting(sub_reference_traj_->subscriber()->get_topic_name());
  }
  if (!current_predicted_trajectory_) {
    return waiting(sub_predicted_traj_->get_topic_name());
  }
  return true;
}

void ControlValidator::on_predicted_trajectory(const Trajectory::ConstSharedPtr msg)
{
  stop_watch.tic();

  current_predicted_trajectory_ = msg;
  current_reference_trajectory_ = sub_reference_traj_->takeData();
  current_kinematics_ = sub_kinematics_->takeData();

  if (!is_data_ready()) return;

  debug_pose_publisher_->clear_markers();

  validate(*current_predicted_trajectory_, *current_reference_trajectory_, *current_kinematics_);

  diag_updater_.force_update();

  // for debug
  publish_debug_info();
  display_status();
}

void ControlValidator::publish_debug_info()
{
  pub_status_->publish(validation_status_);

  if (!is_all_valid(validation_status_)) {
    geometry_msgs::msg::Pose front_pose = current_kinematics_->pose.pose;
    shift_pose(front_pose, vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m);
    debug_pose_publisher_->push_virtual_wall(front_pose);
    debug_pose_publisher_->push_warning_msg(front_pose, "INVALID CONTROL");
  }
  debug_pose_publisher_->publish();

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

void ControlValidator::validate(
  const Trajectory & predicted_trajectory, const Trajectory & reference_trajectory,
  const Odometry & kinematics)
{
  if (predicted_trajectory.points.size() < 2) {
    RCLCPP_DEBUG(get_logger(), "predicted_trajectory size is less than 2. Cannot validate.");
    return;
  }
  if (reference_trajectory.points.size() < 2) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "reference_trajectory size is less than 2. Cannot validate.");
    return;
  }

  validation_status_.stamp = get_clock()->now();

  std::tie(
    validation_status_.max_distance_deviation, validation_status_.is_valid_max_distance_deviation) =
    calc_lateral_deviation_status(predicted_trajectory, *current_reference_trajectory_);

  calc_velocity_deviation_status(*current_reference_trajectory_, kinematics);

  validation_status_.invalid_count =
    is_all_valid(validation_status_) ? 0 : validation_status_.invalid_count + 1;
}

std::pair<double, bool> ControlValidator::calc_lateral_deviation_status(
  const Trajectory & predicted_trajectory, const Trajectory & reference_trajectory) const
{
  auto max_distance_deviation =
    calc_max_lateral_distance(reference_trajectory, predicted_trajectory);
  return {
    max_distance_deviation,
    max_distance_deviation <= validation_params_.max_distance_deviation_threshold};
}

void ControlValidator::calc_velocity_deviation_status(
  const Trajectory & reference_trajectory, const Odometry & kinematics)
{
  auto & status = validation_status_;
  const auto & params = validation_params_;
  status.vehicle_vel = vehicle_vel_.filter(kinematics.twist.twist.linear.x);
  status.target_vel = target_vel_.filter(
    autoware::motion_utils::calcInterpolatedPoint(reference_trajectory, kinematics.pose.pose)
      .longitudinal_velocity_mps);

  const bool is_rolling_back = std::signbit(status.vehicle_vel * status.target_vel) &&
                               std::abs(status.vehicle_vel) > params.rolling_back_velocity;
  if (
    !hold_velocity_error_until_stop_ || !status.is_rolling_back ||
    std::abs(status.vehicle_vel) < 0.05) {
    status.is_rolling_back = is_rolling_back;
  }

  const bool is_over_velocity =
    std::abs(status.vehicle_vel) >
    std::abs(status.target_vel) * (1.0 + params.over_velocity_ratio) + params.over_velocity_offset;
  if (
    !hold_velocity_error_until_stop_ || !status.is_over_velocity ||
    std::abs(status.vehicle_vel) < 0.05) {
    status.is_over_velocity = is_over_velocity;
  }
}

bool ControlValidator::is_all_valid(const ControlValidatorStatus & s)
{
  return s.is_valid_max_distance_deviation && !s.is_rolling_back && !s.is_over_velocity;
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
