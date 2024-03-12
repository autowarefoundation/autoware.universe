// Copyright 2024 TIER IV, Inc.
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

#include "emergency_stop_operator.hpp"

namespace emergency_handler::emergency_stop_operator
{

EmergencyStopOperator::EmergencyStopOperator(rclcpp::Node * node) : node_(node)
{
  // Parameter
  params_.target_acceleration =
    node_->declare_parameter<double>("emergency_stop_operator.target_acceleration");
  params_.target_jerk = node_->declare_parameter<double>("emergency_stop_operator.target_jerk");
  params_.turning_hazard_on = node_->declare_parameter<bool>("emergency_stop_operator.turning_hazard_on");
  params_.use_parking_after_stopped = node_->declare_parameter<bool>("emergency_stop_operator.use_parking_after_stopped");

  // Subscriber
  sub_control_cmd_ = node_->create_subscription<AckermannControlCommand>(
    "~/input/control/control_cmd", 1,
    std::bind(&EmergencyStopOperator::onControlCommand, this, std::placeholders::_1));
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&EmergencyStopOperator::onOdometry, this, std::placeholders::_1));

  // Publisher
  pub_control_cmd_ =
    node_->create_publisher<AckermannControlCommand>("~/output/mrm/emergency_stop/control_cmd", rclcpp::QoS{1});
  pub_hazard_light_cmd_ = node_->create_publisher<HazardLightsCommand>("~/output/hazard", rclcpp::QoS{1});
  pub_gear_cmd_ = node_->create_publisher<GearCommand>("~/output/gear", rclcpp::QoS{1});

  // Initialize
  is_prev_control_cmd_subscribed_ = false;
}

void EmergencyStopOperator::onControlCommand(AckermannControlCommand::ConstSharedPtr msg)
{
  prev_control_cmd_ = *msg;
  is_prev_control_cmd_subscribed_ = true;
}

void EmergencyStopOperator::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  constexpr auto th_stopped_velocity = 0.001;
  is_stopped_ = msg->twist.twist.linear.x < th_stopped_velocity;
}

void EmergencyStopOperator::onTimer()
{
  publishControlCmd();
  publishHazardLightCmd();
  publishGearCmd();
}

bool EmergencyStopOperator::operate()
{
  publishControlCmd();
  publishHazardLightCmd();
  publishGearCmd();

  // Currently, EmergencyStopOperator does not return false
  return true;
}

bool EmergencyStopOperator::cancel()
{
  // do nothing

  // Currently, EmergencyStopOperator does not return false
  return true;
}

void EmergencyStopOperator::publishControlCmd()
{
  AckermannControlCommand control_cmd;
  auto now = node_->now();

  if (!is_prev_control_cmd_subscribed_) {
    control_cmd = AckermannControlCommand();
    control_cmd.stamp = now;

    // longitudinal
    control_cmd.longitudinal.stamp = now;
    control_cmd.longitudinal.speed = 0.0;
    control_cmd.longitudinal.acceleration = static_cast<float>(params_.target_acceleration);
    control_cmd.longitudinal.jerk = 0.0;

    // lateral
    control_cmd.lateral.stamp = now;
    control_cmd.lateral.steering_tire_angle = 0.0;
    control_cmd.lateral.steering_tire_rotation_rate = 0.0;
  } else {
    control_cmd = prev_control_cmd_;
    control_cmd.stamp = now;

    // longitudinal
    const auto dt = (now - prev_control_cmd_.stamp).seconds();
    control_cmd.longitudinal.stamp = now;
    control_cmd.longitudinal.speed = static_cast<float>(std::max(
      prev_control_cmd_.longitudinal.speed + prev_control_cmd_.longitudinal.acceleration * dt,
      0.0));
    control_cmd.longitudinal.acceleration = static_cast<float>(std::max(
      prev_control_cmd_.longitudinal.acceleration + params_.target_jerk * dt,
      params_.target_acceleration));
    if (prev_control_cmd_.longitudinal.acceleration == params_.target_acceleration) {
      control_cmd.longitudinal.jerk = 0.0;
    } else {
      control_cmd.longitudinal.jerk = static_cast<float>(params_.target_jerk);
    }

    // lateral: keep previous lateral command
  }

  pub_control_cmd_->publish(control_cmd);
}

void EmergencyStopOperator::publishHazardLightCmd()
{
  HazardLightsCommand hazard_light_cmd;
  hazard_light_cmd.stamp = node_->now();
  if (params_.turning_hazard_on) {
    hazard_light_cmd.command = HazardLightsCommand::ENABLE;
  } else {
    hazard_light_cmd.command = HazardLightsCommand::NO_COMMAND;
  }
  pub_hazard_light_cmd_->publish(hazard_light_cmd);
}

void EmergencyStopOperator::publishGearCmd()
{
  GearCommand gear_cmd;
  gear_cmd.stamp = node_->now();
  if (params_.use_parking_after_stopped && is_stopped_) {
    gear_cmd.command = GearCommand::PARK;
  } else {
    gear_cmd.command = GearCommand::DRIVE;
  }
  pub_gear_cmd_->publish(gear_cmd);
}

}  // namespace emergency_handler::emergency_stop_operator
