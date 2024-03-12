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

  // Subscriber
  sub_control_cmd_ = node_->create_subscription<AckermannControlCommand>(
    "~/input/control/control_cmd", 1,
    std::bind(&EmergencyStopOperator::onControlCommand, this, std::placeholders::_1));

  // Publisher
  pub_control_cmd_ =
    node_->create_publisher<AckermannControlCommand>("~/output/mrm/emergency_stop/control_cmd", 1);

  // Initialize
  is_prev_control_cmd_subscribed_ = false;
}

void EmergencyStopOperator::onControlCommand(AckermannControlCommand::ConstSharedPtr msg)
{
  prev_control_cmd_ = *msg;
  is_prev_control_cmd_subscribed_ = true;
}

void EmergencyStopOperator::onTimer()
{
  pub_control_cmd_->publish(calcNextControlCmd());
}

bool EmergencyStopOperator::operate()
{
  pub_control_cmd_->publish(calcNextControlCmd());

  // Currently, EmergencyStopOperator does not return false
  return true;
}

bool EmergencyStopOperator::cancel()
{
  // do nothing

  // Currently, EmergencyStopOperator does not return false
  return true;
}

AckermannControlCommand EmergencyStopOperator::calcNextControlCmd()
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

  return control_cmd;
}

}  // namespace emergency_handler::emergency_stop_operator
