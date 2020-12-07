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

#ifndef EMERGENCY_HANDLER_EMERGENCY_HANDLER_CORE_HPP_
#define EMERGENCY_HANDLER_EMERGENCY_HANDLER_CORE_HPP_

// Autoware
#include "autoware_control_msgs/msg/control_command_stamped.hpp"
#include "autoware_control_msgs/msg/gate_mode.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_system_msgs/msg/driving_capability.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"

// ROS2 core
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

// Core
#include <string>

class EmergencyHandler : public rclcpp::Node
{
public:
  EmergencyHandler();

private:
  // Subscribers
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr sub_autoware_state_;
  rclcpp::Subscription<autoware_system_msgs::msg::DrivingCapability>::SharedPtr
    sub_driving_capability_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr
    sub_prev_control_command_;
  rclcpp::Subscription<autoware_control_msgs::msg::GateMode>::SharedPtr sub_current_gate_mode_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;

  autoware_system_msgs::msg::AutowareState::ConstSharedPtr autoware_state_;
  autoware_system_msgs::msg::DrivingCapability::ConstSharedPtr driving_capability_;
  autoware_control_msgs::msg::ControlCommand::ConstSharedPtr prev_control_command_;
  autoware_control_msgs::msg::GateMode::ConstSharedPtr current_gate_mode_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;

  void onAutowareState(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  void onDrivingCapability(const autoware_system_msgs::msg::DrivingCapability::ConstSharedPtr msg);
  // To be replaced by ControlCommand
  void onPrevControlCommand(const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg);
  void onCurrentGateMode(const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    pub_control_command_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr pub_shift_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr pub_turn_signal_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_is_emergency_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  int update_rate_;
  bool use_parking_after_stopped_;

  bool isDataReady();
  void onTimer();

  // Algorithm
  bool isStopped();
  bool isEmergency();
  autoware_control_msgs::msg::ControlCommand selectAlternativeControlCommand();
};

#endif
