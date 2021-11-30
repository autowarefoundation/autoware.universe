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

#pragma once

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/msg/control_command_stamped.hpp"
#include "autoware_control_msgs/msg/gate_mode.hpp"
#include "autoware_control_msgs/msg/emergency_mode.hpp"
#include "autoware_vehicle_msgs/msg/raw_control_command.hpp"
#include "autoware_vehicle_msgs/msg/raw_control_command_stamped.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "raw_vehicle_cmd_converter/accel_map.hpp"
#include "raw_vehicle_cmd_converter/brake_map.hpp"

class RemoteCmdConverter : public rclcpp::Node
{
public:
  RemoteCmdConverter();

private:
  // Publisher
  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::RawControlCommandStamped>::SharedPtr pub_current_cmd_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::RawControlCommandStamped>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr sub_shift_cmd_;
  rclcpp::Subscription<autoware_control_msgs::msg::GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<autoware_control_msgs::msg::EmergencyMode>::SharedPtr sub_emergency_;

  void onVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void onRemoteCmd(const autoware_vehicle_msgs::msg::RawControlCommandStamped::ConstSharedPtr remote_cmd_ptr);
  void onShiftCmd(const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);
  void onGateMode(const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onEmergency(const autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr msg);

  std::shared_ptr<double> current_velocity_ptr_;  // [m/s]
  std::shared_ptr<rclcpp::Time> latest_cmd_received_time_;
  autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr current_shift_cmd_;
  autoware_control_msgs::msg::GateMode::ConstSharedPtr current_gate_mode_;
  bool current_emergency_cmd_ = false;

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr rate_check_timer_;

  // Parameter
  double ref_vel_gain_;  // reference velocity = current velocity + desired acceleration * gain
  double time_threshold_;

  // Diagnostics
  diagnostic_updater::Updater updater_;

  void checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkEmergency(diagnostic_updater::DiagnosticStatusWrapper & stat);
  bool checkRemoteTopicRate();

  // Algorithm
  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool acc_map_initialized_;

  double calculateAcc(const autoware_vehicle_msgs::msg::RawControlCommand & cmd, const double vel);
  double getShiftVelocitySign(const autoware_vehicle_msgs::msg::ShiftStamped & cmd);
};
