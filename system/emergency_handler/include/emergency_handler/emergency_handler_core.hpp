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

#ifndef EMERGENCY_HANDLER__EMERGENCY_HANDLER_CORE_HPP_
#define EMERGENCY_HANDLER__EMERGENCY_HANDLER_CORE_HPP_

// Core
#include <memory>
#include <string>

// Autoware
#include "autoware_control_msgs/msg/control_command_stamped.hpp"
#include "autoware_control_msgs/msg/emergency_mode.hpp"
#include "autoware_control_msgs/msg/gate_mode.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_system_msgs/msg/driving_capability.hpp"
#include "autoware_system_msgs/msg/hazard_status_stamped.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"

// ROS2 core
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

#include "emergency_handler/util/heartbeat_checker.hpp"

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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_is_state_timeout_;

  autoware_system_msgs::msg::AutowareState::ConstSharedPtr autoware_state_;
  autoware_system_msgs::msg::DrivingCapability::ConstSharedPtr driving_capability_;
  autoware_control_msgs::msg::ControlCommand::ConstSharedPtr prev_control_command_;
  autoware_control_msgs::msg::GateMode::ConstSharedPtr current_gate_mode_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;
  std_msgs::msg::Bool::ConstSharedPtr is_state_timeout_;

  void onAutowareState(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  void onDrivingCapability(const autoware_system_msgs::msg::DrivingCapability::ConstSharedPtr msg);
  // To be replaced by ControlCommand
  void onPrevControlCommand(const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg);
  void onCurrentGateMode(const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void onIsStateTimeout(const std_msgs::msg::Bool::ConstSharedPtr msg);

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_emergency_;

  bool onClearEmergencyService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Publisher
  rclcpp::Publisher<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    pub_control_command_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr pub_shift_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr pub_turn_signal_;
  rclcpp::Publisher<autoware_control_msgs::msg::EmergencyMode>::SharedPtr pub_is_emergency_;
  rclcpp::Publisher<autoware_system_msgs::msg::HazardStatusStamped>::SharedPtr pub_hazard_status_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_err_;

  void publishHazardStatus(const autoware_system_msgs::msg::HazardStatus & hazard_status);
  void publishControlCommands();

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  int update_rate_;
  double data_ready_timeout_;
  double timeout_driving_capability_;
  double timeout_is_state_timeout_;
  int emergency_hazard_level_;
  bool use_emergency_hold_;
  bool use_parking_after_stopped_;

  bool isDataReady();
  void onTimer();

  // Heartbeat
  rclcpp::Time initialized_time_;
  std::shared_ptr<HeaderlessHeartbeatChecker<autoware_system_msgs::msg::DrivingCapability>>
  heartbeat_driving_capability_;
  std::shared_ptr<HeaderlessHeartbeatChecker<std_msgs::msg::Bool>> heartbeat_is_state_timeout_;

  // Algorithm
  bool is_emergency_ = false;
  autoware_system_msgs::msg::HazardStatus hazard_status_;

  bool isStopped();
  bool isEmergency(const autoware_system_msgs::msg::HazardStatus & hazard_status);
  autoware_system_msgs::msg::HazardStatus judgeHazardStatus();
  autoware_control_msgs::msg::ControlCommand selectAlternativeControlCommand();
};

#endif  // EMERGENCY_HANDLER__EMERGENCY_HANDLER_CORE_HPP_
