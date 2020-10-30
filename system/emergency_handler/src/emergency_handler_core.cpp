/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "emergency_handler/emergency_handler_core.hpp"

void EmergencyHandler::onAutowareState(
  const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg)
{
  autoware_state_ = msg;
}

void EmergencyHandler::onDrivingCapability(
  const autoware_system_msgs::msg::DrivingCapability::ConstSharedPtr msg)
{
  driving_capability_ = msg;
}

// To be replaced by ControlCommand
void EmergencyHandler::onPrevControlCommand(
  const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg)
{
  const auto control_command = new autoware_control_msgs::msg::ControlCommand();
  *control_command = msg->control;
  prev_control_command_ =
    autoware_control_msgs::msg::ControlCommand::ConstSharedPtr(control_command);
}

void EmergencyHandler::onCurrentGateMode(
  const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  current_gate_mode_ = msg;
}

void EmergencyHandler::onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  twist_ = msg;
}

bool EmergencyHandler::isDataReady()
{
  if (!driving_capability_) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for driving_capability msg...");
    return false;
  }

  if (!prev_control_command_) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for prev_control_command msg...");
    return false;
  }

  if (!current_gate_mode_) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current_gate_mode msg...");
    return false;
  }

  if (!twist_) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for twist msg...");
    return false;
  }

  return true;
}

void EmergencyHandler::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  // Create timestamp
  const auto stamp = this->get_clock()->now();

  // Check if emergency
  {
    std_msgs::msg::Bool is_emergency;
    is_emergency.data = isEmergency();
    pub_is_emergency_->publish(is_emergency);
  }

  // Select ControlCommand
  autoware_control_msgs::msg::ControlCommandStamped emergency_control_command;
  emergency_control_command.header.stamp = stamp;
  emergency_control_command.control = selectAlternativeControlCommand();
  pub_control_command_->publish(emergency_control_command);

  // TurnSignal
  {
    autoware_vehicle_msgs::msg::TurnSignal turn_signal;
    turn_signal.header.stamp = stamp;
    turn_signal.data = autoware_vehicle_msgs::msg::TurnSignal::HAZARD;
    pub_turn_signal_->publish(turn_signal);
  }

  // Shift
  if (use_parking_after_stopped_ && isStopped()) {
    autoware_vehicle_msgs::msg::ShiftStamped shift;
    shift.header.stamp = stamp;
    shift.shift.data = autoware_vehicle_msgs::msg::Shift::PARKING;
    pub_shift_->publish(shift);
  }
}

bool EmergencyHandler::isStopped()
{
  constexpr auto th_stopped_velocity = 0.1;
  if (twist_->twist.linear.x < th_stopped_velocity) {
    return true;
  }

  return false;
}

bool EmergencyHandler::isEmergency()
{
  using autoware_control_msgs::msg::GateMode;
  using autoware_system_msgs::msg::AutowareState;

  if (current_gate_mode_->data == GateMode::AUTO) {
    const auto is_in_target_state =
      (autoware_state_->state != AutowareState::INITIALIZING_VEHICLE) &&
      (autoware_state_->state != AutowareState::WAITING_FOR_ROUTE) &&
      (autoware_state_->state != AutowareState::PLANNING);

    if (is_in_target_state && !driving_capability_->autonomous_driving) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "autonomous_driving is failed");
      return true;
    }
  }

  if (current_gate_mode_->data == GateMode::REMOTE) {
    if (!driving_capability_->remote_control) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "remote_control is failed");
      return true;
    }
  }

  if (!driving_capability_->manual_driving) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "manual_driving is failed");
    return true;
  }

  /* Currently not supported */
  // if (!driving_capability_->safe_stop) {
  //   RCLCPP_WARN_THROTTLE(
  //     this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
  //     "safe_stop is failed");
  //   return true;
  // }
  /* Currently not supported */

  if (!driving_capability_->emergency_stop) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "emergency_stop is failed");
    return true;
  }

  return false;
}

autoware_control_msgs::msg::ControlCommand EmergencyHandler::selectAlternativeControlCommand()
{
  /* Currently not supported */
  // Safe Stop
  // if (driving_capability_->safe_stop) {
  // TODO: Add safe_stop planner
  // }
  /* Currently not supported */

  // Emergency Stop
  {
    autoware_control_msgs::msg::ControlCommand emergency_stop_cmd;
    emergency_stop_cmd.steering_angle = prev_control_command_->steering_angle;
    emergency_stop_cmd.steering_angle_velocity = prev_control_command_->steering_angle_velocity;
    emergency_stop_cmd.velocity = 0.0;
    emergency_stop_cmd.acceleration = -2.5;

    return emergency_stop_cmd;
  }
}

EmergencyHandler::EmergencyHandler()
: Node("emergency_handler"),
  update_rate_(declare_parameter<int>("update_rate", 10)),
  use_parking_after_stopped_(declare_parameter<bool>("use_parking_after_stopped", false))
{
  // Subscriber
  sub_autoware_state_ = create_subscription<autoware_system_msgs::msg::AutowareState>(
    "input/autoware_state", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onAutowareState, this, std::placeholders::_1));
  sub_driving_capability_ = create_subscription<autoware_system_msgs::msg::DrivingCapability>(
    "input/driving_capability", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onDrivingCapability, this, std::placeholders::_1));
  sub_prev_control_command_ = create_subscription<autoware_vehicle_msgs::msg::VehicleCommand>(
    "input/prev_control_command", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onPrevControlCommand, this, std::placeholders::_1));
  sub_current_gate_mode_ = create_subscription<autoware_control_msgs::msg::GateMode>(
    "input/current_gate_mode", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onCurrentGateMode, this, std::placeholders::_1));
  sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onTwist, this, std::placeholders::_1));

  // Publisher
  pub_control_command_ = create_publisher<autoware_control_msgs::msg::ControlCommandStamped>(
    "output/control_command", rclcpp::QoS{1});
  pub_shift_ =
    create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>("output/shift", rclcpp::QoS{1});
  pub_turn_signal_ =
    create_publisher<autoware_vehicle_msgs::msg::TurnSignal>("output/turn_signal", rclcpp::QoS{1});
  pub_is_emergency_ = create_publisher<std_msgs::msg::Bool>("output/is_emergency", rclcpp::QoS{1});

  // Timer
  auto timer_callback = std::bind(&EmergencyHandler::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(update_rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}
