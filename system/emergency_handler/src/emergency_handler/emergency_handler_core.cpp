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

#include <memory>
#include <string>
#include <utility>

#include "emergency_handler/emergency_handler_core.hpp"

EmergencyHandler::EmergencyHandler()
: Node("emergency_handler")
{
  // Parameter
  update_rate_ = declare_parameter<int>("update_rate", 10);
  use_parking_after_stopped_ = declare_parameter<bool>("use_parking_after_stopped", false);
  turning_hazard_on_.emergency = declare_parameter<bool>("turning_hazard_on.emergency", true);

  using std::placeholders::_1;

  // Subscriber
  sub_hazard_status_stamped_ = create_subscription<autoware_system_msgs::msg::HazardStatusStamped>(
    "~/input/hazard_status", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onHazardStatusStamped, this, _1));
  sub_prev_control_command_ = create_subscription<autoware_vehicle_msgs::msg::VehicleCommand>(
    "~/input/prev_control_command", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onPrevControlCommand, this, _1));
  sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/input/twist", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onTwist, this, _1));

  // Heartbeat
  const double timeout_hazard_status = declare_parameter<double>("timeout_hazard_status", 0.5);
  heartbeat_hazard_status_ =
    std::make_shared<HeaderlessHeartbeatChecker<autoware_system_msgs::msg::HazardStatusStamped>>(
    *this, "~/input/hazard_status", timeout_hazard_status);

  // Publisher
  pub_control_command_ = create_publisher<autoware_control_msgs::msg::ControlCommandStamped>(
    "~/output/control_command", rclcpp::QoS{1});
  pub_shift_ = create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>(
    "~/output/shift", rclcpp::QoS{1});
  pub_turn_signal_ = create_publisher<autoware_vehicle_msgs::msg::TurnSignal>(
    "~/output/turn_signal", rclcpp::QoS{1});
  pub_is_emergency_ = create_publisher<autoware_control_msgs::msg::EmergencyMode>(
    "~/output/is_emergency", rclcpp::QoS{1});

  // Initialize
  twist_ = std::make_shared<const geometry_msgs::msg::TwistStamped>();
  prev_control_command_ = autoware_control_msgs::msg::ControlCommand::ConstSharedPtr(
    new autoware_control_msgs::msg::ControlCommand);

  // Timer
  auto timer_callback = std::bind(&EmergencyHandler::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void EmergencyHandler::onHazardStatusStamped(
  const autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  hazard_status_stamped_ = msg;
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

void EmergencyHandler::onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  twist_ = msg;
}

autoware_vehicle_msgs::msg::TurnSignal EmergencyHandler::createTurnSignalMsg()
{
  autoware_vehicle_msgs::msg::TurnSignal msg;
  msg.header.stamp = this->now();

  using autoware_vehicle_msgs::msg::TurnSignal;
  if (hazard_status_stamped_->status.emergency_holding) {
    // turn hazard on during emergency holding
    msg.data = TurnSignal::HAZARD;
  } else if (is_emergency_ && turning_hazard_on_.emergency) {
    // turn hazard on if vehicle is in emergency state and
    // turning hazard on if emergency flag is true
    msg.data = TurnSignal::HAZARD;
  } else {
    msg.data = TurnSignal::NONE;
  }

  return msg;
}

void EmergencyHandler::publishControlCommands()
{
  // Create timestamp
  const auto stamp = this->now();

  // Publish ControlCommand
  {
    autoware_control_msgs::msg::ControlCommandStamped msg;
    msg.header.stamp = stamp;
    msg.control = selectAlternativeControlCommand();
    pub_control_command_->publish(msg);
  }

  // Publish TurnSignal
  pub_turn_signal_->publish(createTurnSignalMsg());

  // Publish Shift
  if (use_parking_after_stopped_ && isStopped()) {
    autoware_vehicle_msgs::msg::ShiftStamped msg;
    msg.header.stamp = stamp;
    msg.shift.data = autoware_vehicle_msgs::msg::Shift::PARKING;
    pub_shift_->publish(msg);
  }

  // Publish Emergency State
  {
    autoware_control_msgs::msg::EmergencyMode emergency_mode;
    emergency_mode.is_emergency = is_emergency_;
    pub_is_emergency_->publish(emergency_mode);
  }
}

bool EmergencyHandler::isDataReady()
{
  if (!hazard_status_stamped_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for hazard_status_stamped msg...");
    return false;
  }

  return true;
}

void EmergencyHandler::onTimer()
{
  if (!isDataReady()) {
    return;
  }
  if (heartbeat_hazard_status_->isTimeout()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "heartbeat_hazard_status is timeout");
    is_emergency_ = true;
    publishControlCommands();
    return;
  }

  // Create msg of is_emergency
  is_emergency_ = isEmergency(hazard_status_stamped_->status);
  publishControlCommands();
}

bool EmergencyHandler::isEmergency(
  const autoware_system_msgs::msg::HazardStatus & hazard_status)
{
  return hazard_status.emergency || hazard_status.emergency_holding;
}

bool EmergencyHandler::isStopped()
{
  constexpr auto th_stopped_velocity = 0.001;
  if (twist_->twist.linear.x < th_stopped_velocity) {
    return true;
  }

  return false;
}

autoware_control_msgs::msg::ControlCommand EmergencyHandler::selectAlternativeControlCommand()
{
  // TODO(jilaada): Add safe_stop planner

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
