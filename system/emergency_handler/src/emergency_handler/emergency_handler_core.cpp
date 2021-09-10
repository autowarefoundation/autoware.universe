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
  param_.update_rate = declare_parameter<int>("update_rate", 10);
  param_.timeout_hazard_status = declare_parameter<double>("timeout_hazard_status", 0.5);
  param_.timeout_takeover_request = declare_parameter<double>("timeout_takeover_request", 10.0);
  param_.use_takeover_request = declare_parameter<bool>("use_takeover_request", false);
  param_.use_parking_after_stopped = declare_parameter<bool>("use_parking_after_stopped", false);
  param_.turning_hazard_on.emergency = declare_parameter<bool>("turning_hazard_on.emergency", true);

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
  sub_control_mode_ = create_subscription<autoware_vehicle_msgs::msg::ControlMode>(
    "~/input/control_mode", rclcpp::QoS{1}, std::bind(&EmergencyHandler::onControlMode, this, _1));

  // Heartbeat
  heartbeat_hazard_status_ =
    std::make_shared<HeaderlessHeartbeatChecker<autoware_system_msgs::msg::HazardStatusStamped>>(
    *this, "~/input/hazard_status", param_.timeout_hazard_status);

  // Publisher
  pub_control_command_ = create_publisher<autoware_control_msgs::msg::ControlCommandStamped>(
    "~/output/control_command", rclcpp::QoS{1});
  pub_shift_ = create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>(
    "~/output/shift", rclcpp::QoS{1});
  pub_turn_signal_ = create_publisher<autoware_vehicle_msgs::msg::TurnSignal>(
    "~/output/turn_signal", rclcpp::QoS{1});
  pub_emergency_state_ = create_publisher<autoware_system_msgs::msg::EmergencyStateStamped>(
    "~/output/emergency_state", rclcpp::QoS{1});

  // Initialize
  twist_ = std::make_shared<const geometry_msgs::msg::TwistStamped>();
  prev_control_command_ = autoware_control_msgs::msg::ControlCommand::ConstSharedPtr(
    new autoware_control_msgs::msg::ControlCommand);

  // Timer
  auto timer_callback = std::bind(&EmergencyHandler::onTimer, this);
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), update_period_ns, std::move(timer_callback),
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

void EmergencyHandler::onControlMode(
  const autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr msg)
{
  control_mode_ = msg;
}

autoware_vehicle_msgs::msg::TurnSignal EmergencyHandler::createTurnSignalMsg()
{
  autoware_vehicle_msgs::msg::TurnSignal msg;
  msg.header.stamp = this->now();


  // Check emergency
  const bool is_emergency = isEmergency(hazard_status_stamped_->status);

  using autoware_vehicle_msgs::msg::TurnSignal;
  if (hazard_status_stamped_->status.emergency_holding) {
    // turn hazard on during emergency holding
    msg.data = TurnSignal::HAZARD;
  } else if (is_emergency && param_.turning_hazard_on.emergency) {
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
  if (param_.use_parking_after_stopped && isStopped()) {
    autoware_vehicle_msgs::msg::ShiftStamped msg;
    msg.header.stamp = stamp;
    msg.shift.data = autoware_vehicle_msgs::msg::Shift::PARKING;
    pub_shift_->publish(msg);
  }

  // Publish Emergency State
  {
    autoware_system_msgs::msg::EmergencyStateStamped emergency_state;
    emergency_state.state.state = emergency_state_;
    pub_emergency_state_->publish(emergency_state);
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
    emergency_state_ = autoware_system_msgs::msg::EmergencyState::MRM_OPERATING;
    publishControlCommands();
    return;
  }

  // Update Emergency State
  updateEmergencyState();

  // Publish control commands
  publishControlCommands();
}

void EmergencyHandler::transitionTo(const int new_state)
{
  using autoware_system_msgs::msg::EmergencyState;

  const auto state2string = [](const int state) {
      if (state == EmergencyState::NORMAL) {return "NORMAL";}
      if (state == EmergencyState::OVERRIDE_REQUESTING) {return "OVERRIDE_REQUESTING";}
      if (state == EmergencyState::MRM_OPERATING) {return "MRM_OPERATING";}
      if (state == EmergencyState::MRM_SUCCEEDED) {return "MRM_SUCCEEDED";}
      if (state == EmergencyState::MRM_FAILED) {return "MRM_FAILED";}

      const auto msg = "invalid state: " + std::to_string(state);
      throw std::runtime_error(msg);
    };

  RCLCPP_INFO(
    this->get_logger(), "EmergencyState changed: %s -> %s",
    state2string(emergency_state_), state2string(new_state));

  emergency_state_ = new_state;
}

void EmergencyHandler::updateEmergencyState()
{
  using autoware_vehicle_msgs::msg::ControlMode;
  using autoware_system_msgs::msg::EmergencyState;

  // Check emergency
  const bool is_emergency = isEmergency(hazard_status_stamped_->status);

  // Get mode
  const bool is_auto_mode = control_mode_->data == ControlMode::AUTO;
  const bool is_takeover_done = control_mode_->data == ControlMode::MANUAL;

  // State Machine
  if (emergency_state_ == EmergencyState::NORMAL) {
    // NORMAL
    if (is_auto_mode && is_emergency) {
      if (param_.use_takeover_request) {
        takeover_requested_time_ = this->get_clock()->now();
        transitionTo(EmergencyState::OVERRIDE_REQUESTING);
        return;
      } else {
        transitionTo(EmergencyState::MRM_OPERATING);
        return;
      }
    }
  } else {
    // Emergency
    // Send recovery events if "not emergency" or "takeover done"
    if (!is_emergency) {
      transitionTo(EmergencyState::NORMAL);
      return;
    }
    // TODO(Kenji Miyake): Check if human can safely override, for example using DSM
    if (is_takeover_done) {
      transitionTo(EmergencyState::NORMAL);
      return;
    }

    if (emergency_state_ == EmergencyState::OVERRIDE_REQUESTING) {
      const auto time_from_takeover_request = this->get_clock()->now() - takeover_requested_time_;
      if (time_from_takeover_request.seconds() > param_.timeout_takeover_request) {
        transitionTo(EmergencyState::MRM_OPERATING);
        return;
      }
    } else if (emergency_state_ == EmergencyState::MRM_OPERATING) {
      // TODO(Kenji Miyake): Check MRC is accomplished
      if (isStopped()) {
        transitionTo(EmergencyState::MRM_SUCCEEDED);
        return;
      }
    } else if (emergency_state_ == EmergencyState::MRM_SUCCEEDED) {
      // Do nothing(only checking common recovery events)
    } else if (emergency_state_ == EmergencyState::MRM_FAILED) {
      // Do nothing(only checking common recovery events)
    } else {
      const auto msg = "invalid state: " + std::to_string(emergency_state_);
      throw std::runtime_error(msg);
    }
  }
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
