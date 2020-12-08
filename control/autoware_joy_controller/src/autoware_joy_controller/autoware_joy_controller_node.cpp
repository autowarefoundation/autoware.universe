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

#include "autoware_joy_controller/autoware_joy_controller.hpp"
#include "autoware_joy_controller/joy_converter/g29_joy_converter.hpp"
#include "autoware_joy_controller/joy_converter/ds4_joy_converter.hpp"

namespace
{
ShiftType getUpperShift(const ShiftType & shift)
{
  using autoware_vehicle_msgs::msg::Shift;

  if (shift == Shift::NONE) return Shift::PARKING;
  if (shift == Shift::PARKING) return Shift::REVERSE;
  if (shift == Shift::REVERSE) return Shift::NEUTRAL;
  if (shift == Shift::NEUTRAL) return Shift::DRIVE;
  if (shift == Shift::DRIVE) return Shift::LOW;
  if (shift == Shift::LOW) return Shift::LOW;

  return Shift::NONE;
}

ShiftType getLowerShift(const ShiftType & shift)
{
  using autoware_vehicle_msgs::msg::Shift;

  if (shift == Shift::NONE) return Shift::PARKING;
  if (shift == Shift::PARKING) return Shift::PARKING;
  if (shift == Shift::REVERSE) return Shift::PARKING;
  if (shift == Shift::NEUTRAL) return Shift::REVERSE;
  if (shift == Shift::DRIVE) return Shift::NEUTRAL;
  if (shift == Shift::LOW) return Shift::DRIVE;

  return Shift::NONE;
}

const char * getShiftName(const ShiftType & shift)
{
  using autoware_vehicle_msgs::msg::Shift;

  if (shift == Shift::NONE) return "NONE";
  if (shift == Shift::PARKING) return "PARKING";
  if (shift == Shift::REVERSE) return "REVERSE";
  if (shift == Shift::NEUTRAL) return "NEUTRAL";
  if (shift == Shift::DRIVE) return "DRIVE";
  if (shift == Shift::LOW) return "LOW";

  return "NOT_SUPPORTED";
}

const char * getTurnSignalName(const TurnSignalType & turn_signal)
{
  using autoware_vehicle_msgs::msg::TurnSignal;

  if (turn_signal == TurnSignal::NONE) return "NONE";
  if (turn_signal == TurnSignal::LEFT) return "LEFT";
  if (turn_signal == TurnSignal::RIGHT) return "RIGHT";
  if (turn_signal == TurnSignal::HAZARD) return "HAZARD";

  return "NOT_SUPPORTED";
}

const char * getGateModeName(const GateModeType & gate_mode)
{
  using autoware_control_msgs::msg::GateMode;

  if (gate_mode == GateMode::AUTO) return "AUTO";
  if (gate_mode == GateMode::REMOTE) return "REMOTE";

  return "NOT_SUPPORTED";
}

}  // namespace

void AutowareJoyControllerNode::onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  last_joy_received_time_ = msg->header.stamp;
  if (joy_type_ == "G29") {
    joy_ = std::make_shared<const G29JoyConverter>(*msg);
  } else {
    joy_ = std::make_shared<const DS4JoyConverter>(*msg);
  }

  if (joy_->shift_up() || joy_->shift_down() || joy_->shift_drive() || joy_->shift_reverse()) {
    publishShift();
  }

  if (joy_->turn_signal_left() || joy_->turn_signal_right() || joy_->clear_turn_signal()) {
    publishTurnSignal();
  }

  if (joy_->gate_mode()) {
    publishGateMode();
  }

  if (joy_->emergency() || joy_->clear_emergency()) {
    publishEmergency();
  }

  if (joy_->autoware_engage() || joy_->autoware_disengage()) {
    publishAutowareEngage();
  }

  if (joy_->vehicle_engage() || joy_->vehicle_disengage()) {
    publishVehicleEngage();
  }
}

void AutowareJoyControllerNode::onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  twist_ = msg;
}

bool AutowareJoyControllerNode::isDataReady()
{
  // Joy
  {
    if (!joy_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
        "waiting for joy msg...");
      return false;
    }

    constexpr auto timeout = 2.0;
    const auto time_diff = this->now() - last_joy_received_time_;
    if (time_diff.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
        "joy msg is timeout");
      return false;
    }
  }

  // Twist
  {
    if (!twist_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
        "waiting for twist msg...");
      return false;
    }

    constexpr auto timeout = 0.5;
    const auto time_diff = this->now() - twist_->header.stamp;
    if (time_diff.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
        "twist msg is timeout");
      return false;
    }
  }

  return true;
}

void AutowareJoyControllerNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  publishControlCommand();
  publishRawControlCommand();

  // tmp
  publishVehicleCommand();
  publishRawVehicleCommand();
}

void AutowareJoyControllerNode::publishControlCommand()
{
  autoware_control_msgs::msg::ControlCommandStamped cmd_stamped;
  cmd_stamped.header.stamp = this->now();

  {
    auto & cmd = cmd_stamped.control;

    cmd.steering_angle = steer_ratio_ * joy_->steer();
    cmd.steering_angle_velocity = steering_angle_velocity_;

    if (joy_->accel()) {
      cmd.acceleration = accel_ratio_ * joy_->accel();
      cmd.velocity = twist_->twist.linear.x + velocity_gain_ * cmd.acceleration;
      cmd.velocity = std::min(cmd.velocity, max_forward_velocity_);
    }

    if (joy_->brake()) {
      cmd.velocity = 0.0;
      cmd.acceleration = -brake_ratio_ * joy_->brake();
    }

    // Backward
    if (joy_->accel() && joy_->brake()) {
      cmd.acceleration = backward_accel_ratio_ * joy_->accel();
      cmd.velocity = twist_->twist.linear.x - velocity_gain_ * cmd.acceleration;
      cmd.velocity = std::max(cmd.velocity, -max_backward_velocity_);
    }
  }

  pub_control_command_->publish(cmd_stamped);
  prev_control_command_ = cmd_stamped.control;
}

void AutowareJoyControllerNode::publishRawControlCommand()
{
  autoware_vehicle_msgs::msg::RawControlCommandStamped cmd_stamped;
  cmd_stamped.header.stamp = this->now();

  {
    auto & cmd = cmd_stamped.control;

    cmd.steering_angle = steer_ratio_ * joy_->steer();
    cmd.steering_angle_velocity = steering_angle_velocity_;
    cmd.throttle = accel_ratio_ * joy_->accel();
    cmd.brake = brake_ratio_ * joy_->brake();
  }

  pub_raw_control_command_->publish(cmd_stamped);
  prev_raw_control_command_ = cmd_stamped.control;
}

void AutowareJoyControllerNode::publishShift()
{
  using autoware_vehicle_msgs::msg::Shift;

  autoware_vehicle_msgs::msg::ShiftStamped shift_stamped;
  shift_stamped.header.stamp = this->now();

  {
    auto & shift = shift_stamped.shift;
    if (joy_->shift_up()) {
      shift.data = getUpperShift(prev_shift_);
    }

    if (joy_->shift_down()) {
      shift.data = getLowerShift(prev_shift_);
    }

    if (joy_->shift_drive()) {
      shift.data = Shift::DRIVE;
    }

    if (joy_->shift_reverse()) {
      shift.data = Shift::REVERSE;
    }

    RCLCPP_INFO(get_logger(), "Shift::%s", getShiftName(shift.data));
  }

  pub_shift_->publish(shift_stamped);
  prev_shift_ = shift_stamped.shift.data;
}

void AutowareJoyControllerNode::publishTurnSignal()
{
  using autoware_vehicle_msgs::msg::TurnSignal;

  TurnSignal turn_signal;
  turn_signal.header.stamp = this->now();

  if (joy_->turn_signal_left() && joy_->turn_signal_right()) {
    turn_signal.data = TurnSignal::HAZARD;
  } else if (joy_->turn_signal_left()) {
    turn_signal.data = TurnSignal::LEFT;
  } else if (joy_->turn_signal_right()) {
    turn_signal.data = TurnSignal::RIGHT;
  }

  if (joy_->clear_turn_signal()) {
    turn_signal.data = TurnSignal::NONE;
  }

  RCLCPP_INFO(get_logger(), "TurnSignal::%s", getTurnSignalName(turn_signal.data));

  pub_turn_signal_->publish(turn_signal);
}

void AutowareJoyControllerNode::publishGateMode()
{
  using autoware_control_msgs::msg::GateMode;

  autoware_control_msgs::msg::GateMode gate_mode;

  if (prev_gate_mode_ == GateMode::AUTO) {
    gate_mode.data = GateMode::REMOTE;
  }

  if (prev_gate_mode_ == GateMode::REMOTE) {
    gate_mode.data = GateMode::AUTO;
  }

  RCLCPP_INFO(get_logger(), "GateMode::%s", getGateModeName(gate_mode.data));

  pub_gate_mode_->publish(gate_mode);
  prev_gate_mode_ = gate_mode.data;
}

void AutowareJoyControllerNode::publishEmergency()
{
  autoware_debug_msgs::msg::BoolStamped emergency;

  if (joy_->emergency()) {
    emergency.data = true;
    RCLCPP_INFO(get_logger(), "Emergency");
  }

  if (joy_->clear_emergency()) {
    emergency.data = false;
    RCLCPP_INFO(get_logger(), "Clear Emergency");
  }

  pub_emergency_->publish(emergency);
}

void AutowareJoyControllerNode::publishAutowareEngage()
{
  autoware_debug_msgs::msg::BoolStamped engage;

  if (joy_->autoware_engage()) {
    engage.data = true;
    RCLCPP_INFO(get_logger(), "Autoware Engage");
  }

  if (joy_->autoware_disengage()) {
    engage.data = false;
    RCLCPP_INFO(get_logger(), "Autoware Disengage");
  }

  pub_autoware_engage_->publish(engage);
}

void AutowareJoyControllerNode::publishVehicleEngage()
{
  autoware_debug_msgs::msg::BoolStamped engage;

  if (joy_->vehicle_engage()) {
    engage.data = true;
    RCLCPP_INFO(get_logger(), "Vehicle Engage");
  }

  if (joy_->vehicle_disengage()) {
    engage.data = false;
    RCLCPP_INFO(get_logger(), "Vehicle Disengage");
  }

  pub_vehicle_engage_->publish(engage);
}

// tmp
void AutowareJoyControllerNode::publishVehicleCommand()
{
  autoware_vehicle_msgs::msg::VehicleCommand vehicle_cmd;
  vehicle_cmd.header.stamp = this->now();
  vehicle_cmd.control = prev_control_command_;
  vehicle_cmd.shift.data = prev_shift_;

  pub_vehicle_command_->publish(vehicle_cmd);
}

void AutowareJoyControllerNode::publishRawVehicleCommand()
{
  autoware_vehicle_msgs::msg::RawVehicleCommand vehicle_cmd;
  vehicle_cmd.header.stamp = this->now();
  vehicle_cmd.control = prev_raw_control_command_;
  vehicle_cmd.shift.data = prev_shift_;

  pub_raw_vehicle_command_->publish(vehicle_cmd);
}

void AutowareJoyControllerNode::initTimer(double period_s)
{
  auto timer_callback = std::bind(&AutowareJoyControllerNode::onTimer, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

AutowareJoyControllerNode::AutowareJoyControllerNode()
: Node("autoware_joy_controller")
{
  // Parameter
  joy_type_ = declare_parameter("joy_type", std::string("DS4"));
  update_rate_ = declare_parameter("update_rate", 10.0);
  accel_ratio_ = declare_parameter("accel_ratio", 3.0);
  brake_ratio_ = declare_parameter("brake_ratio", 5.0);
  steer_ratio_ = declare_parameter("steer_ratio", 0.5);
  steering_angle_velocity_ = declare_parameter("steering_angle_velocity", 0.1);
  velocity_gain_ = declare_parameter("control_command/velocity_gain", 3.0);
  max_forward_velocity_ = declare_parameter("control_command/max_forward_velocity", 20.0);
  max_backward_velocity_ = declare_parameter("control_command/max_backward_velocity", 3.0);
  backward_accel_ratio_ = declare_parameter("control_command/backward_accel_ratio", 1.0);

  RCLCPP_INFO(get_logger(), "Joy type: %s", joy_type_.c_str());

  // Subscriber
  sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "input/joy", 1, 
    std::bind(&AutowareJoyControllerNode::onJoy, this, std::placeholders::_1));
  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist", 1, 
    std::bind(&AutowareJoyControllerNode::onTwist, this, std::placeholders::_1));

  // Publisher
  pub_control_command_ = this->create_publisher<autoware_control_msgs::msg::ControlCommandStamped>(
    "output/control_command", 1);
  pub_raw_control_command_ = this->create_publisher<autoware_vehicle_msgs::msg::RawControlCommandStamped>(
    "output/raw_control_command", 1);
  pub_shift_ = this->create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>("output/shift", 1);
  pub_turn_signal_ =
    this->create_publisher<autoware_vehicle_msgs::msg::TurnSignal>("output/turn_signal", 1);
  pub_gate_mode_ = this->create_publisher<autoware_control_msgs::msg::GateMode>("output/gate_mode", 1);
  pub_emergency_ = this->create_publisher<autoware_debug_msgs::msg::BoolStamped>("output/emergency", 1);
  pub_autoware_engage_ = this->create_publisher<autoware_debug_msgs::msg::BoolStamped>("output/autoware_engage", 1);
  pub_vehicle_engage_ = this->create_publisher<autoware_debug_msgs::msg::BoolStamped>("output/vehicle_engage", 1);

  // tmp
  pub_vehicle_command_ =
    this->create_publisher<autoware_vehicle_msgs::msg::VehicleCommand>("output/vehicle_cmd", 1);
  pub_raw_vehicle_command_ =
    this->create_publisher<autoware_vehicle_msgs::msg::RawVehicleCommand>("output/raw_vehicle_cmd", 1);

  // Timer
  initTimer(1.0/update_rate_);
}
