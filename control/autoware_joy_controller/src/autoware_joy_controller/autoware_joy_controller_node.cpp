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

#include <autoware_joy_controller/autoware_joy_controller.h>

namespace
{
ShiftType getUpperShift(const ShiftType & shift)
{
  using autoware_vehicle_msgs::Shift;

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
  using autoware_vehicle_msgs::Shift;

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
  using autoware_vehicle_msgs::Shift;

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
  using autoware_vehicle_msgs::TurnSignal;

  if (turn_signal == TurnSignal::NONE) return "NONE";
  if (turn_signal == TurnSignal::LEFT) return "LEFT";
  if (turn_signal == TurnSignal::RIGHT) return "RIGHT";
  if (turn_signal == TurnSignal::HAZARD) return "HAZARD";

  return "NOT_SUPPORTED";
}

const char * getGateModeName(const GateModeType & gate_mode)
{
  using autoware_control_msgs::GateMode;

  if (gate_mode == GateMode::AUTO) return "AUTO";
  if (gate_mode == GateMode::REMOTE) return "REMOTE";

  return "NOT_SUPPORTED";
}

}  // namespace

void AutowareJoyControllerNode::onJoy(const sensor_msgs::Joy::ConstPtr & msg)
{
  last_joy_received_time_ = msg->header.stamp;
  joy_ = std::make_shared<const JoyConverter>(*msg);

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

void AutowareJoyControllerNode::onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  twist_ = msg;
}

bool AutowareJoyControllerNode::isDataReady()
{
  // Joy
  {
    if (!joy_) {
      ROS_WARN_THROTTLE(1.0, "waiting for joy msg...");
      return false;
    }

    constexpr auto timeout = 2.0;
    const auto time_diff = ros::Time::now() - last_joy_received_time_;
    if (time_diff.toSec() > timeout) {
      ROS_WARN_THROTTLE(1.0, "joy msg is timeout");
      return false;
    }
  }

  // Twist
  {
    if (!twist_) {
      ROS_WARN_THROTTLE(1.0, "waiting for twist msg...");
      return false;
    }

    constexpr auto timeout = 0.5;
    const auto time_diff = ros::Time::now() - twist_->header.stamp;
    if (time_diff.toSec() > timeout) {
      ROS_WARN_THROTTLE(1.0, "twist msg is timeout");
      return false;
    }
  }

  return true;
}

void AutowareJoyControllerNode::onTimer(const ros::TimerEvent & event)
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
  autoware_control_msgs::ControlCommandStamped cmd_stamped;
  cmd_stamped.header.stamp = ros::Time::now();

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

  pub_control_command_.publish(cmd_stamped);
  prev_control_command_ = cmd_stamped.control;
}

void AutowareJoyControllerNode::publishRawControlCommand()
{
  autoware_vehicle_msgs::RawControlCommandStamped cmd_stamped;
  cmd_stamped.header.stamp = ros::Time::now();

  {
    auto & cmd = cmd_stamped.control;

    cmd.steering_angle = steer_ratio_ * joy_->steer();
    cmd.steering_angle_velocity = steering_angle_velocity_;
    cmd.throttle = accel_ratio_ * joy_->accel();
    cmd.brake = brake_ratio_ * joy_->brake();
  }

  pub_raw_control_command_.publish(cmd_stamped);
  prev_raw_control_command_ = cmd_stamped.control;
}

void AutowareJoyControllerNode::publishShift()
{
  using autoware_vehicle_msgs::Shift;

  autoware_vehicle_msgs::ShiftStamped shift_stamped;
  shift_stamped.header.stamp = ros::Time::now();

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

    ROS_INFO("Shift::%s", getShiftName(shift.data));
  }

  pub_shift_.publish(shift_stamped);
  prev_shift_ = shift_stamped.shift.data;
}

void AutowareJoyControllerNode::publishTurnSignal()
{
  using autoware_vehicle_msgs::TurnSignal;

  TurnSignal turn_signal;
  turn_signal.header.stamp = ros::Time::now();

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

  ROS_INFO("TurnSignal::%s", getTurnSignalName(turn_signal.data));

  pub_turn_signal_.publish(turn_signal);
}

void AutowareJoyControllerNode::publishGateMode()
{
  using autoware_control_msgs::GateMode;

  autoware_control_msgs::GateMode gate_mode;

  if (prev_gate_mode_ == GateMode::AUTO) {
    gate_mode.data = GateMode::REMOTE;
  }

  if (prev_gate_mode_ == GateMode::REMOTE) {
    gate_mode.data = GateMode::AUTO;
  }

  ROS_INFO("GateMode::%s", getGateModeName(gate_mode.data));

  pub_gate_mode_.publish(gate_mode);
  prev_gate_mode_ = gate_mode.data;
}

void AutowareJoyControllerNode::publishEmergency()
{
  std_msgs::Bool emergency;

  if (joy_->emergency()) {
    emergency.data = true;
    ROS_INFO("Emergency");
  }

  if (joy_->clear_emergency()) {
    emergency.data = false;
    ROS_INFO("Clear Emergency");
  }

  pub_emergency_.publish(emergency);
}

void AutowareJoyControllerNode::publishAutowareEngage()
{
  std_msgs::Bool engage;

  if (joy_->autoware_engage()) {
    engage.data = true;
    ROS_INFO("Autoware Engage");
  }

  if (joy_->autoware_disengage()) {
    engage.data = false;
    ROS_INFO("Autoware Disengage");
  }

  pub_autoware_engage_.publish(engage);
}

void AutowareJoyControllerNode::publishVehicleEngage()
{
  std_msgs::Bool engage;

  if (joy_->vehicle_engage()) {
    engage.data = true;
    ROS_INFO("Vehicle Engage");
  }

  if (joy_->vehicle_disengage()) {
    engage.data = false;
    ROS_INFO("Vehicle Disengage");
  }

  pub_vehicle_engage_.publish(engage);
}

// tmp
void AutowareJoyControllerNode::publishVehicleCommand()
{
  autoware_vehicle_msgs::VehicleCommand vehicle_cmd;
  vehicle_cmd.header.stamp = ros::Time::now();
  vehicle_cmd.control = prev_control_command_;
  vehicle_cmd.shift.data = prev_shift_;

  pub_vehicle_command_.publish(vehicle_cmd);
}

void AutowareJoyControllerNode::publishRawVehicleCommand()
{
  autoware_vehicle_msgs::RawVehicleCommand vehicle_cmd;
  vehicle_cmd.header.stamp = ros::Time::now();
  vehicle_cmd.control = prev_raw_control_command_;
  vehicle_cmd.shift.data = prev_shift_;

  pub_raw_vehicle_command_.publish(vehicle_cmd);
}

AutowareJoyControllerNode::AutowareJoyControllerNode()
{
  // Parameter
  private_nh_.param("update_rate", update_rate_, 10.0);
  private_nh_.param("accel_ratio", accel_ratio_, 3.0);
  private_nh_.param("brake_ratio", brake_ratio_, 5.0);
  private_nh_.param("steer_ratio", steer_ratio_, 0.5);
  private_nh_.param("steering_angle_velocity", steering_angle_velocity_, 0.1);
  private_nh_.param("control_command/velocity_gain", velocity_gain_, 3.0);
  private_nh_.param("control_command/max_forward_velocity", max_forward_velocity_, 20.0);
  private_nh_.param("control_command/max_backward_velocity", max_backward_velocity_, 3.0);
  private_nh_.param("control_command/backward_accel_ratio", backward_accel_ratio_, 1.0);

  // Subscriber
  sub_joy_ = private_nh_.subscribe("input/joy", 1, &AutowareJoyControllerNode::onJoy, this);
  sub_twist_ = private_nh_.subscribe("input/twist", 1, &AutowareJoyControllerNode::onTwist, this);

  // Publisher
  pub_control_command_ = private_nh_.advertise<autoware_control_msgs::ControlCommandStamped>(
    "output/control_command", 1);
  pub_raw_control_command_ = private_nh_.advertise<autoware_vehicle_msgs::RawControlCommandStamped>(
    "output/raw_control_command", 1);
  pub_shift_ = private_nh_.advertise<autoware_vehicle_msgs::ShiftStamped>("output/shift", 1);
  pub_turn_signal_ =
    private_nh_.advertise<autoware_vehicle_msgs::TurnSignal>("output/turn_signal", 1);
  pub_gate_mode_ = private_nh_.advertise<autoware_control_msgs::GateMode>("output/gate_mode", 1);
  pub_emergency_ = private_nh_.advertise<std_msgs::Bool>("output/emergency", 1);
  pub_autoware_engage_ = private_nh_.advertise<std_msgs::Bool>("output/autoware_engage", 1);
  pub_vehicle_engage_ = private_nh_.advertise<std_msgs::Bool>("output/vehicle_engage", 1);

  // tmp
  pub_vehicle_command_ =
    private_nh_.advertise<autoware_vehicle_msgs::VehicleCommand>("output/vehicle_cmd", 1);
  pub_raw_vehicle_command_ =
    private_nh_.advertise<autoware_vehicle_msgs::RawVehicleCommand>("output/raw_vehicle_cmd", 1);

  // Timer
  timer_ =
    private_nh_.createTimer(ros::Rate(update_rate_), &AutowareJoyControllerNode::onTimer, this);
}
