/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"

#include "vehicle_cmd_gate/vehicle_cmd_gate.hpp"

using std::placeholders::_1;

namespace
{
void fillFrameId(std::string * frame_id, const std::string & name)
{
  if (*frame_id == "") {
    *frame_id = name;
  }
}

const char * getGateModeName(const autoware_control_msgs::msg::GateMode::_data_type & gate_mode)
{
  using autoware_control_msgs::msg::GateMode;

  if (gate_mode == GateMode::AUTO) {return "AUTO";}
  if (gate_mode == GateMode::REMOTE) {return "REMOTE";}

  return "NOT_SUPPORTED";
}
}  // namespace

VehicleCmdGate::VehicleCmdGate()
: Node("vehicle_cmd_gate"), is_engaged_(false), is_emergency_(false)
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  // Publisher
  vehicle_cmd_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VehicleCommand>(
    "output/vehicle_cmd", durable_qos);
  control_cmd_pub_ = this->create_publisher<autoware_control_msgs::msg::ControlCommandStamped>(
    "output/control_cmd", durable_qos);
  shift_cmd_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>(
    "output/shift_cmd", durable_qos);
  turn_signal_cmd_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::TurnSignal>(
    "output/turn_signal_cmd", durable_qos);
  gate_mode_pub_ =
    this->create_publisher<autoware_control_msgs::msg::GateMode>("output/gate_mode", durable_qos);

  // Subscriber
  engage_sub_ = this->create_subscription<autoware_control_msgs::msg::EngageMode>(
    "input/engage", 1, std::bind(&VehicleCmdGate::onEngage, this, _1));
  emergency_sub_ = this->create_subscription<autoware_control_msgs::msg::EmergencyMode>(
    "input/emergency", 1, std::bind(&VehicleCmdGate::onEmergency, this, _1));
  gate_mode_sub_ = this->create_subscription<autoware_control_msgs::msg::GateMode>(
    "input/gate_mode", 1, std::bind(&VehicleCmdGate::onGateMode, this, _1));

  // Subscriber for auto
  auto_control_cmd_sub_ =
    this->create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
    "input/auto/control_cmd", 1, std::bind(&VehicleCmdGate::onAutoCtrlCmd, this, _1));
  auto_turn_signal_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "input/auto/turn_signal_cmd", 1, std::bind(&VehicleCmdGate::onAutoTurnSignalCmd, this, _1));
  auto_shift_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "input/auto/shift_cmd", 1, std::bind(&VehicleCmdGate::onAutoShiftCmd, this, _1));

  // Subscriber for remote
  remote_control_cmd_sub_ =
    this->create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
    "input/remote/control_cmd", 1, std::bind(&VehicleCmdGate::onRemoteCtrlCmd, this, _1));
  remote_turn_signal_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "input/remote/turn_signal_cmd", 1, std::bind(&VehicleCmdGate::onRemoteTurnSignalCmd, this, _1));
  remote_shift_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "input/remote/shift_cmd", 1, std::bind(&VehicleCmdGate::onRemoteShiftCmd, this, _1));

  // Subscriber for emergency
  emergency_control_cmd_sub_ =
    this->create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
    "input/emergency/control_cmd", 1, std::bind(&VehicleCmdGate::onEmergencyCtrlCmd, this, _1));
  emergency_turn_signal_cmd_sub_ =
    this->create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "input/emergency/turn_signal_cmd", 1,
    std::bind(&VehicleCmdGate::onEmergencyTurnSignalCmd, this, _1));
  emergency_shift_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "input/emergency/shift_cmd", 1, std::bind(&VehicleCmdGate::onEmergencyShiftCmd, this, _1));

  // Parameter
  update_period_ = 1.0 / declare_parameter("update_rate", 10.0);
  use_emergency_handling_ = declare_parameter("use_emergency_handling", false);

  // Vehicle Parameter
  double wheel_base = declare_parameter("/vehicle_info/wheel_base", 2.79);
  double vel_lim = declare_parameter("vel_lim", 25.0);
  double lon_acc_lim = declare_parameter("lon_acc_lim", 5.0);
  double lon_jerk_lim = declare_parameter("lon_jerk_lim", 5.0);
  double lat_acc_lim = declare_parameter("lat_acc_lim", 5.0);
  double lat_jerk_lim = declare_parameter("lat_jerk_lim", 5.0);
  filter_.setWheelBase(wheel_base);
  filter_.setVelLim(vel_lim);
  filter_.setLonAccLim(lon_acc_lim);
  filter_.setLonJerkLim(lon_jerk_lim);
  filter_.setLatAccLim(lat_acc_lim);
  filter_.setLatJerkLim(lat_jerk_lim);

  // Set default value
  current_gate_mode_.data = autoware_control_msgs::msg::GateMode::AUTO;

  // Timer
  auto timer_callback = std::bind(&VehicleCmdGate::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(update_period_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

// for auto
void VehicleCmdGate::onAutoCtrlCmd(
  autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg)
{
  auto_commands_.control = *msg;

  if (current_gate_mode_.data == autoware_control_msgs::msg::GateMode::AUTO) {
    publishControlCommands(auto_commands_);
  }
}
void VehicleCmdGate::onAutoTurnSignalCmd(autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  auto_commands_.turn_signal = *msg;
}
void VehicleCmdGate::onAutoShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  auto_commands_.shift = *msg;
}

// for remote
void VehicleCmdGate::onRemoteCtrlCmd(
  autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg)
{
  remote_commands_.control = *msg;

  if (current_gate_mode_.data == autoware_control_msgs::msg::GateMode::REMOTE) {
    publishControlCommands(remote_commands_);
  }
}
void VehicleCmdGate::onRemoteTurnSignalCmd(
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  remote_commands_.turn_signal = *msg;
}
void VehicleCmdGate::onRemoteShiftCmd(autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  remote_commands_.shift = *msg;
}

// for emergency
void VehicleCmdGate::onEmergencyCtrlCmd(
  autoware_control_msgs::msg::ControlCommandStamped::ConstSharedPtr msg)
{
  emergency_commands_.control = *msg;

  if (use_emergency_handling_ && is_emergency_) {
    publishControlCommands(emergency_commands_);
  }
}
void VehicleCmdGate::onEmergencyTurnSignalCmd(
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  emergency_commands_.turn_signal = *msg;
}
void VehicleCmdGate::onEmergencyShiftCmd(
  autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  emergency_commands_.shift = *msg;
}

void VehicleCmdGate::onTimer()
{
  // Select commands
  autoware_vehicle_msgs::msg::TurnSignal turn_signal;
  autoware_vehicle_msgs::msg::ShiftStamped shift;
  if (use_emergency_handling_ && is_emergency_) {
    turn_signal = emergency_commands_.turn_signal;
    shift = emergency_commands_.shift;
  } else if (current_gate_mode_.data == autoware_control_msgs::msg::GateMode::AUTO) {
    turn_signal = auto_commands_.turn_signal;
    shift = auto_commands_.shift;
  } else if (current_gate_mode_.data == autoware_control_msgs::msg::GateMode::REMOTE) {
    turn_signal = remote_commands_.turn_signal;
    shift = remote_commands_.shift;
  } else {
    throw std::runtime_error("invalid mode");
  }

  // Add frame_id to prevent RViz warnings
  fillFrameId(&shift.header.frame_id, "base_link");
  fillFrameId(&turn_signal.header.frame_id, "base_link");

  // Publish topics
  gate_mode_pub_->publish(current_gate_mode_);
  turn_signal_cmd_pub_->publish(turn_signal);
  shift_cmd_pub_->publish(shift);
}

void VehicleCmdGate::publishControlCommands(const Commands & commands)
{
  Commands filtered_commands;

  // Set default commands
  {
    filtered_commands.control = commands.control;
    filtered_commands.shift = commands.shift;  // tmp
  }

  // Check emergency
  if (use_emergency_handling_ && is_emergency_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "Emergency!");
    filtered_commands.control = emergency_commands_.control;
    filtered_commands.shift = emergency_commands_.shift;  // tmp
  }

  // Check engage
  if (!is_engaged_) {
    filtered_commands.control.control = createStopControlCmd();
  }

  // Apply limit filtering
  filtered_commands.control.control = filterControlCommand(filtered_commands.control.control);

  // tmp: Create VehicleCmd
  autoware_vehicle_msgs::msg::VehicleCommand vehicle_cmd;
  vehicle_cmd.header = filtered_commands.control.header;
  vehicle_cmd.control = filtered_commands.control.control;
  vehicle_cmd.shift = filtered_commands.shift.shift;
  vehicle_cmd.emergency = (use_emergency_handling_ && is_emergency_);

  // Add frame_id to prevent RViz warnings
  fillFrameId(&vehicle_cmd.header.frame_id, "base_link");
  fillFrameId(&filtered_commands.control.header.frame_id, "base_link");

  // Publish commands
  vehicle_cmd_pub_->publish(vehicle_cmd);
  control_cmd_pub_->publish(filtered_commands.control);

  // Save ControlCmd to steering angle when disengaged
  prev_control_cmd_ = filtered_commands.control.control;
}

autoware_control_msgs::msg::ControlCommand VehicleCmdGate::filterControlCommand(
  const autoware_control_msgs::msg::ControlCommand & in)
{
  autoware_control_msgs::msg::ControlCommand out = in;
  const double dt = getDt();

  filter_.limitLongitudinalWithVel(out);
  filter_.limitLongitudinalWithAcc(dt, out);
  filter_.limitLongitudinalWithJerk(dt, out);
  filter_.limitLateralWithLatAcc(dt, out);
  filter_.limitLateralWithLatJerk(dt, out);
  filter_.setPrevCmd(out);

  return out;
}

autoware_control_msgs::msg::ControlCommand VehicleCmdGate::createStopControlCmd() const
{
  autoware_control_msgs::msg::ControlCommand cmd;

  cmd.steering_angle = prev_control_cmd_.steering_angle;
  cmd.steering_angle_velocity = prev_control_cmd_.steering_angle_velocity;
  cmd.velocity = 0.0;
  cmd.acceleration = -1.5;

  return cmd;
}

void VehicleCmdGate::onEngage(autoware_control_msgs::msg::EngageMode::ConstSharedPtr msg)
{
  is_engaged_ = msg->is_engaged;
}

void VehicleCmdGate::onEmergency(autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr msg)
{
  is_emergency_ = msg->is_emergency;
}

void VehicleCmdGate::onGateMode(autoware_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  const auto prev_gate_mode = current_gate_mode_;
  current_gate_mode_ = *msg;

  if (current_gate_mode_.data != prev_gate_mode.data) {
    RCLCPP_INFO(
      get_logger(), "GateMode changed: %s -> %s", getGateModeName(prev_gate_mode.data),
      getGateModeName(current_gate_mode_.data));
  }
}

double VehicleCmdGate::getDt()
{
  if (!prev_time_) {
    prev_time_ = std::make_shared<rclcpp::Time>(this->now());
    return 0.0;
  }

  const auto current_time = this->now();
  const auto dt = (current_time - *prev_time_).seconds();
  *prev_time_ = current_time;

  return dt;
}
