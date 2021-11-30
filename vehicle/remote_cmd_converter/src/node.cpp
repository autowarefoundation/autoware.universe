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

#include "remote_cmd_converter/node.hpp"

RemoteCmdConverter::RemoteCmdConverter() : nh_(""), pnh_("~")
{
  pub_cmd_ = pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("out/control_cmd", 1);
  pub_current_cmd_ = pnh_.advertise<autoware_vehicle_msgs::RawControlCommandStamped>(
    "out/latest_raw_control_cmd", 1);

  sub_velocity_ = pnh_.subscribe("in/twist", 1, &RemoteCmdConverter::onVelocity, this);
  sub_control_cmd_ =
    pnh_.subscribe("in/raw_control_cmd", 1, &RemoteCmdConverter::onRemoteCmd, this);
  sub_shift_cmd_ = pnh_.subscribe("in/shift_cmd", 1, &RemoteCmdConverter::onShiftCmd, this);
  sub_gate_mode_ = pnh_.subscribe("in/current_gate_mode", 1, &RemoteCmdConverter::onGateMode, this);
  sub_emergency_ = pnh_.subscribe("in/emergency", 1, &RemoteCmdConverter::onEmergency, this);

  // Parameter
  pnh_.param<double>("ref_vel_gain", ref_vel_gain_, 3.0);

  // Parameter for Hz check
  pnh_.param<double>("time_threshold", time_threshold_, 3.0);
  double timer_rate;
  pnh_.param<double>("timer_rate", timer_rate, 10.0);
  rate_check_timer_ = pnh_.createTimer(ros::Rate(timer_rate), &RemoteCmdConverter::onTimer, this);

  // Parameter for accel/brake map
  std::string csv_path_accel_map, csv_path_brake_map;
  pnh_.param<std::string>("csv_path_accel_map", csv_path_accel_map, std::string("empty"));
  pnh_.param<std::string>("csv_path_brake_map", csv_path_brake_map, std::string("empty"));
  acc_map_initialized_ = true;
  if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
    ROS_ERROR("Cannot read accelmap. csv path = %s. stop calculation.", csv_path_accel_map.c_str());
    acc_map_initialized_ = false;
  }
  if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
    ROS_ERROR("Cannot read brakemap. csv path = %s. stop calculation.", csv_path_brake_map.c_str());
    acc_map_initialized_ = false;
  }

  // Diagnostics
  updater_.setHardwareID("remote_cmd_converter");
  updater_.add("remote_control_topic_status", this, &RemoteCmdConverter::checkTopicStatus);
  updater_.add("emergency_stop_operation", this, &RemoteCmdConverter::checkEmergency);

  // Set default values
  current_shift_cmd_ = boost::make_shared<autoware_vehicle_msgs::ShiftStamped>();
}

void RemoteCmdConverter::onTimer(const ros::TimerEvent & event) { updater_.force_update(); }

void RemoteCmdConverter::onVelocity(const geometry_msgs::TwistStamped::ConstPtr msg)
{
  current_velocity_ptr_ = std::make_shared<double>(msg->twist.linear.x);
}

void RemoteCmdConverter::onShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr msg)
{
  current_shift_cmd_ = msg;
}

void RemoteCmdConverter::onEmergency(const std_msgs::Bool::ConstPtr msg)
{
  current_emergency_cmd_ = msg->data;
  updater_.force_update();
}

void RemoteCmdConverter::onRemoteCmd(
  const autoware_vehicle_msgs::RawControlCommandStampedConstPtr raw_control_cmd_ptr)
{
  // Echo back received command
  {
    auto current_remote_cmd = *raw_control_cmd_ptr;
    current_remote_cmd.header.stamp = ros::Time::now();
    pub_current_cmd_.publish(current_remote_cmd);
  }

  // Save received time for rate check
  latest_cmd_received_time_ = std::make_shared<ros::Time>(ros::Time::now());

  // Wait for input data
  if (!current_velocity_ptr_ || !acc_map_initialized_) {
    return;
  }

  // Calculate reference velocity and acceleration
  const double sign = getShiftVelocitySign(*current_shift_cmd_);
  const double ref_acceleration =
    calculateAcc(raw_control_cmd_ptr->control, std::fabs(*current_velocity_ptr_));

  double ref_velocity = *current_velocity_ptr_ + ref_acceleration * ref_vel_gain_ * sign;
  if (current_shift_cmd_->shift.data == autoware_vehicle_msgs::Shift::REVERSE) {
    ref_velocity = std::min(0.0, ref_velocity);
  } else {
    ref_velocity = std::max(0.0, ref_velocity);
  }

  // Publish ControlCommand
  autoware_control_msgs::ControlCommandStamped output;
  output.header = raw_control_cmd_ptr->header;
  output.control.steering_angle = raw_control_cmd_ptr->control.steering_angle;
  output.control.steering_angle_velocity = raw_control_cmd_ptr->control.steering_angle_velocity;
  output.control.velocity = ref_velocity;
  output.control.acceleration = ref_acceleration;

  pub_cmd_.publish(output);
}

double RemoteCmdConverter::calculateAcc(
  const autoware_vehicle_msgs::RawControlCommand & cmd, const double vel)
{
  const double desired_throttle = cmd.throttle;
  const double desired_brake = cmd.brake;
  const double desired_pedal = desired_throttle - desired_brake;

  double ref_acceleration = 0.0;
  if (desired_pedal > 0.0) {
    accel_map_.getAcceleration(desired_pedal, vel, ref_acceleration);
  } else {
    brake_map_.getAcceleration(-desired_pedal, vel, ref_acceleration);
  }

  return ref_acceleration;
}

double RemoteCmdConverter::getShiftVelocitySign(const autoware_vehicle_msgs::ShiftStamped & cmd)
{
  using autoware_vehicle_msgs::Shift;

  if (cmd.shift.data == Shift::DRIVE) return 1.0;
  if (cmd.shift.data == Shift::LOW) return 1.0;
  if (cmd.shift.data == Shift::REVERSE) return -1.0;

  return 0.0;
}

void RemoteCmdConverter::checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::DiagnosticStatus;

  DiagnosticStatus status;
  if (!checkRemoteTopicRate()) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "low topic rate for remote vehicle_cmd";
  } else {
    status.level = DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}

void RemoteCmdConverter::checkEmergency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::DiagnosticStatus;

  DiagnosticStatus status;
  if (current_emergency_cmd_) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "remote emergency requested";
  } else {
    status.level = DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}

void RemoteCmdConverter::onGateMode(const autoware_control_msgs::GateModeConstPtr msg)
{
  current_gate_mode_ = msg;
}

bool RemoteCmdConverter::checkRemoteTopicRate()
{
  if (!latest_cmd_received_time_ || !current_gate_mode_) return true;

  if (current_gate_mode_->data == autoware_control_msgs::GateMode::REMOTE) {
    const auto duration = (ros::Time::now() - *latest_cmd_received_time_);
    if (duration.toSec() > time_threshold_) return false;
  } else {
    latest_cmd_received_time_ = nullptr;  // reset;
  }

  return true;
}
