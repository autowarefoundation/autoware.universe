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

#pragma once

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_control_msgs/GateMode.h>
#include <autoware_vehicle_msgs/RawControlCommand.h>
#include <autoware_vehicle_msgs/RawControlCommandStamped.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/VehicleCommand.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>

#include <raw_vehicle_cmd_converter/accel_map.h>
#include <raw_vehicle_cmd_converter/brake_map.h>

class RemoteCmdConverter
{
public:
  RemoteCmdConverter();

private:
  // NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Publisher
  ros::Publisher pub_cmd_;
  ros::Publisher pub_current_cmd_;

  // Subscriber
  ros::Subscriber sub_velocity_;
  ros::Subscriber sub_control_cmd_;
  ros::Subscriber sub_shift_cmd_;
  ros::Subscriber sub_gate_mode_;
  ros::Subscriber sub_emergency_;

  void onVelocity(const geometry_msgs::TwistStamped::ConstPtr msg);
  void onRemoteCmd(const autoware_vehicle_msgs::RawControlCommandStamped::ConstPtr remote_cmd_ptr);
  void onShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr msg);
  void onGateMode(const autoware_control_msgs::GateMode::ConstPtr msg);
  void onEmergency(const std_msgs::Bool::ConstPtr msg);

  std::shared_ptr<double> current_velocity_ptr_;  // [m/s]
  std::shared_ptr<ros::Time> latest_cmd_received_time_;
  autoware_vehicle_msgs::ShiftStamped::ConstPtr current_shift_cmd_;
  autoware_control_msgs::GateMode::ConstPtr current_gate_mode_;
  bool current_emergency_cmd_ = false;

  // Timer
  ros::Timer rate_check_timer_;
  void onTimer(const ros::TimerEvent & event);

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

  double calculateAcc(const autoware_vehicle_msgs::RawControlCommand & cmd, const double vel);
  double getShiftVelocitySign(const autoware_vehicle_msgs::ShiftStamped & cmd);
};
