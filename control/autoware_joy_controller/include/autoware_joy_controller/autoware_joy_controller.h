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

#include <algorithm>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_control_msgs/GateMode.h>
#include <autoware_vehicle_msgs/RawControlCommandStamped.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/TurnSignal.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <autoware_joy_controller/joy_converter/joy_converter_base.h>

// tmp
#include <autoware_vehicle_msgs/RawVehicleCommand.h>
#include <autoware_vehicle_msgs/VehicleCommand.h>

using ShiftType = autoware_vehicle_msgs::Shift::_data_type;
using TurnSignalType = autoware_vehicle_msgs::TurnSignal::_data_type;
using GateModeType = autoware_control_msgs::GateMode::_data_type;

class AutowareJoyControllerNode
{
public:
  AutowareJoyControllerNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  std::string joy_type_;
  double update_rate_;
  double accel_ratio_;
  double brake_ratio_;
  double steer_ratio_;
  double steering_angle_velocity_;

  // ControlCommand Parameter
  double velocity_gain_;
  double max_forward_velocity_;
  double max_backward_velocity_;
  double backward_accel_ratio_;

  // Subscriber
  ros::Subscriber sub_joy_;
  ros::Subscriber sub_twist_;

  ros::Time last_joy_received_time_;
  std::shared_ptr<const JoyConverterBase> joy_;
  geometry_msgs::TwistStamped::ConstPtr twist_;

  void onJoy(const sensor_msgs::Joy::ConstPtr & msg);
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg);

  // Publisher
  ros::Publisher pub_control_command_;
  ros::Publisher pub_raw_control_command_;
  ros::Publisher pub_shift_;
  ros::Publisher pub_turn_signal_;
  ros::Publisher pub_gate_mode_;
  ros::Publisher pub_emergency_;
  ros::Publisher pub_autoware_engage_;
  ros::Publisher pub_vehicle_engage_;

  void publishControlCommand();
  void publishRawControlCommand();
  void publishShift();
  void publishTurnSignal();
  void publishGateMode();
  void publishEmergency();
  void publishAutowareEngage();
  void publishVehicleEngage();

  // Previous State
  autoware_control_msgs::ControlCommand prev_control_command_;
  autoware_vehicle_msgs::RawControlCommand prev_raw_control_command_;
  ShiftType prev_shift_ = autoware_vehicle_msgs::Shift::NONE;
  TurnSignalType prev_turn_signal_ = autoware_vehicle_msgs::TurnSignal::NONE;
  GateModeType prev_gate_mode_ = autoware_control_msgs::GateMode::AUTO;

  // tmp
  ros::Publisher pub_vehicle_command_;
  ros::Publisher pub_raw_vehicle_command_;
  void publishVehicleCommand();
  void publishRawVehicleCommand();

  // Timer
  ros::Timer timer_;

  bool isDataReady();
  void onTimer(const ros::TimerEvent & event);
};
