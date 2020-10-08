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

#ifndef AWAPI_AUTOWARE_UTIL_H
#define AWAPI_AUTOWARE_UTIL_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <pacmod_msgs/GlobalRpt.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_control_msgs/GateMode.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/StopReasonArray.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_system_msgs/AutowareState.h>
#include <autoware_vehicle_msgs/ControlMode.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/Steering.h>
#include <autoware_vehicle_msgs/TurnSignal.h>
#include <autoware_vehicle_msgs/VehicleCommand.h>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace autoware_api
{
struct AutowareInfo
{
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr;
  autoware_vehicle_msgs::Steering::ConstPtr steer_ptr;
  autoware_vehicle_msgs::VehicleCommand::ConstPtr vehicle_cmd_ptr;
  autoware_vehicle_msgs::TurnSignal::ConstPtr turn_signal_ptr;
  geometry_msgs::TwistStamped::ConstPtr twist_ptr;
  autoware_vehicle_msgs::ShiftStamped::ConstPtr gear_ptr;
  std_msgs::Float32::ConstPtr battery_ptr;
  sensor_msgs::NavSatFix::ConstPtr nav_sat_ptr;
  autoware_system_msgs::AutowareState::ConstPtr autoware_state_ptr;
  autoware_vehicle_msgs::ControlMode::ConstPtr control_mode_ptr;
  autoware_control_msgs::GateMode::ConstPtr gate_mode_ptr;
  std_msgs::Bool::ConstPtr is_emergency_ptr;
  autoware_planning_msgs::StopReasonArray::ConstPtr stop_reason_ptr;
  diagnostic_msgs::DiagnosticArray::ConstPtr diagnostic_ptr;
  pacmod_msgs::GlobalRpt::ConstPtr global_rpt_ptr;
  std_msgs::Bool::ConstPtr lane_change_available_ptr;
  std_msgs::Bool::ConstPtr lane_change_ready_ptr;
  autoware_planning_msgs::Path::ConstPtr lane_change_candidate_ptr;
  std_msgs::Bool::ConstPtr obstacle_avoid_ready_ptr;
  autoware_planning_msgs::Trajectory::ConstPtr obstacle_avoid_candidate_ptr;
};

template <class T>
T getParam(const ros::NodeHandle & nh, const std::string & key, const T & default_value)
{
  T value;
  nh.param<T>(key, value, default_value);
  return value;
}

template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_WARN("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}

double lowpass_filter(const double current_value, const double prev_value, const double gain);

}  // namespace autoware_api

#endif  // AWAPI_AUTOWARE_UTIL_H
