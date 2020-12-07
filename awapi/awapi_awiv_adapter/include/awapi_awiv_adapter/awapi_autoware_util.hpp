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

#pragma once

#ifndef AWAPI_AUTOWARE_UTIL_H
#define AWAPI_AUTOWARE_UTIL_H

#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pacmod_msgs/msg/global_rpt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "autoware_control_msgs/msg/emergency_mode.hpp"
#include "autoware_control_msgs/msg/gate_mode.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/stop_reason_array.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_vehicle_msgs/msg/control_mode.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/steering.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace autoware_api
{
struct AutowareInfo
{
  std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_ptr;
  autoware_vehicle_msgs::msg::Steering::ConstSharedPtr steer_ptr;
  autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr vehicle_cmd_ptr;
  autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr turn_signal_ptr;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_ptr;
  autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr gear_ptr;
  std_msgs::msg::Float32::ConstSharedPtr battery_ptr;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_ptr;
  autoware_system_msgs::msg::AutowareState::ConstSharedPtr autoware_state_ptr;
  autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr control_mode_ptr;
  autoware_control_msgs::msg::GateMode::ConstSharedPtr gate_mode_ptr;
  autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr is_emergency_ptr;
  autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr stop_reason_ptr;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diagnostic_ptr;
  pacmod_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr;
  std_msgs::msg::Bool::ConstSharedPtr lane_change_available_ptr;
  std_msgs::msg::Bool::ConstSharedPtr lane_change_ready_ptr;
  autoware_planning_msgs::msg::Path::ConstSharedPtr lane_change_candidate_ptr;
  std_msgs::msg::Bool::ConstSharedPtr obstacle_avoid_ready_ptr;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr obstacle_avoid_candidate_ptr;
};

double lowpass_filter(const double current_value, const double prev_value, const double gain);

}  // namespace autoware_api

#endif  // AWAPI_AUTOWARE_UTIL_H
