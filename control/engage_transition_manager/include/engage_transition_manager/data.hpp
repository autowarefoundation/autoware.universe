// Copyright 2022 Autoware Foundation
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

#ifndef ENGAGE_TRANSITION_MANAGER__DATA_HPP_
#define ENGAGE_TRANSITION_MANAGER__DATA_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/srv/autonomy_mode_change.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_system_msgs/msg/operation_mode.hpp>
#include <tier4_system_msgs/srv/operation_mode_request.hpp>
#include <tier4_vehicle_msgs/msg/control_mode.hpp>
#include <tier4_vehicle_msgs/srv/control_mode_request.hpp>

namespace engage_transition_manager
{

using nav_msgs::msg::Odometry;

using autoware_auto_planning_msgs::msg::Trajectory;
using tier4_system_msgs::msg::OperationMode;
using tier4_system_msgs::srv::OperationModeRequest;
using tier4_vehicle_msgs::msg::ControlMode;
using tier4_vehicle_msgs::srv::ControlModeRequest;

enum class State {
  STOP = 0,
  MANUAL_DIRECT,
  REMOTE_OPERATOR,
  LOCAL_OPERATOR,
  TRANSITION_TO_AUTO,
  AUTONOMOUS,
};

struct Data
{
  bool is_auto_available;
  State requested_state;
  Odometry kinematics;
  Trajectory trajectory;
};

struct EngageAcceptableParam
{
  double dist_threshold = 2.0;
  double speed_threshold = 10.0;
  double yaw_threshold = 0.785;
};

struct StableCheckParam
{
  double duration = 3.0;
  double dist_threshold = 0.5;
  double speed_threshold = 3.0;
  double yaw_threshold = M_PI / 10.0;
};

uint8_t toMsg(const State s);
State toEnum(const OperationMode s);
std::string toStr(const State s);
bool isManual(const State s);
bool isAuto(const State s);

}  // namespace engage_transition_manager

#endif  // ENGAGE_TRANSITION_MANAGER__DATA_HPP_
