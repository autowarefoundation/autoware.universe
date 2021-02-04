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

#ifndef AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_
#define AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_

#include <deque>
#include <string>
#include <vector>

#include "autoware_planning_msgs/msg/route.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_control_msgs/msg/emergency_mode.hpp"
#include "autoware_control_msgs/msg/engage_mode.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_vehicle_msgs/msg/control_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/time.hpp"

#include "autoware_state_monitor/autoware_state.hpp"
#include "autoware_state_monitor/config.hpp"
#include "autoware_state_monitor/module_name.hpp"

struct StateInput
{
  TopicStats topic_stats;
  ParamStats param_stats;
  TfStats tf_stats;

  rclcpp::Time current_time;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  geometry_msgs::msg::Pose::ConstSharedPtr goal_pose;

  autoware_control_msgs::msg::EngageMode::ConstSharedPtr autoware_engage;
  autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr vehicle_control_mode;
  autoware_control_msgs::msg::EmergencyMode::ConstSharedPtr emergency_mode;
  bool is_finalizing = false;
  autoware_planning_msgs::msg::Route::ConstSharedPtr route;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist;
  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_buffer;
};

struct StateParam
{
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
};

struct Times
{
  rclcpp::Time arrived_goal;
  rclcpp::Time initializing_completed;
  rclcpp::Time planning_completed;
};

struct Flags
{
  bool waiting_after_initializing = false;
  bool waiting_after_planning = false;
};

class StateMachine
{
public:
  explicit StateMachine(const StateParam & state_param)
  : state_param_(state_param) {}

  AutowareState getCurrentState() const {return autoware_state_;}
  AutowareState updateState(const StateInput & state_input);
  std::vector<std::string> getMessages() const {return msgs_;}

private:
  AutowareState autoware_state_ = AutowareState::InitializingVehicle;
  StateInput state_input_;
  const StateParam state_param_;

  mutable AutowareState state_before_emergency_ = AutowareState::InitializingVehicle;
  mutable std::vector<std::string> msgs_;
  mutable Times times_;
  mutable Flags flags_;
  mutable autoware_planning_msgs::msg::Route::ConstSharedPtr executing_route_ = nullptr;

  AutowareState judgeAutowareState() const;

  bool isModuleInitialized(const char * module_name) const;
  bool isVehicleInitialized() const;
  bool hasRoute() const;
  bool isRouteReceived() const;
  bool isPlanningCompleted() const;
  bool isEngaged() const;
  bool isOverridden() const;
  bool isEmergency() const;
  bool hasArrivedGoal() const;
  bool isFinalizing() const;
};

#endif  // AUTOWARE_STATE_MONITOR__STATE_MACHINE_HPP_
