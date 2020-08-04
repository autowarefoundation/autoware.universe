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

#include <deque>
#include <string>
#include <vector>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_system_msgs/AutowareState.h>
#include <autoware_vehicle_msgs/ControlMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>

#include <autoware_state_monitor/autoware_state.h>
#include <autoware_state_monitor/config.h>

struct StateInput
{
  TopicStats topic_stats;
  ParamStats param_stats;
  TfStats tf_stats;

  geometry_msgs::PoseStamped::ConstPtr current_pose;
  geometry_msgs::Pose::ConstPtr goal_pose;

  std_msgs::Bool::ConstPtr autoware_engage;
  autoware_vehicle_msgs::ControlMode::ConstPtr vehicle_control_mode;
  std_msgs::Bool::ConstPtr is_emergency;
  autoware_planning_msgs::Route::ConstPtr route;
  geometry_msgs::TwistStamped::ConstPtr twist;
  std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_buffer;
};

struct StateParam
{
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
};

struct Times
{
  ros::Time arrived_goal;
  ros::Time planning_completed;
};

class StateMachine
{
public:
  explicit StateMachine(const StateParam & state_param) : state_param_(state_param) {}

  AutowareState getCurrentState() const { return autoware_state_; }
  AutowareState updateState(const StateInput & state_input);
  std::vector<std::string> getMessages() const { return msgs_; }

private:
  AutowareState autoware_state_ = AutowareState::InitializingVehicle;
  StateInput state_input_;
  const StateParam state_param_;

  mutable std::vector<std::string> msgs_;
  mutable Times times_;
  mutable autoware_planning_msgs::Route::ConstPtr executing_route_ = nullptr;
  mutable bool waiting_after_planning_ = false;

  AutowareState judgeAutowareState() const;

  bool isModuleInitialized(const char * module_name) const;
  bool isVehicleInitialized() const;
  bool isRouteReceived() const;
  bool isPlanningCompleted() const;
  bool isEngaged() const;
  bool isOverridden() const;
  bool isEmergency() const;
  bool hasArrivedGoal() const;
};
