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

#include <autoware_state_monitor/state_machine.h>

#include <autoware_state_monitor/rosconsole_wrapper.h>

namespace
{
double calcDistance2d(const geometry_msgs::Point & p1, const geometry_msgs::Point & p2)
{
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double calcDistance2d(const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2)
{
  return calcDistance2d(p1.position, p2.position);
}

bool isNearGoal(
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose,
  const double th_dist)
{
  return calcDistance2d(current_pose, goal_pose) < th_dist;
}

bool isStopped(
  const std::deque<geometry_msgs::TwistStamped::ConstPtr> & twist_buffer,
  const double th_stopped_velocity_mps)
{
  for (const auto & twist : twist_buffer) {
    if (std::abs(twist->twist.linear.x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

template <class T>
std::vector<T> filterConfigByModuleName(const std::vector<T> & configs, const char * module_name)
{
  std::vector<T> filtered;

  for (const auto & config : configs) {
    if (config.module == module_name) {
      filtered.push_back(config);
    }
  }

  return filtered;
}

}  // namespace

struct ModuleName
{
  static constexpr const char * map = "map";
  static constexpr const char * sensing = "sensing";
  static constexpr const char * localization = "localization";
  static constexpr const char * perception = "perception";
  static constexpr const char * planning = "planning";
  static constexpr const char * control = "control";
  static constexpr const char * vehicle = "vehicle";
};

bool StateMachine::isModuleInitialized(const char * module_name) const
{
  const auto non_received_topics =
    filterConfigByModuleName(state_input_.topic_stats.non_received_list, module_name);
  const auto non_set_params =
    filterConfigByModuleName(state_input_.param_stats.non_set_list, module_name);
  const auto non_received_tfs =
    filterConfigByModuleName(state_input_.tf_stats.non_received_list, module_name);

  if (non_received_topics.empty() && non_set_params.empty() && non_received_tfs.empty()) {
    return true;
  }

  for (const auto & topic_config : non_received_topics) {
    const auto msg = fmt::format("topic `{}` is not received yet", topic_config.name);
    msgs_.push_back(msg);
  }

  for (const auto & param_config : non_set_params) {
    const auto msg = fmt::format("param `{}` is not set", param_config.name);
    msgs_.push_back(msg);
  }

  for (const auto & tf_config : non_received_tfs) {
    const auto msg =
      fmt::format("tf from `{}` to `{}` is not received yet", tf_config.from, tf_config.to);
    msgs_.push_back(msg);
  }

  {
    const auto msg = fmt::format("module `{}` is not initialized", module_name);
    msgs_.push_back(msg);
  }

  return false;
}

bool StateMachine::isVehicleInitialized() const
{
  if (!isModuleInitialized(ModuleName::map)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::vehicle)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::sensing)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::localization)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::perception)) {
    return false;
  }

  // TODO(Kenji Miyake): Check if the vehicle is on a lane?

  return true;
}

bool StateMachine::isRouteReceived() const { return state_input_.route != executing_route_; }

bool StateMachine::isPlanningCompleted() const
{
  if (!isModuleInitialized(ModuleName::planning)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::control)) {
    return false;
  }

  return true;
}

bool StateMachine::isEngaged() const
{
  if (!state_input_.autoware_engage) {
    return false;
  }

  if (state_input_.autoware_engage->data != 1) {
    return false;
  }

  if (!state_input_.vehicle_control_mode) {
    return false;
  }

  if (state_input_.vehicle_control_mode->data == autoware_vehicle_msgs::ControlMode::MANUAL) {
    return false;
  }

  return true;
}

bool StateMachine::isOverridden() const { return !isEngaged(); }

bool StateMachine::isEmergency() const
{
  if (!state_input_.is_emergency) {
    return false;
  }

  return state_input_.is_emergency->data;
}

bool StateMachine::hasArrivedGoal() const
{
  const auto is_near_goal = isNearGoal(
    state_input_.current_pose->pose, *state_input_.goal_pose, state_param_.th_arrived_distance_m);
  const auto is_stopped =
    isStopped(state_input_.twist_buffer, state_param_.th_stopped_velocity_mps);

  if (is_near_goal && is_stopped) {
    return true;
  }

  return false;
}

AutowareState StateMachine::updateState(const StateInput & state_input)
{
  msgs_ = {};
  state_input_ = state_input;
  autoware_state_ = judgeAutowareState();
  return autoware_state_;
}

AutowareState StateMachine::judgeAutowareState() const
{
  switch (autoware_state_) {
    case (AutowareState::InitializingVehicle): {
      if (isVehicleInitialized()) {
        return AutowareState::WaitingForRoute;
      }

      break;
    }

    case (AutowareState::WaitingForRoute): {
      if (isRouteReceived()) {
        return AutowareState::Planning;
      }

      break;
    }

    case (AutowareState::Planning): {
      executing_route_ = state_input_.route;

      if (isPlanningCompleted()) {
        if (!waiting_after_planning_) {
          waiting_after_planning_ = true;
          times_.planning_completed = ros::Time::now();
          break;
        }

        // Wait after planning completed to avoid sync error
        constexpr double wait_time_after_planning = 1.0;
        const auto time_from_planning = ros::Time::now() - times_.planning_completed;
        if (time_from_planning.toSec() > wait_time_after_planning) {
          waiting_after_planning_ = false;
          return AutowareState::WaitingForEngage;
        }
      }

      break;
    }

    case (AutowareState::WaitingForEngage): {
      if (isEmergency()) {
        return AutowareState::Emergency;
      }

      if (isRouteReceived()) {
        return AutowareState::Planning;
      }

      if (isEngaged()) {
        return AutowareState::Driving;
      }

      break;
    }

    case (AutowareState::Driving): {
      if (isEmergency()) {
        return AutowareState::Emergency;
      }

      if (isRouteReceived()) {
        return AutowareState::Planning;
      }

      if (isOverridden()) {
        return AutowareState::WaitingForEngage;
      }

      if (hasArrivedGoal()) {
        times_.arrived_goal = ros::Time::now();
        return AutowareState::ArrivedGoal;
      }

      break;
    }

    case (AutowareState::ArrivedGoal): {
      constexpr double wait_time_after_arrived_goal = 2.0;
      const auto time_from_arrived_goal = ros::Time::now() - times_.arrived_goal;
      if (time_from_arrived_goal.toSec() > wait_time_after_arrived_goal) {
        return AutowareState::WaitingForRoute;
      }

      break;
    }

    case (AutowareState::Emergency): {
      if (!isEmergency()) {
        return AutowareState::WaitingForEngage;
      }

      break;
    }

    default: {
      throw std::runtime_error("invalid state");
    }
  }

  // continue previous state when break
  return autoware_state_;
}
