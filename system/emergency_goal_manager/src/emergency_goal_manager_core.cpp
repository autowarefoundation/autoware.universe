// Copyright 2022 Tier IV, Inc.
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

#include <emergency_goal_manager_core.hpp>

namespace emergency_goal_manager
{
EmergencyGoalManager::EmergencyGoalManager() : Node("emergency_goal_manager")
{
  // Subscriber
  sub_emergency_goals_ = create_subscription<tier4_system_msgs::msg::EmergencyGoalsStamped>(
    "~/input/emergency_goals", rclcpp::QoS{1},
    std::bind(&EmergencyGoalManager::onEmergencyGoals, this, std::placeholders::_1));
  sub_emergency_goals_clear_command_ =
    create_subscription<tier4_system_msgs::msg::EmergencyGoalsClearCommand>(
      "~/input/emergency_goals_clear_command", rclcpp::QoS{1},
      std::bind(&EmergencyGoalManager::onEmergencyGoalsClearCommand, this, std::placeholders::_1));

  // Client
  client_set_mrm_route_points_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_set_mrm_route_points_ = create_client<SetRoutePoints>(
    "/planning/mission_planning/mission_planner/srv/set_mrm_route",
    rmw_qos_profile_services_default, client_set_mrm_route_points_callback_group_);
  client_clear_mrm_route_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_clear_mrm_route_ = create_client<ClearRoute>(
    "/planning/mission_planning/mission_planner/srv/clear_mrm_route",
    rmw_qos_profile_services_default, client_clear_mrm_route_callback_group_);

  // Initialize
  while (!client_set_mrm_route_points_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
  }
  while (!client_clear_mrm_route_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
  }
}

void EmergencyGoalManager::onEmergencyGoals(
  const tier4_system_msgs::msg::EmergencyGoalsStamped::SharedPtr msg)
{
  if (!emergency_goals_map_.empty()) {
    emergency_goals_map_.clear();
  }

  std::queue<geometry_msgs::msg::Pose> emergency_goals_queue;
  for (const auto & goal : msg->goals) {
    emergency_goals_queue.push(goal);
  }
  emergency_goals_map_.emplace(msg->sender, emergency_goals_queue);

  callSetMrmRoutePoints();
}

void EmergencyGoalManager::onEmergencyGoalsClearCommand(
  const tier4_system_msgs::msg::EmergencyGoalsClearCommand::SharedPtr msg)
{
  if (emergency_goals_map_.count(msg->sender) == 0) {
    RCLCPP_WARN(get_logger(), "Emergency goals from %s is empty.", msg->sender.c_str());
  }

  if (msg->command) {
    emergency_goals_map_.erase(msg->sender);

    if (emergency_goals_map_.empty()) {
      callClearMrmRoute();
    } else {
      callSetMrmRoutePoints();
    }
  }
}

void EmergencyGoalManager::callSetMrmRoutePoints()
{
  auto request = std::make_shared<SetRoutePoints::Request>();
  request->header.frame_id = "map";
  request->header.stamp = this->now();
  request->option.allow_goal_modification = true;

  while (!emergency_goals_map_.empty()) {
    // TODO: set goals with the highest priority
    auto goals = emergency_goals_map_.begin();

    auto sender = goals->first;
    auto & goal_queue = goals->second;
    if (goal_queue.empty()) {
      emergency_goals_map_.erase(sender);
      continue;
    }

    request->goal = goal_queue.front();
    goal_queue.pop();

    auto future = client_set_mrm_route_points_->async_send_request(request);
    const auto duration = std::chrono::duration<double, std::ratio<1>>(10);
    if (future.wait_for(duration) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "MRM Route service timeout.");
      continue;
    } else {
      if (future.get()->status.success) {
        RCLCPP_INFO(get_logger(), "MRM Route has been successfully sent.");
        return;
      } else {
        RCLCPP_WARN(get_logger(), "MRM Route service has failed.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
    }
  }

  callClearMrmRoute();
}

void EmergencyGoalManager::callClearMrmRoute()
{
  auto request = std::make_shared<ClearRoute::Request>();
  const auto duration = std::chrono::duration<double, std::ratio<1>>(10);
  const auto start_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    if (std::chrono::steady_clock::now() - start_time > duration) {
      RCLCPP_WARN(get_logger(), "Clear MRM Route operation timeout.");
      return;
    }

    auto future = client_clear_mrm_route_->async_send_request(request);
    if (future.wait_for(duration) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Clear MRM Route service timeout.");
      return;
    } else {
      if (future.get()->status.success) {
        RCLCPP_INFO(get_logger(), "Clear MRM Route has been successfully sent.");
        return;
      } else {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_WARN(get_logger(), "Clear MRM Route has failed.");
        continue;
      }
    }
  }
}
}  // namespace emergency_goal_manager
