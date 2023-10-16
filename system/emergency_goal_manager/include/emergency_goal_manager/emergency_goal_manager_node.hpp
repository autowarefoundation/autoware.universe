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

#ifndef EMERGENCY_GOAL_MANAGER__EMERGENCY_GOAL_MANAGER_CORE_HPP_
#define EMERGENCY_GOAL_MANAGER__EMERGENCY_GOAL_MANAGER_CORE_HPP_

// Core
#include <unordered_map>
#include <queue>
#include <string>

// Autoware
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <tier4_system_msgs/msg/emergency_goals_clear_command.hpp>
#include <tier4_system_msgs/msg/emergency_goals_stamped.hpp>

// ROS 2 core
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace emergency_goal_manager
{

struct Parameters
{
};

class EmergencyGoalManagerNode : public rclcpp::Node
{
public:
  explicit EmergencyGoalManagerNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  Parameters params_;

  // Subscriber
  rclcpp::Subscription<tier4_system_msgs::msg::EmergencyGoalsStamped>::SharedPtr sub_emergency_goals_;
  rclcpp::Subscription<tier4_system_msgs::msg::EmergencyGoalsClearCommand>::SharedPtr
    sub_emergency_goals_clear_command_;
  
  void onEmergencyGoals(const tier4_system_msgs::msg::EmergencyGoalsStamped::SharedPtr msg);
  void onEmergencyGoalsClearCommand(
    const tier4_system_msgs::msg::EmergencyGoalsClearCommand::SharedPtr msg);

  // Server
 
  // Publisher

  // Client
  rclcpp::CallbackGroup::SharedPtr client_set_mrm_route_points_callback_group_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedPtr
    client_set_mrm_route_points_;
  rclcpp::CallbackGroup::SharedPtr client_clear_mrm_route_callback_group_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_clear_mrm_route_;
  
  void callSetMrmRoutePoints();
  void callClearMrmRoute();

  // Timer

  // States
  std::unordered_map<std::string, std::queue<geometry_msgs::msg::Pose>> emergency_goals_map_;

  // Algorithm
};

}  // namespace emergency_goal_manager

#endif  // EMERGENCY_GOAL_MANAGER__EMERGENCY_GOAL_MANAGER_CORE_HPP_
