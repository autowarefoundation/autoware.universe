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

#include <emergency_goal_manager/emergency_goal_manager_node.hpp>


namespace emergency_goal_manager
{

EmergencyGoalManagerNode::EmergencyGoalManagerNode(const rclcpp::NodeOptions & node_options)
: Node("emergency_goal_manager_node", node_options)
{
  // Parameter

  // Subscriber
  sub_emergency_goals_ = create_subscription<tier4_system_msgs::msg::EmergencyGoalsStamped>(
    "~/input/emergency_goals", rclcpp::QoS{1},
    std::bind(&EmergencyGoalManagerNode::onEmergencyGoals, this, std::placeholders::_1));
  sub_emergency_goals_clear_command_ =
    create_subscription<tier4_system_msgs::msg::EmergencyGoalsClearCommand>(
    "~/input/emergency_goals_clear_command", rclcpp::QoS{1},
    std::bind(&EmergencyGoalManagerNode::onEmergencyGoalsClearCommand, this, std::placeholders::_1));

  // Server

  // Publisher
 
  // Client
  client_set_mrm_route_points_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_set_mrm_route_points_ = create_client<autoware_adapi_v1_msgs::srv::SetRoutePoints>(
    "~/output/set_mrm_route_points", rmw_qos_profile_services_default,
    client_set_mrm_route_points_callback_group_);
  client_clear_mrm_route_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_clear_mrm_route_ = create_client<std_srvs::srv::Trigger>(
    "~/output/clear_mrm_route_", rmw_qos_profile_services_default,
    client_clear_mrm_route_callback_group_);

  // Timer

  // Initialize
}

void EmergencyGoalManagerNode::onEmergencyGoals(
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

void EmergencyGoalManagerNode::onEmergencyGoalsClearCommand(
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

void EmergencyGoalManagerNode::callSetMrmRoutePoints()
{
  
}

void EmergencyGoalManagerNode::callClearMrmRoute()
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
}

}  // namespace emergency_goal_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(emergency_goal_manager::EmergencyGoalManagerNode)
