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
  params_.update_rate = static_cast<int>(declare_parameter<int>("update_rate", 30));
  params_.target_acceleration = declare_parameter<double>("target_acceleration", -2.5);
  params_.target_jerk = declare_parameter<double>("target_jerk", -1.5);

  // Subscriber

  // Server

  // Publisher

  // Timer

  // Initialize
}


}  // namespace emergency_goal_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(emergency_goal_manager::EmergencyGoalManagerNode)
