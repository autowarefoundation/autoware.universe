// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
// governing permissions and limitations under the License.

#include "redundancy_relay_manager_node.hpp"

namespace autoware::redundancy_relay_manager
{
RedundancyRelayManager::RedundancyRelayManager(const rclcpp::NodeOptions & options)
: Node("redundancy_relay_manager", options)
{
  // Subscribers
  sub_operation_mode_state_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "~/input/operation_mode/state", rclcpp::QoS{1},
    std::bind(&RedundancyRelayManager::onOperationModeState, this, std::placeholders::_1));
  sub_main_election_status_ = create_subscription<tier4_system_msgs::msg::ElectionStatus>(
    "~/input/main/election/status", rclcpp::QoS{1},
    std::bind(&RedundancyRelayManager::onMainElectionStatus, this, std::placeholders::_1));
  sub_sub_election_status_ = create_subscription<tier4_system_msgs::msg::ElectionStatus>(
    "~/input/sub/election/status", rclcpp::QoS{1},
    std::bind(&RedundancyRelayManager::onSubElectionStatus, this, std::placeholders::_1));
  
}

void RedundancyRelayManager::onOperationModeState(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received operation mode state: %d", msg->mode);
}

void RedundancyRelayManager::onMainElectionStatus(
  const tier4_system_msgs::msg::ElectionStatus::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received main election state: %d", msg->path_info);
}

void RedundancyRelayManager::onSubElectionStatus(
  const tier4_system_msgs::msg::ElectionStatus::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received sub election state: %d", msg->path_info);
}
}  // namespace autoware::redundancy_relay_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::redundancy_relay_manager::RedundancyRelayManager)
