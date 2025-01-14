// Copyright 2025 TIER IV, Inc.
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

#ifndef REDUNDANCY_RELAY_MANAGER_NODE_HPP_
#define REDUNDANCY_RELAY_MANAGER_NODE_HPP_

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <tier4_system_msgs/msg/election_status.hpp>

namespace autoware::redundancy_relay_manager
{
class RedundancyRelayManager : public rclcpp::Node
{
public:
  explicit RedundancyRelayManager(const rclcpp::NodeOptions & options);

private:
  // Subscribers
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    sub_operation_mode_state_;
  rclcpp::Subscription<tier4_system_msgs::msg::ElectionStatus>::SharedPtr sub_main_election_status_;
  rclcpp::Subscription<tier4_system_msgs::msg::ElectionStatus>::SharedPtr sub_sub_election_status_;

  // Callbacks
  void onOperationModeState(
    const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg);
  void onMainElectionStatus(const tier4_system_msgs::msg::ElectionStatus::SharedPtr msg);
  void onSubElectionStatus(const tier4_system_msgs::msg::ElectionStatus::SharedPtr msg);
};
}  // namespace autoware::redundancy_relay_manager

#endif  // REDUNDANCY_RELAY_MANAGER_NODE_HPP_
