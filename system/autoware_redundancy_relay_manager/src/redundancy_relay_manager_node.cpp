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

#include <chrono>
#include <memory>
#include <string>

namespace autoware::redundancy_relay_manager
{
RedundancyRelayManager::RedundancyRelayManager(const rclcpp::NodeOptions & options)
: Node("redundancy_relay_manager", options), is_relaying_(true), is_stopped_by_main_(true)
{
  // Params
  node_param_.service_timeout_ms = declare_parameter<int>("service_timeout_ms");

  // Subscribers
  sub_main_election_status_ = create_subscription<tier4_system_msgs::msg::ElectionStatus>(
    "~/input/main/election/status", rclcpp::QoS{1},
    std::bind(&RedundancyRelayManager::onMainElectionStatus, this, std::placeholders::_1));
  sub_sub_election_status_ = create_subscription<tier4_system_msgs::msg::ElectionStatus>(
    "~/input/sub/election/status", rclcpp::QoS{1},
    std::bind(&RedundancyRelayManager::onSubElectionStatus, this, std::placeholders::_1));

  // Clients
  client_relay_trajectory_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_relay_trajectory_ = create_client<tier4_system_msgs::srv::ChangeTopicRelayControl>(
    "~/output/topic_relay_controller_trajectory/operate", rmw_qos_profile_services_default,
    client_relay_trajectory_group_);
  client_relay_pose_with_covariance_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_relay_pose_with_covariance_ =
    create_client<tier4_system_msgs::srv::ChangeTopicRelayControl>(
      "~/output/topic_relay_controller_pose_with_covariance/operate",
      rmw_qos_profile_services_default, client_relay_pose_with_covariance_group_);
}

void RedundancyRelayManager::onMainElectionStatus(
  const tier4_system_msgs::msg::ElectionStatus::SharedPtr msg)
{
  const auto tmp_election_status = main_election_status_;
  main_election_status_ = msg;

  auto operation_mode_state = sub_operation_mode_state_.takeData();
  if (operation_mode_state == nullptr) return;

  if (is_relaying_) {
    if (tmp_election_status == nullptr) return;
    if (operation_mode_state->mode != autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS) return;
    if (((msg->path_info >> 3) & 0x01) == 1 || ((tmp_election_status->path_info >> 3) & 0x01) == 0) return;

    requestTopicRelayControl(false, client_relay_trajectory_, "topic_relay_control_trajectory");
    requestTopicRelayControl(
      false, client_relay_pose_with_covariance_, "topic_relay_control_pose_with_covariance");
    is_relaying_ = false;
    is_stopped_by_main_ = true;
  } else {
    if (((msg->path_info >> 3) & 0x01) == 1 && is_stopped_by_main_) {
      requestTopicRelayControl(true, client_relay_trajectory_, "topic_relay_control_trajectory");
      requestTopicRelayControl(
        true, client_relay_pose_with_covariance_, "topic_relay_control_pose_with_covariance");
      is_relaying_ = true;
    }
  }
}

void RedundancyRelayManager::onSubElectionStatus(
  const tier4_system_msgs::msg::ElectionStatus::SharedPtr msg)
{
  const auto tmp_election_status = sub_election_status_;
  sub_election_status_ = msg;

  auto operation_mode_state = sub_operation_mode_state_.takeData();
  if (operation_mode_state == nullptr) return;

  if (is_relaying_) {
    if (tmp_election_status == nullptr) return;
    if (operation_mode_state->mode != autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS) return;
    if (((msg->path_info >> 3) & 0x01) == 1 || ((tmp_election_status->path_info >> 3) & 0x01) == 0) return;

    requestTopicRelayControl(false, client_relay_trajectory_, "topic_relay_control_trajectory");
    requestTopicRelayControl(
      false, client_relay_pose_with_covariance_, "topic_relay_control_pose_with_covariance");
    is_relaying_ = false;
    is_stopped_by_main_ = false;
  } else {
    if (((msg->path_info >> 3) & 0x01) == 1 && !is_stopped_by_main_) {
      requestTopicRelayControl(true, client_relay_trajectory_, "topic_relay_control_trajectory");
      requestTopicRelayControl(
        true, client_relay_pose_with_covariance_, "topic_relay_control_pose_with_covariance");
      is_relaying_ = true;
    }
  }
}

void RedundancyRelayManager::requestTopicRelayControl(
  const bool relay_on,
  rclcpp::Client<tier4_system_msgs::srv::ChangeTopicRelayControl>::SharedPtr client,
  std::string srv_name)
{
  auto request = std::make_shared<tier4_system_msgs::srv::ChangeTopicRelayControl::Request>();
  request->relay_on = relay_on;

  const auto duration = std::chrono::milliseconds(node_param_.service_timeout_ms);
  auto future = client->async_send_request(request).future.share();

  if (future.wait_for(duration) == std::future_status::ready) {
    auto response = future.get();
    if (response->status.success) {
      RCLCPP_INFO(
        get_logger(), "Changed %s relay control: %s", srv_name.c_str(), relay_on ? "ON" : "OFF");
    } else {
      RCLCPP_ERROR(
        get_logger(), "Failed to change %s relay control: %s", srv_name.c_str(),
        relay_on ? "ON" : "OFF");
    }
  } else {
    RCLCPP_ERROR(
      get_logger(), "Service timeout %s relay control: %s", srv_name.c_str(),
      relay_on ? "ON" : "OFF");
  }
}
}  // namespace autoware::redundancy_relay_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::redundancy_relay_manager::RedundancyRelayManager)
