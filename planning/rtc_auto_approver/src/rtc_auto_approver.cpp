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

#include "rtc_auto_approver/rtc_auto_approver.hpp"

namespace rtc_auto_approver
{

RTCAutoApproverNode::RTCAutoApproverNode(const rclcpp::NodeOptions & node_options)
: Node("rtc_auto_approver_node", node_options)
{
  using std::placeholders::_1;

  // Subscrivers
  blind_spot_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/blind_spot/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onBlindSpotStatus, this, _1));
  crosswalk_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/crosswalk/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onCrosswalkStatus, this, _1));
  detection_area_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/detection_area/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onDetectionAreaStatus, this, _1));
  intersection_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/intersection/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onIntersectionStatus, this, _1));
  no_stopping_area_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/no_stopping_area/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onNoStoppingAreaStatus, this, _1));
  traffic_light_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/traffic_light/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onTrafficLightStatus, this, _1));
  lane_change_left_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/lane_change_left/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onLaneChangeLeftStatus, this, _1));
  lane_change_right_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/lane_change_right/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onLaneChangeRightStatus, this, _1));
  avoidance_left_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/avoidance_left/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onAvoidanceLeftStatus, this, _1));
  avoidance_right_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/avoidance_right/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onAvoidanceRightStatus, this, _1));
  pull_over_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/pull_over/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onPullOverStatus, this, _1));
  pull_out_sub_ = create_subscription<CooperateStatusArray>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_path_planner/pull_out/cooperate_status",
    rclcpp::QoS(1), std::bind(&RTCAutoApproverNode::onPullOutStatus, this, _1));

  // Service clients
  blind_spot_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/blind_spot/cooperate_commands",
    rmw_qos_profile_services_default);
  crosswalk_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/crosswalk/cooperate_commands",
    rmw_qos_profile_services_default);
  detection_area_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/detection_area/cooperate_commands",
    rmw_qos_profile_services_default);
  intersection_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/intersection/cooperate_commands",
    rmw_qos_profile_services_default);
  no_stopping_area_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/no_stopping_area/cooperate_commands",
    rmw_qos_profile_services_default);
  traffic_light_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/traffic_light/cooperate_commands",
    rmw_qos_profile_services_default);
  lane_change_left_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/lane_change_left/cooperate_commands",
    rmw_qos_profile_services_default);
  lane_change_right_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/lane_change_right/cooperate_commands",
    rmw_qos_profile_services_default);
  avoidance_left_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/avoidance_left/cooperate_commands",
    rmw_qos_profile_services_default);
  avoidance_right_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/avoidance_right/cooperate_commands",
    rmw_qos_profile_services_default);
  pull_over_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/pull_over/cooperate_commands",
    rmw_qos_profile_services_default);
  pull_out_cli_ = create_client<CooperateCommands>(
    BEHAVIOR_PLANNING_NAMESPACE + "/behavior_velocity_planner/pull_out/cooperate_commands",
    rmw_qos_profile_services_default);
}

void RTCAutoApproverNode::onBlindSpotStatus(const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, blind_spot_cli_);
}

void RTCAutoApproverNode::onCrosswalkStatus(const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, crosswalk_cli_);
}

void RTCAutoApproverNode::onDetectionAreaStatus(
  const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, detection_area_cli_);
}

void RTCAutoApproverNode::onIntersectionStatus(const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, intersection_cli_);
}

void RTCAutoApproverNode::onNoStoppingAreaStatus(
  const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, no_stopping_area_cli_);
}

void RTCAutoApproverNode::onTrafficLightStatus(const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, traffic_light_cli_);
}

void RTCAutoApproverNode::onLaneChangeLeftStatus(
  const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, lane_change_left_cli_);
}

void RTCAutoApproverNode::onLaneChangeRightStatus(
  const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, lane_change_right_cli_);
}

void RTCAutoApproverNode::onAvoidanceLeftStatus(
  const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, avoidance_left_cli_);
}

void RTCAutoApproverNode::onAvoidanceRightStatus(
  const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, avoidance_right_cli_);
}

void RTCAutoApproverNode::onPullOverStatus(const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, pull_over_cli_);
}

void RTCAutoApproverNode::onPullOutStatus(const CooperateStatusArray::ConstSharedPtr msg) const
{
  onStatus(msg, pull_out_cli_);
}

void RTCAutoApproverNode::onStatus(
  const CooperateStatusArray::ConstSharedPtr msg,
  const rclcpp::Client<CooperateCommands>::SharedPtr cli) const
{
  if (!msg) {
    return;
  }

  const auto request = std::make_shared<CooperateCommands::Request>(createRequest(*msg));

  if (!request->commands.empty()) {
    cli->async_send_request(request);
  }
}

bool RTCAutoApproverNode::isNecessarySendCommand(const CooperateStatus & status) const
{
  const bool is_activate = (status.command_status.type == Command::ACTIVATE);
  if (status.safe && !is_activate) {
    return true;
  }
  if (!status.safe && is_activate) {
    return true;
  }
  return false;
}

CooperateCommands::Request RTCAutoApproverNode::createRequest(
  const CooperateStatusArray & array) const
{
  CooperateCommands::Request request;
  request.stamp = array.stamp;

  for (const auto & status : array.statuses) {
    if (isNecessarySendCommand(status)) {
      CooperateCommand cmd;
      cmd.module = status.module;
      cmd.uuid = status.uuid;
      if (status.command_status.type == Command::DEACTIVATE) {
        cmd.command.type = Command::ACTIVATE;
        request.commands.push_back(cmd);
      } else if (status.command_status.type == Command::ACTIVATE) {
        cmd.command.type = Command::DEACTIVATE;
        request.commands.push_back(cmd);
      }
    }
  }

  return request;
}

}  // namespace rtc_auto_approver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rtc_auto_approver::RTCAutoApproverNode)
