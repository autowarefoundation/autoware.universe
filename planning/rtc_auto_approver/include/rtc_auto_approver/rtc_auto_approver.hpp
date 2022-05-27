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

#ifndef RTC_AUTO_APPROVER__RTC_AUTO_APPROVER_HPP_
#define RTC_AUTO_APPROVER__RTC_AUTO_APPROVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_rtc_msgs/msg/command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_response.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/msg/module.hpp"
#include "tier4_rtc_msgs/srv/auto_mode.hpp"
#include "tier4_rtc_msgs/srv/cooperate_commands.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <string>
#include <vector>

namespace rtc_auto_approver
{
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateCommand;
using tier4_rtc_msgs::msg::CooperateResponse;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::srv::AutoMode;
using tier4_rtc_msgs::srv::CooperateCommands;
using unique_identifier_msgs::msg::UUID;

class RTCAutoApproverNode : public rclcpp::Node
{
public:
  explicit RTCAutoApproverNode(const rclcpp::NodeOptions & node_options);

private:
  // Callback for cooperate status
  void onBlindSpotStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onCrosswalkStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onDetectionAreaStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onIntersectionStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onNoStoppingAreaStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onTrafficLightStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onLaneChangeLeftStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onLaneChangeRightStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onAvoidanceLeftStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onAvoidanceRightStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onPullOverStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  void onPullOutStatus(const CooperateStatusArray::ConstSharedPtr msg) const;

  // Callback for auto mode
  void onBlindSpotEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onCrosswalkEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onDetectionAreaEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onIntersectionEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onNoStoppingAreaEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onTrafficLightEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onLaneChangeLeftEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onLaneChangeRightEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onAvoidanceLeftEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onAvoidanceRightEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onPullOverEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onPullOutEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);

  void onStatus(
    const CooperateStatusArray::ConstSharedPtr msg,
    const rclcpp::Client<CooperateCommands>::SharedPtr cli) const;
  bool isNecessarySendCommand(const CooperateStatus & status) const;
  CooperateCommands::Request createRequest(const CooperateStatusArray & array) const;

  /* subscribers */
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr blind_spot_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr crosswalk_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr detection_area_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr intersection_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr no_stopping_area_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr traffic_light_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr lane_change_left_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr lane_change_right_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr avoidance_left_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr avoidance_right_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr pull_over_sub_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr pull_out_sub_;

  /* service clients */
  rclcpp::Client<CooperateCommands>::SharedPtr blind_spot_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr crosswalk_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr detection_area_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr intersection_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr no_stopping_area_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr traffic_light_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr lane_change_left_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr lane_change_right_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr avoidance_left_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr avoidance_right_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr pull_over_cli_;
  rclcpp::Client<CooperateCommands>::SharedPtr pull_out_cli_;

  /* service */
  rclcpp::Service<AutoMode>::SharedPtr blind_spot_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr crosswalk_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr detection_area_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr intersection_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr no_stopping_area_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr traffic_light_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr lane_change_left_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr lane_change_right_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr avoidance_left_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr avoidance_right_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr pull_over_enable_srv_;
  rclcpp::Service<AutoMode>::SharedPtr pull_out_enable_srv_;

  bool blind_spot_enabled_;
  bool crosswalk_enabled_;
  bool detection_area_enabled_;
  bool intersection_enabled_;
  bool no_stopping_area_enabled_;
  bool traffic_light_enabled_;
  bool lane_change_left_enabled_;
  bool lane_change_right_enabled_;
  bool avoidance_left_enabled_;
  bool avoidance_right_enabled_;
  bool pull_over_enabled_;
  bool pull_out_enabled_;

  std::string BEHAVIOR_PLANNING_NAMESPACE =
    "/planning/scenario_planning/lane_driving/behavior_planning";
};

}  // namespace rtc_auto_approver

#endif  // RTC_AUTO_APPROVER__RTC_AUTO_APPROVER_HPP_
