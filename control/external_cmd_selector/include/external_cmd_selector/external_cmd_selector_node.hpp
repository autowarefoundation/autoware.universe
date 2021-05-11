// Copyright 2020 Tier IV, Inc.
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

#ifndef EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
#define EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/srv/remote_command_select.hpp"
#include "autoware_control_msgs/msg/remote_command_selector_mode.hpp"
#include "autoware_vehicle_msgs/msg/raw_control_command_stamped.hpp"
#include "autoware_vehicle_msgs/msg/shift_stamped.hpp"
#include "autoware_vehicle_msgs/msg/turn_signal.hpp"

class ExternalCmdSelector : public rclcpp::Node
{
public:
  explicit ExternalCmdSelector(const rclcpp::NodeOptions & node_options);

private:
  // Publisher
  rclcpp::Publisher<autoware_control_msgs::msg::RemoteCommandSelectorMode>::SharedPtr
    pub_current_selector_mode_;

  rclcpp::Publisher<autoware_vehicle_msgs::msg::RawControlCommandStamped>::SharedPtr
    pub_raw_control_cmd_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr pub_shift_cmd_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr pub_turn_signal_cmd_;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Subscriber
  rclcpp::Subscription<autoware_vehicle_msgs::msg::RawControlCommandStamped>::SharedPtr
    sub_local_raw_control_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr sub_local_shift_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr sub_local_turn_signal_cmd_; //NOLINT

  rclcpp::Subscription<autoware_vehicle_msgs::msg::RawControlCommandStamped>::SharedPtr
    sub_remote_raw_control_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ShiftStamped>::SharedPtr sub_remote_shift_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnSignal>::SharedPtr
    sub_remote_turn_signal_cmd_;

  void onSelectorModeCmd(
    const autoware_control_msgs::msg::RemoteCommandSelectorMode::ConstSharedPtr msg);

  void onLocalRawControlCmd(
    const autoware_vehicle_msgs::msg::RawControlCommandStamped::ConstSharedPtr msg);
  void onLocalShiftCmd(const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);
  void onLocalTurnSignalCmd(const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);

  void onRemoteRawControlCmd(
    const autoware_vehicle_msgs::msg::RawControlCommandStamped::ConstSharedPtr msg);
  void onRemoteShiftCmd(const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg);
  void onRemoteTurnSignalCmd(const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg);

  // Service
  rclcpp::Service<autoware_control_msgs::srv::RemoteCommandSelect>::SharedPtr
    srv_select_external_command_;
  autoware_control_msgs::msg::RemoteCommandSelectorMode current_selector_mode_;

  bool onSelectRemoteCommandService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const autoware_control_msgs::srv::RemoteCommandSelect::Request::SharedPtr req,
    const autoware_control_msgs::srv::RemoteCommandSelect::Response::SharedPtr res);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();

  // Parameter
  double update_rate_;
  int initial_selector_mode_;
};

#endif  // EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
