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

#include <chrono>
#include <utility>
#include <memory>

#include "external_cmd_selector/external_cmd_selector_node.hpp"

ExternalCmdSelector::ExternalCmdSelector(const rclcpp::NodeOptions & node_options)
: Node("external_cmd_selector", node_options)
{
  // Parameter
  update_rate_ = declare_parameter("update_rate", 10.0);
  initial_selector_mode_ = declare_parameter("initial_selector_mode", 0);

  // Publisher
  pub_current_selector_mode_ =
    create_publisher<autoware_control_msgs::msg::RemoteCommandSelectorMode>(
    "~/output/current_selector_mode", 1);

  pub_external_control_cmd_ =
    create_publisher<autoware_vehicle_msgs::msg::ExternalControlCommandStamped>(
    "~/output/external_control_cmd", 1);
  pub_shift_cmd_ = create_publisher<autoware_vehicle_msgs::msg::ShiftStamped>(
    "~/output/shift_cmd",
    1);
  pub_turn_signal_cmd_ =
    create_publisher<autoware_vehicle_msgs::msg::TurnSignal>("~/output/turn_signal_cmd", 1);

  // Callback Groups
  callback_group_subscribers_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_services_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  // Subscriber
  sub_local_control_cmd_ =
    create_subscription<autoware_vehicle_msgs::msg::ExternalControlCommandStamped>(
    "~/input/local/control_cmd", 1, std::bind(
      &ExternalCmdSelector::onLocalControlCmd, this,
      _1), subscriber_option);
  sub_local_shift_cmd_ =
    create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "~/input/local/shift_cmd", 1, std::bind(
      &ExternalCmdSelector::onLocalShiftCmd, this,
      _1), subscriber_option);
  sub_local_turn_signal_cmd_ = create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "~/input/local/turn_signal_cmd", 1,
    std::bind(&ExternalCmdSelector::onLocalTurnSignalCmd, this, _1), subscriber_option);

  sub_remote_control_cmd_ =
    create_subscription<autoware_vehicle_msgs::msg::ExternalControlCommandStamped>(
    "~/input/remote/control_cmd", 1, std::bind(
      &ExternalCmdSelector::onRemoteControlCmd, this,
      _1), subscriber_option);
  sub_remote_shift_cmd_ =
    create_subscription<autoware_vehicle_msgs::msg::ShiftStamped>(
    "~/input/remote/shift_cmd", 1, std::bind(
      &ExternalCmdSelector::onRemoteShiftCmd, this,
      _1));
  sub_remote_turn_signal_cmd_ = create_subscription<autoware_vehicle_msgs::msg::TurnSignal>(
    "~/input/remote/turn_signal_cmd", 1,
    std::bind(&ExternalCmdSelector::onRemoteTurnSignalCmd, this, _1), subscriber_option);

  // Service
  srv_select_external_command_ = create_service<autoware_control_msgs::srv::RemoteCommandSelect>(
    "~/service/select_external_command",
    std::bind(
      &ExternalCmdSelector::onSelectRemoteCommandService, this, _1, _2,
      _3), rmw_qos_profile_services_default, callback_group_services_);

  // Initialize mode
  current_selector_mode_.data = initial_selector_mode_;

  // Timer
  auto timer_callback = std::bind(&ExternalCmdSelector::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    get_clock(), period, std::move(timer_callback),
    get_node_base_interface()->get_context());
  get_node_timers_interface()->add_timer(timer_, callback_group_subscribers_);
}

void ExternalCmdSelector::onLocalControlCmd(
  const autoware_vehicle_msgs::msg::ExternalControlCommandStamped::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != autoware_control_msgs::msg::RemoteCommandSelectorMode::LOCAL) {
    return;
  }

  pub_external_control_cmd_->publish(*msg);
}

void ExternalCmdSelector::onLocalShiftCmd(
  const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != autoware_control_msgs::msg::RemoteCommandSelectorMode::LOCAL) {
    return;
  }

  pub_shift_cmd_->publish(*msg);
}

void ExternalCmdSelector::onLocalTurnSignalCmd(
  const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != autoware_control_msgs::msg::RemoteCommandSelectorMode::LOCAL) {
    return;
  }

  pub_turn_signal_cmd_->publish(*msg);
}

void ExternalCmdSelector::onRemoteControlCmd(
  const autoware_vehicle_msgs::msg::ExternalControlCommandStamped::ConstSharedPtr msg)
{
  if (current_selector_mode_.data !=
    autoware_control_msgs::msg::RemoteCommandSelectorMode::REMOTE)
  {
    return;
  }

  pub_external_control_cmd_->publish(*msg);
}

void ExternalCmdSelector::onRemoteShiftCmd(
  const autoware_vehicle_msgs::msg::ShiftStamped::ConstSharedPtr msg)
{
  if (current_selector_mode_.data !=
    autoware_control_msgs::msg::RemoteCommandSelectorMode::REMOTE)
  {
    return;
  }

  pub_shift_cmd_->publish(*msg);
}

void ExternalCmdSelector::onRemoteTurnSignalCmd(
  const autoware_vehicle_msgs::msg::TurnSignal::ConstSharedPtr msg)
{
  if (current_selector_mode_.data !=
    autoware_control_msgs::msg::RemoteCommandSelectorMode::REMOTE)
  {
    return;
  }

  pub_turn_signal_cmd_->publish(*msg);
}

bool ExternalCmdSelector::onSelectRemoteCommandService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const autoware_control_msgs::srv::RemoteCommandSelect::Request::SharedPtr req,
  const autoware_control_msgs::srv::RemoteCommandSelect::Response::SharedPtr res)
{
  current_selector_mode_.data = req->mode.data;
  res->success = true;
  res->message = "Success.";

  return true;
}

void ExternalCmdSelector::onTimer()
{
  pub_current_selector_mode_->publish(current_selector_mode_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExternalCmdSelector)
