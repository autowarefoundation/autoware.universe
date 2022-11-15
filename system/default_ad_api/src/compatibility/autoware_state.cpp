// Copyright 2022 TIER IV, Inc.
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

#include "autoware_state.hpp"

#include <string>
#include <vector>

namespace default_ad_api
{

AutowareStateNode::AutowareStateNode(const rclcpp::NodeOptions & options)
: Node("autoware_state", options)
{
  const std::vector<std::string> module_names = {
    "sensing", "perception", "map", "localization", "planning", "control", "vehicle", "system",
  };

  for (size_t i = 0; i < module_names.size(); ++i) {
    const auto name = "/system/component_state_monitor/component/launch/" + module_names[i];
    const auto qos = rclcpp::QoS(1).transient_local();
    const auto callback = [this, i](const ModeChangeAvailable::ConstSharedPtr msg) {
      launch_states_[i] = msg->available;
    };
    sub_launch_states_.push_back(create_subscription<ModeChangeAvailable>(name, qos, callback));
  }
  launch_states_.resize(module_names.size());

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_sub(sub_localization_, [this](const LocalizationState::ConstSharedPtr msg) {
    localization_ = *msg;
  });
  // adaptor.init_sub(sub_routing_, on_interface_version);
  // adaptor.init_sub(sub_operation_mode_, on_interface_version);

  const auto rate = rclcpp::Rate(1.0);
  // const auto rate = rclcpp::Rate(5.0);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });

  launch_ = LaunchState::Initializing;
  localization_.state = LocalizationState::UNKNOWN;
  routing_.state = RoutingState::UNKNOWN;
  operation_mode_.mode = OperationModeState::UNKNOWN;
}

void AutowareStateNode::on_timer()
{
  using autoware_auto_system_msgs::msg::AutowareState;

  const auto convert_state = [this]() -> uint8_t {
    if (launch_ == LaunchState::Initializing) {
      return AutowareState::INITIALIZING;
    }
    if (launch_ == LaunchState::Finalizing) {
      return AutowareState::FINALIZING;
    }
    if (localization_.state != LocalizationState::INITIALIZED) {
      return AutowareState::INITIALIZING;
    }
    if (routing_.state == RoutingState::UNSET) {
      return AutowareState::WAITING_FOR_ROUTE;
    }
    if (routing_.state == RoutingState::ARRIVED) {
      return AutowareState::ARRIVED_GOAL;
    }
    if (operation_mode_.mode != OperationModeState::STOP) {
      return AutowareState::DRIVING;
    }
    if (operation_mode_.is_autonomous_mode_available) {
      return AutowareState::WAITING_FOR_ENGAGE;
    }
    return AutowareState::PLANNING;
  };

  if (launch_ == LaunchState::Initializing) {
    bool is_initialized = true;
    for (const auto & state : launch_states_) {
      is_initialized &= state;
    }
    if (is_initialized) {
      launch_ = LaunchState::Running;
    }
  }

  const auto state = convert_state();
  RCLCPP_INFO_STREAM(get_logger(), (int)state);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::AutowareStateNode)
