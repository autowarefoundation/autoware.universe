// Copyright 2023 The Autoware Foundation
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

#include "autoware_auto_msgs_adapter/autoware_auto_msgs_adapter_core.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware_auto_msgs_adapter
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_control_msgs::msg::Control;

AutowareAutoMsgsAdapterNode::AutowareAutoMsgsAdapterNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("autoware_auto_msgs_adapter", node_options)
{
  using Adapter = rclcpp::TypeAdapter<Control, AckermannControlCommand>;

  pub_ackermann_control_command_ =
    create_publisher<AckermannControlCommand>("ackermann_control_command", rclcpp::QoS{1});

  sub_control_ = create_subscription<Adapter>(
    "control_command", rclcpp::QoS{1}, [this](const Control::SharedPtr msg) {
      AckermannControlCommand ackermann_control_command;
      rclcpp::TypeAdapter<Control, AckermannControlCommand>::convert_to_ros_message(
        *msg, ackermann_control_command);
      pub_ackermann_control_command_->publish(ackermann_control_command);
    });
}

}  // namespace autoware_auto_msgs_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_auto_msgs_adapter::AutowareAutoMsgsAdapterNode)
