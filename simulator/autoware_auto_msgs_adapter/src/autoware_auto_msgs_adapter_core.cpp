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

#include "autoware_auto_msgs_adapter/adapter_control.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware_auto_msgs_adapter
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_control_msgs::msg::Control;

AutowareAutoMsgsAdapterNode::AutowareAutoMsgsAdapterNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("autoware_auto_msgs_adapter", node_options)
{
  std::string msg_type_target = declare_parameter<std::string>("msg_type_target");
  std::string topic_name_source = declare_parameter<std::string>("topic_name_source");
  std::string topic_name_target = declare_parameter<std::string>("topic_name_target");

  if (msg_type_target == "autoware_auto_control_msgs::msg::AckermannControlCommand") {
    AdapterControl::SharedPtr adapter =
      std::make_shared<AdapterControl>(*this, topic_name_source, topic_name_target);
    adapter_ = std::dynamic_pointer_cast<AdapterBaseInterface>(adapter);
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown msg type: %s", msg_type_target.c_str());
  }
}

}  // namespace autoware_auto_msgs_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_auto_msgs_adapter::AutowareAutoMsgsAdapterNode)
