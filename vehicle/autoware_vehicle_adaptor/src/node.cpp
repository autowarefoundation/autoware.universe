//  Copyright 2024 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autoware_vehicle_adaptor/node.hpp"

#include <memory>
#include <string>

namespace autoware::vehicle_adaptor
{
VehicleAdaptorNode::VehicleAdaptorNode(const rclcpp::NodeOptions & node_options)
: Node("autoware_vehicle_adaptor_node", node_options)
{
  using std::placeholders::_1;
  sub_control_cmd_ = create_subscription<Control>(
    "~/input/control_cmd", 1, std::bind(&VehicleAdaptorNode::onControlCmd, this, _1));
  pub_control_cmd_ = create_publisher<Control>("~/output/control_cmd", 1);

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
}

void VehicleAdaptorNode::onControlCmd(const Control::ConstSharedPtr msg)
{
  // TODO(someone): implement your logic here
  pub_control_cmd_->publish(*msg);
}

}  // namespace autoware::vehicle_adaptor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vehicle_adaptor::VehicleAdaptorNode)
