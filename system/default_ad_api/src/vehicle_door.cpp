// Copyright 2023 TIER IV, Inc.
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

#include "vehicle_door.hpp"

namespace default_ad_api
{

VehicleDoorNode::VehicleDoorNode(const rclcpp::NodeOptions & options)
: Node("vehicle_door", options)
{
  const auto on_command = [this](auto, auto res) { res->status.success = true; };

  const auto on_layout = [this](auto, auto res) { res->status.success = true; };

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_srv(srv_command_, on_command);
  adaptor.init_srv(srv_layout_, on_layout);
  adaptor.init_pub(pub_status_);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::VehicleDoorNode)
