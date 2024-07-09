// Copyright 2024 driveblocks GmbH, authors: Simon Eisenmann, Thomas Herrmann
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/local_map_provider/local_map_provider_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace autoware::mapless_architecture
{
using std::placeholders::_1;

LocalMapProviderNode::LocalMapProviderNode(const rclcpp::NodeOptions & options)
: Node("local_map_provider_node", options)
{
  // Set quality of service to best effort (if transmission fails, do not try to
  // resend but rather use new sensor data)
  // the history_depth is set to 1 (message queue size)
  auto qos = rclcpp::QoS(1);
  qos.best_effort();

  // Initialize publisher for local map
  map_publisher_ = this->create_publisher<autoware_planning_msgs::msg::LocalMap>(
    "local_map_provider_node/output/local_map", 1);

  // Initialize subscriber to road segments messages
  road_subscriber_ = this->create_subscription<autoware_planning_msgs::msg::RoadSegments>(
    "local_map_provider_node/input/road_segments", qos,
    std::bind(&LocalMapProviderNode::CallbackRoadSegmentsMessages_, this, _1));
}

void LocalMapProviderNode::CallbackRoadSegmentsMessages_(
  const autoware_planning_msgs::msg::RoadSegments & msg)
{
  autoware_planning_msgs::msg::LocalMap local_map;

  // Save road segments in the local map message
  local_map.road_segments = msg;

  // Publish the LocalMap message
  map_publisher_->publish(
    local_map);  // Outlook: Add global map, sign detection etc. to the message
}
}  // namespace autoware::mapless_architecture

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mapless_architecture::LocalMapProviderNode)
