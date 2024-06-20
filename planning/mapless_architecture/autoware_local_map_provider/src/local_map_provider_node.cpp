// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license

#include "autoware/local_map_provider/local_map_provider_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace autoware::mapless_architecture
{
using std::placeholders::_1;

LocalMapProviderNode::LocalMapProviderNode() : Node("local_map_provider_node")
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
