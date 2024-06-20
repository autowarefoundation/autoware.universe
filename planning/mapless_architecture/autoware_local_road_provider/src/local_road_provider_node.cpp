// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license

#include "autoware/local_road_provider/local_road_provider_node.hpp"

#include "autoware/local_mission_planner_common/helper_functions.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

LocalRoadProviderNode::LocalRoadProviderNode() : Node("local_road_provider_node")
{
  // Set quality of service to best effort (if transmission fails, do not try to
  // resend but rather use new sensor data)
  // the history_depth is set to 1 (message queue size)
  auto qos = rclcpp::QoS(1);
  qos.best_effort();

  // Initialize publisher for road segments
  road_publisher_ = this->create_publisher<autoware_planning_msgs::msg::RoadSegments>(
    "local_road_provider_node/output/road_segments", 1);

  // Initialize subscriber to lanelets stamped messages
  lanelets_subscriber_ = this->create_subscription<db_msgs::msg::LaneletsStamped>(
    "local_road_provider_node/input/lanelets", qos,
    std::bind(&LocalRoadProviderNode::CallbackLaneletsMessages_, this, _1));
}

void LocalRoadProviderNode::CallbackLaneletsMessages_(const db_msgs::msg::LaneletsStamped & msg)
{
  autoware_planning_msgs::msg::RoadSegments road_segments =
    lib_mission_planner::ConvertLaneletsStamped2RoadSegments(msg);

  // Publish the RoadSegments message
  road_publisher_->publish(road_segments);
}
