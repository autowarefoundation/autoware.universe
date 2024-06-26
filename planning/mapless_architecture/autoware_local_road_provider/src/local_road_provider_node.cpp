// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license

#include "autoware/local_road_provider/local_road_provider_node.hpp"

#include "autoware/local_mission_planner_common/helper_functions.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autoware::mapless_architecture
{
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

autoware_planning_msgs::msg::RoadSegments
LocalRoadProviderNode::ConvertLaneletsStamped2RoadSegments(
  const db_msgs::msg::LaneletsStamped & msg)
{
  // Declare a static logger
  static rclcpp::Logger static_logger = rclcpp::get_logger("static_logger");

  // Initialize road segments message
  autoware_planning_msgs::msg::RoadSegments road_segments;

  // Fill message header and pose
  road_segments.header = msg.header;
  road_segments.pose = msg.pose;

  // Convert lanelets to segments if lanelets are not empty
  if (!msg.lanelets.empty()) {
    for (const db_msgs::msg::Lanelet & lanelet : msg.lanelets) {
      // Initialize a segment
      autoware_planning_msgs::msg::Segment segment;

      // Fill the segment with basic information
      segment.id = lanelet.id;
      segment.successor_lanelet_id = lanelet.successor_lanelet_id;
      segment.neighboring_lanelet_id = lanelet.neighboring_lanelet_id;

      // Copy linestrings data
      for (int i = 0; i < 2; ++i) {
        // Copy points from the original linestring to the new one if points are not empty
        if (!lanelet.linestrings[i].points.empty()) {
          segment.linestrings[i].poses.reserve(lanelet.linestrings[i].points.size());

          for (const db_msgs::msg::DBPoint & point : lanelet.linestrings[i].points) {
            segment.linestrings[i].poses.push_back(point.pose);
          }
        } else {
          RCLCPP_WARN(
            static_logger,
            "Linestring does not contain points (ConvertLaneletsStamped2RoadSegments)!");
        }
      }

      // Add the filled segment to the road_segments message
      road_segments.segments.push_back(segment);
    }
  }

  return road_segments;
}

void LocalRoadProviderNode::CallbackLaneletsMessages_(const db_msgs::msg::LaneletsStamped & msg)
{
  autoware_planning_msgs::msg::RoadSegments road_segments =
    ConvertLaneletsStamped2RoadSegments(msg);

  // Publish the RoadSegments message
  road_publisher_->publish(road_segments);
}

}  // namespace autoware::mapless_architecture
