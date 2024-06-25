// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#ifndef AUTOWARE__LOCAL_MAP_PROVIDER__LOCAL_MAP_PROVIDER_NODE_HPP_
#define AUTOWARE__LOCAL_MAP_PROVIDER__LOCAL_MAP_PROVIDER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/local_map.hpp"
#include "autoware_planning_msgs/msg/road_segments.hpp"

namespace autoware::mapless_architecture
{

/**
 * Node for the Local Map Provider.
 */
class LocalMapProviderNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the LocalMapProviderNode class.
   *
   * Initializes the publisher and subscriber with appropriate topics and QoS
   * settings.
   */
  LocalMapProviderNode();

private:
  /**
   * @brief The callback for the RoadSegments messages.
   *
   * @param msg The autoware_planning_msgs::msg::RoadSegments message.
   */
  void CallbackRoadSegmentsMessages_(const autoware_planning_msgs::msg::RoadSegments & msg);

  // Declare ROS2 publisher and subscriber

  rclcpp::Publisher<autoware_planning_msgs::msg::LocalMap>::SharedPtr map_publisher_;

  rclcpp::Subscription<autoware_planning_msgs::msg::RoadSegments>::SharedPtr road_subscriber_;
};
}  // namespace autoware::mapless_architecture

#endif  // AUTOWARE__LOCAL_MAP_PROVIDER__LOCAL_MAP_PROVIDER_NODE_HPP_
