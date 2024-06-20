// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#ifndef LOCAL_MAP_PROVIDER_NODE_HPP_
#define LOCAL_MAP_PROVIDER_NODE_HPP_

#include "autoware_planning_msgs/msg/local_map.hpp"
#include "autoware_planning_msgs/msg/road_segments.hpp"
#include "rclcpp/rclcpp.hpp"

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
  // ###########################################################################
  // #  PRIVATE PROCESSING METHODS
  // ###########################################################################

  /**
   * @brief The callback for the RoadSegments messages.
   *
   * @param msg The autoware_planning_msgs::msg::RoadSegments message.
   */
  void CallbackRoadSegmentsMessages_(const autoware_planning_msgs::msg::RoadSegments & msg);

  // ###########################################################################
  // #  PRIVATE VARIABLES
  // ###########################################################################
  // Declare ROS2 publisher and subscriber

  rclcpp::Publisher<autoware_planning_msgs::msg::LocalMap>::SharedPtr map_publisher_;

  rclcpp::Subscription<autoware_planning_msgs::msg::RoadSegments>::SharedPtr road_subscriber_;
};

#endif  // LOCAL_MAP_PROVIDER_NODE_HPP_
