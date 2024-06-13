// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#ifndef LOCAL_ROAD_PROVIDER_NODE_HPP_
#define LOCAL_ROAD_PROVIDER_NODE_HPP_

#include "mission_planner_messages/msg/road_segments.hpp"
#include "rclcpp/rclcpp.hpp"

#include "db_msgs/msg/lanelets_stamped.hpp"

/**
 * Node for the Local Road Provider.
 */
class LocalRoadProviderNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the LocalRoadProviderNode class.
   *
   * Initializes the publisher and subscriber with appropriate topics and QoS
   * settings.
   */
  LocalRoadProviderNode();

private:
  // ###########################################################################
  // #  PRIVATE PROCESSING METHODS
  // ###########################################################################

  /**
   * @brief The callback for the LaneletsStamped messages.
   *
   * @param msg The db_msgs::msg::LaneletsStamped message.
   */
  void CallbackLaneletsMessages_(const db_msgs::msg::LaneletsStamped & msg);

  // ###########################################################################
  // #  PRIVATE VARIABLES
  // ###########################################################################
  // Declare ROS2 publisher and subscriber

  rclcpp::Publisher<mission_planner_messages::msg::RoadSegments>::SharedPtr road_publisher_;

  rclcpp::Subscription<db_msgs::msg::LaneletsStamped>::SharedPtr lanelets_subscriber_;
};

#endif  // LOCAL_ROAD_PROVIDER_NODE_HPP_
