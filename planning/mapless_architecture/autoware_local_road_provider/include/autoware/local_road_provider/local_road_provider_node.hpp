// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#ifndef AUTOWARE__LOCAL_ROAD_PROVIDER__LOCAL_ROAD_PROVIDER_NODE_HPP_
#define AUTOWARE__LOCAL_ROAD_PROVIDER__LOCAL_ROAD_PROVIDER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/road_segments.hpp"
#include "db_msgs/msg/lanelets_stamped.hpp"

namespace autoware::mapless_architecture
{

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
  /**
   * @brief The callback for the LaneletsStamped messages.
   *
   * @param msg The db_msgs::msg::LaneletsStamped message.
   */
  void CallbackLaneletsMessages_(const db_msgs::msg::LaneletsStamped & msg);

  /**
   * @brief Convert the LaneletsStamped message into a RoadSegments message.
   *
   * @param msg The message (db_msgs::msg::LaneletsStamped).
   * @return autoware_planning_msgs::msg::RoadSegments.
   */
  autoware_planning_msgs::msg::RoadSegments ConvertLaneletsStamped2RoadSegments(
    const db_msgs::msg::LaneletsStamped & msg);

  // Declare ROS2 publisher and subscriber

  rclcpp::Publisher<autoware_planning_msgs::msg::RoadSegments>::SharedPtr road_publisher_;

  rclcpp::Subscription<db_msgs::msg::LaneletsStamped>::SharedPtr lanelets_subscriber_;
};
}  // namespace autoware::mapless_architecture

#endif  // AUTOWARE__LOCAL_ROAD_PROVIDER__LOCAL_ROAD_PROVIDER_NODE_HPP_
