// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#ifndef HMI_NODE_HPP_
#define HMI_NODE_HPP_

#include "mission_planner_messages/msg/mission.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <vector>

/**
 * Node for HMI.
 */
class HMINode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the HMINode class.
   * Initializes the publisher and subscriber with appropriate topics and QoS
   * settings.
   */
  HMINode();

private:
  // ###########################################################################
  // #  PRIVATE PROCESSING METHODS
  // ###########################################################################

  /**
   * @brief Callback function for parameter changes.
   * This callback function is triggered whenever a ROS 2 parameter is changed.
   * @param parameters The received parameter changes.
   */
  rcl_interfaces::msg::SetParametersResult ParamCallback_(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Function which publishes the mission.
   *
   * @param mission The mission that should be published.
   */
  void PublishMission_(std::string mission);

  // ###########################################################################
  // #  PRIVATE VARIABLES
  // ###########################################################################
  // Declare ROS2 publisher and subscriber

  rclcpp::Publisher<mission_planner_messages::msg::Mission>::SharedPtr mission_publisher_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif  // HMI_NODE_HPP_
