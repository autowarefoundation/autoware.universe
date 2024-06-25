// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#ifndef AUTOWARE__HMI__HMI_NODE_HPP_
#define AUTOWARE__HMI__HMI_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/mission.hpp"

#include <string>
#include <vector>

namespace autoware::mapless_architecture
{

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

  // Declare ROS2 publisher and subscriber

  rclcpp::Publisher<autoware_planning_msgs::msg::Mission>::SharedPtr mission_publisher_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
}  // namespace autoware::mapless_architecture

#endif  // AUTOWARE__HMI__HMI_NODE_HPP_
