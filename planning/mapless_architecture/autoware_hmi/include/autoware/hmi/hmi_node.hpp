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
  explicit HMINode(const rclcpp::NodeOptions & options);

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
