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

#include "autoware/hmi/hmi_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/mission.hpp"

namespace autoware::mapless_architecture
{
using std::placeholders::_1;

HMINode::HMINode(const rclcpp::NodeOptions & options) : Node("hmi_node", options)
{
  // Set quality of service to best effort (if transmission fails, do not try to
  // resend but rather use new sensor data)
  // the history_depth is set to 1 (message queue size)
  auto qos = rclcpp::QoS(1);
  qos.best_effort();

  // Declare parameter
  this->declare_parameter("mission", "LANE_KEEP");

  // Initialize publisher
  mission_publisher_ =
    this->create_publisher<autoware_planning_msgs::msg::Mission>("hmi_node/output/mission", 1);

  // Initialize parameters callback handle
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HMINode::ParamCallback_, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult HMINode::ParamCallback_(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // Initialize output
  rcl_interfaces::msg::SetParametersResult result;

  result.successful = false;
  result.reason = "";
  std::string mission;
  for (const auto & param : parameters) {
    if (param.get_name() == "mission") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        mission = param.as_string();

        // Publish mission
        PublishMission_(mission);

        result.successful = true;
      } else {
        result.reason = "Incorrect Type";
      }
    }
  }
  return result;
}

void HMINode::PublishMission_(std::string mission)
{
  autoware_planning_msgs::msg::Mission missionMessage;
  if (mission == "LANE_KEEP") {
    missionMessage.mission_type = autoware_planning_msgs::msg::Mission::LANE_KEEP;
  } else if (mission == "LANE_CHANGE_LEFT") {
    missionMessage.mission_type = autoware_planning_msgs::msg::Mission::LANE_CHANGE_LEFT;
  } else if (mission == "LANE_CHANGE_RIGHT") {
    missionMessage.mission_type = autoware_planning_msgs::msg::Mission::LANE_CHANGE_RIGHT;
  } else if (mission == "TAKE_NEXT_EXIT_LEFT") {
    missionMessage.mission_type = autoware_planning_msgs::msg::Mission::TAKE_NEXT_EXIT_LEFT;
  } else if (mission == "TAKE_NEXT_EXIT_RIGHT") {
    missionMessage.mission_type = autoware_planning_msgs::msg::Mission::TAKE_NEXT_EXIT_RIGHT;
  }

  // TODO(simon.eisenmann@driveblocks.ai): Change deadline parameter
  missionMessage.deadline = 1000;

  mission_publisher_->publish(missionMessage);
}
}  // namespace autoware::mapless_architecture

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mapless_architecture::HMINode)
