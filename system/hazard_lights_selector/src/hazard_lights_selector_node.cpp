// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <hazard_lights_selector/hazard_lights_selector_node.hpp>

namespace hazard_lights_selector
{
HazardLightsSelectorNode::HazardLightsSelectorNode(const rclcpp::NodeOptions & node_options)
: Node("hazard_lights_selector", node_options)
{
  using std::placeholders::_1;

  // Parameter
  params_.update_rate = static_cast<int>(declare_parameter<int>("update_rate", 10));

  // Subscriber
  sub_hazard_lights_cmd_from_path_planner_ = this->create_subscription<HazardLightsCommand>(
    "~/input/behavior_path_planner/hazard_lights_cmd", 1,
    std::bind(&HazardLightsSelectorNode::onHazardLightsCommandFromPathPlanner, this, _1));
  sub_hazard_lights_cmd_from_mrm_operator_ = this->create_subscription<HazardLightsCommand>(
    "~/input/behavior_mrm_operator/hazard_lights_cmd", 1,
    std::bind(&HazardLightsSelectorNode::onHazardLightsCommandFromMrmOperator, this, _1));

  // Publisher
  pub_hazard_lights_cmd_ =
    this->create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&HazardLightsSelectorNode::onTimer, this));
}

void HazardLightsSelectorNode::onHazardLightsCommandFromPathPlanner(
  HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_command_from_path_planner_ = msg;
}

void HazardLightsSelectorNode::onHazardLightsCommandFromMrmOperator(
  HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_command_from_mrm_operator_ = msg;
}

void HazardLightsSelectorNode::onTimer()
{
  auto hazard_lights_cmd = HazardLightsCommand();
  hazard_lights_cmd.stamp = this->now();
  hazard_lights_cmd.command = HazardLightsCommand::DISABLE;

  if (hazard_lights_command_from_path_planner_ != nullptr) {
    if (hazard_lights_command_from_path_planner_->command == HazardLightsCommand::ENABLE) {
      hazard_lights_cmd.command = HazardLightsCommand::ENABLE;
    }
  }
  if (hazard_lights_command_from_mrm_operator_ != nullptr) {
    if (hazard_lights_command_from_mrm_operator_->command == HazardLightsCommand::ENABLE) {
      hazard_lights_cmd.command = HazardLightsCommand::ENABLE;
    }
  }

  pub_hazard_lights_cmd_->publish(hazard_lights_cmd);
}

}  // namespace hazard_lights_selector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hazard_lights_selector::HazardLightsSelectorNode)
