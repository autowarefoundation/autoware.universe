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

#ifndef HAZARD_LIGHTS_SELECTOR__HAZARD_LIGHTS_SELECTOR_NODE_HPP_
#define HAZARD_LIGHTS_SELECTOR__HAZARD_LIGHTS_SELECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>

namespace hazard_lights_selector
{
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;

class HazardLightsSelectorNode : public rclcpp::Node
{
  struct Parameters
  {
    int update_rate;  // [Hz]
  };

public:
  explicit HazardLightsSelectorNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  Parameters params_;

  // Subscriber
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_cmd_from_path_planner_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_cmd_from_mrm_operator_;

  void onHazardLightsCommandFromPathPlanner(HazardLightsCommand::ConstSharedPtr msg);
  void onHazardLightsCommandFromMrmOperator(HazardLightsCommand::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr pub_hazard_lights_cmd_;

  void publishHazardLightsCommand() const;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();

  // State
  HazardLightsCommand::ConstSharedPtr hazard_lights_command_from_path_planner_;
  HazardLightsCommand::ConstSharedPtr hazard_lights_command_from_mrm_operator_;
};
}  // namespace hazard_lights_selector

#endif  // HAZARD_LIGHTS_SELECTOR__HAZARD_LIGHTS_SELECTOR_NODE_HPP_
