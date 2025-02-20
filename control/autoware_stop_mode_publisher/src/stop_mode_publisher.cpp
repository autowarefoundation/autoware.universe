//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "stop_mode_publisher.hpp"

namespace autoware::stop_mode_publisher
{

StopModePublisher::StopModePublisher(const rclcpp::NodeOptions & options)
: Node("stop_mode_publisher", options)
{
  const auto control_qos = rclcpp::QoS(5);
  const auto durable_qos = rclcpp::QoS(1).transient_local();

  pub_control_ = create_publisher<Control>("~/control", control_qos);
  pub_gear_ = create_publisher<GearCommand>("~/gear", durable_qos);
  pub_turn_indicators_ = create_publisher<TurnIndicatorsCommand>("~/turn_indicators", durable_qos);
  pub_hazard_lights_ = create_publisher<HazardLightsCommand>("~/hazard_lights", durable_qos);

  const auto period = rclcpp::Rate(declare_parameter<double>("rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void StopModePublisher::on_timer()
{
  pub_control_->publish(Control());
  pub_gear_->publish(GearCommand());
  pub_turn_indicators_->publish(TurnIndicatorsCommand());
  pub_hazard_lights_->publish(HazardLightsCommand());
}

}  // namespace autoware::stop_mode_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::stop_mode_publisher::StopModePublisher)
