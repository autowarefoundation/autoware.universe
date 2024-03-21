// Copyright 2022 The Autoware Foundation
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

#include "base_interface/base_interface_node.hpp"

#include <chrono>
#include <functional>
#include <string>
namespace autoware
{
namespace vehicle
{
namespace interface
{
using FeatureSet = BaseInterface::FeatureSet;

BaseInterfaceNode::BaseInterfaceNode(
  const std::string & node_name, const FeatureSet & features, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  this->features() = features;
  // Declare required pubs, sub and service
  m_command_sub = create_subscription<AckermannControlCommand>(
    "control_cmd", 1, std::bind(&BaseInterfaceNode::on_command, this, _1));
  m_mode_service = create_service<autoware_auto_vehicle_msgs::srv::AutonomyModeChange>(
    "autonomy_mode", std::bind(&BaseInterfaceNode::on_mode_change_request, this, _1, _2));
  m_steering_pub = create_publisher<SteeringReport>("steering_report", 1);
  m_velocity_pub = create_publisher<VelocityReport>("velocity_report", 1);

  // Declare optional pubs and subs
  for (const auto & feat : features) {
    if (InterfaceFeature::GEAR == feat) {
      m_gear_pub = create_publisher<GearReport>("gear_report", 1);
      m_gear_sub = create_subscription<GearCommand>(
        "gear_cmd", 1, [this](GearCommand::SharedPtr msg) { send_gear_command(*msg); });
    } else if (InterfaceFeature::HAND_BRAKE == feat) {
      m_hand_brake_pub = create_publisher<HandBrakeReport>("hand_brake_report", 1);
      m_hand_brake_sub = create_subscription<HandBrakeCommand>(
        "hand_brake_cmd", 1,
        [this](HandBrakeCommand::SharedPtr msg) { send_hand_brake_command(*msg); });
    } else if (InterfaceFeature::HAZARD_LIGHTS == feat) {
      m_hazard_lights_pub = create_publisher<HazardLightsReport>("hazard_lights_report", 1);
      m_hazard_lights_sub = create_subscription<HazardLightsCommand>(
        "hazard_lights_cmd", 1,
        [this](HazardLightsCommand::SharedPtr msg) { send_hazard_lights_command(*msg); });
    } else if (InterfaceFeature::HEADLIGHTS == feat) {
      m_headlights_pub = create_publisher<HeadlightsReport>("headlights_report", 1);
      m_headlights_sub = create_subscription<HeadlightsCommand>(
        "headlights_cmd", 1,
        [this](HeadlightsCommand::SharedPtr msg) { send_headlights_command(*msg); });
    } else if (InterfaceFeature::HORN == feat) {
      m_horn_pub = create_publisher<HornReport>("horn_report", 1);
      m_horn_sub = create_subscription<HornCommand>(
        "horn_cmd", 1, [this](HornCommand::SharedPtr msg) { send_horn_command(*msg); });
    } else if (InterfaceFeature::ODOMETRY == feat) {
      m_odometry_pub = create_publisher<VehicleOdometry>("odom", 1);
    } else if (InterfaceFeature::TURN_INDICATORS == feat) {
      m_turn_indicators_pub = create_publisher<TurnIndicatorsReport>("turn_indicators_report", 1);
      m_turn_indicators_sub = create_subscription<TurnIndicatorsCommand>(
        "turn_indicators_cmd", 1,
        [this](TurnIndicatorsCommand::SharedPtr msg) { send_turn_indicators_command(*msg); });
    } else if (InterfaceFeature::WIPERS == feat) {
      m_wipers_pub = create_publisher<WipersReport>("wipers_report", 1);
      m_wipers_sub = create_subscription<WipersCommand>(
        "wipers_cmd", 1, [this](WipersCommand::SharedPtr msg) { send_wipers_command(*msg); });
    }
  }

  // Start querying reports
  m_report_timer = create_wall_timer(
    std::chrono::milliseconds(
      static_cast<uint64_t>(declare_parameter<double>("report_interval_sec", 0.01) * 1000.0)),
    std::bind(&BaseInterfaceNode::on_report_timer, this));
}

void BaseInterfaceNode::on_command(AckermannControlCommand::SharedPtr msg)
{
  send_control_command(*msg);
  // TODO(haoru): handle send_control_command failures
}

void BaseInterfaceNode::on_mode_change_request(
  ModeChangeRequest::SharedPtr request, ModeChangeResponse::SharedPtr response)
{
  handle_mode_change_request(request);
  (void)response;
  // TODO(haoru): handle mode change failures
}

void BaseInterfaceNode::on_report_timer()
{
  m_velocity_pub->publish(velocity_report());
  m_steering_pub->publish(steering_report());
  for (const auto & feat : features()) {
    if (InterfaceFeature::GEAR == feat && m_gear_pub) {
      m_gear_pub->publish(gear_report());
    } else if (InterfaceFeature::HAND_BRAKE == feat && m_hand_brake_pub) {
      m_hand_brake_pub->publish(hand_brake_report());
    } else if (InterfaceFeature::HAZARD_LIGHTS == feat && m_hazard_lights_pub) {
      m_hazard_lights_pub->publish(hazard_lights_report());
    } else if (InterfaceFeature::HEADLIGHTS == feat && m_headlights_pub) {
      m_headlights_pub->publish(headlights_report());
    } else if (InterfaceFeature::HORN == feat && m_horn_pub) {
      m_horn_pub->publish(horn_report());
    } else if (InterfaceFeature::ODOMETRY == feat && m_odometry_pub) {
      m_odometry_pub->publish(odometry());
    } else if (InterfaceFeature::TURN_INDICATORS == feat && m_turn_indicators_pub) {
      m_turn_indicators_pub->publish(turn_indicators_report());
    } else if (InterfaceFeature::WIPERS == feat && m_wipers_pub) {
      m_wipers_pub->publish(wipers_report());
    }
  }
}
}  // namespace interface
}  // namespace vehicle
}  // namespace autoware
