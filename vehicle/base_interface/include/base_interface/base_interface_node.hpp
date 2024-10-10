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

#ifndef BASE_INTERFACE__BASE_INTERFACE_NODE_HPP_
#define BASE_INTERFACE__BASE_INTERFACE_NODE_HPP_

#include <base_interface/base_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware
{
namespace vehicle
{
namespace interface
{
class BaseInterfaceNode : public rclcpp::Node, public BaseInterface
{
public:
  /**
   * @brief Creates a new Base Interface Node
   *
   * @param node_name Name of the node
   * @param features Set of InterfaceFeature to turn on
   * @param options NodeOptions
   */
  explicit BaseInterfaceNode(
    const std::string & node_name, const FeatureSet & features,
    const rclcpp::NodeOptions & options);

private:
  // Pubs and subs
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr m_command_sub{nullptr};
  rclcpp::Subscription<GearCommand>::SharedPtr m_gear_sub{nullptr};
  rclcpp::Subscription<HandBrakeCommand>::SharedPtr m_hand_brake_sub{nullptr};
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr m_hazard_lights_sub{nullptr};
  rclcpp::Subscription<HeadlightsCommand>::SharedPtr m_headlights_sub{nullptr};
  rclcpp::Subscription<HornCommand>::SharedPtr m_horn_sub{nullptr};
  rclcpp::Subscription<WipersCommand>::SharedPtr m_wipers_sub{nullptr};
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr m_turn_indicators_sub{nullptr};

  rclcpp::Publisher<GearReport>::SharedPtr m_gear_pub{nullptr};
  rclcpp::Publisher<HandBrakeReport>::SharedPtr m_hand_brake_pub{nullptr};
  rclcpp::Publisher<HazardLightsReport>::SharedPtr m_hazard_lights_pub{nullptr};
  rclcpp::Publisher<HeadlightsReport>::SharedPtr m_headlights_pub{nullptr};
  rclcpp::Publisher<HornReport>::SharedPtr m_horn_pub{nullptr};
  rclcpp::Publisher<WipersReport>::SharedPtr m_wipers_pub{nullptr};
  rclcpp::Publisher<TurnIndicatorsReport>::SharedPtr m_turn_indicators_pub{nullptr};
  rclcpp::Publisher<VehicleOdometry>::SharedPtr m_odometry_pub{nullptr};
  rclcpp::Publisher<SteeringReport>::SharedPtr m_steering_pub{nullptr};
  rclcpp::Publisher<VelocityReport>::SharedPtr m_velocity_pub{nullptr};

  // Mode-change service
  rclcpp::Service<autoware_auto_vehicle_msgs::srv::AutonomyModeChange>::SharedPtr m_mode_service{
    nullptr};

  // Report query timer
  rclcpp::TimerBase::SharedPtr m_report_timer{nullptr};

  // Callbacks
  void on_command(AckermannControlCommand::SharedPtr msg);
  void on_mode_change_request(
    ModeChangeRequest::SharedPtr request, ModeChangeResponse::SharedPtr response);
  void on_report_timer();
};
}  // namespace interface
}  // namespace vehicle
}  // namespace autoware

#endif  // BASE_INTERFACE__BASE_INTERFACE_NODE_HPP_
