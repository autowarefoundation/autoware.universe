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

#ifndef BASE_INTERFACE__BASE_INTERFACE_HPP_
#define BASE_INTERFACE__BASE_INTERFACE_HPP_

#include <common/types.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/headlights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/headlights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/horn_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/horn_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/wipers_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/wipers_report.hpp>
#include <autoware_auto_vehicle_msgs/srv/autonomy_mode_change.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <unordered_set>

using autoware::common::types::bool8_t;

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::HandBrakeCommand;
using autoware_auto_vehicle_msgs::msg::HandBrakeReport;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsReport;
using autoware_auto_vehicle_msgs::msg::HeadlightsCommand;
using autoware_auto_vehicle_msgs::msg::HeadlightsReport;
using autoware_auto_vehicle_msgs::msg::HornCommand;
using autoware_auto_vehicle_msgs::msg::HornReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using autoware_auto_vehicle_msgs::msg::WipersCommand;
using autoware_auto_vehicle_msgs::msg::WipersReport;
using VehicleOdometry = nav_msgs::msg::Odometry;

using ModeChangeRequest = autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Request;
using ModeChangeResponse = autoware_auto_vehicle_msgs::srv::AutonomyModeChange_Response;

namespace autoware
{
namespace vehicle
{
namespace interface
{
enum InterfaceFeature {
  GEAR = 0,
  HAND_BRAKE = 1,
  HAZARD_LIGHTS = 2,
  HEADLIGHTS = 3,
  HORN = 4,
  WIPERS = 5,
  TURN_INDICATORS = 6,
  ODOMETRY = 7
};

class BaseInterface
{
public:
  typedef std::unordered_set<InterfaceFeature> FeatureSet;

  BaseInterface() = default;
  virtual ~BaseInterface() = default;

  virtual bool8_t send_control_command(const AckermannControlCommand & msg) = 0;
  virtual bool8_t handle_mode_change_request(ModeChangeRequest::SharedPtr request) = 0;
  virtual void send_gear_command(const GearCommand & msg);
  virtual void send_hand_brake_command(const HandBrakeCommand & msg);
  virtual void send_hazard_lights_command(const HazardLightsCommand & msg);
  virtual void send_headlights_command(const HeadlightsCommand & msg);
  virtual void send_horn_command(const HornCommand & msg);
  virtual void send_wipers_command(const WipersCommand & msg);
  virtual void send_turn_indicators_command(const TurnIndicatorsCommand & msg);

  const GearReport & get_gear_report() const noexcept;
  const HandBrakeReport & get_hand_brake_report() const noexcept;
  const HazardLightsReport & get_hazard_lights_report() const noexcept;
  const HeadlightsReport & get_headlights_report() const noexcept;
  const HornReport & get_horn_report() const noexcept;
  const WipersReport & get_wipers_report() const noexcept;
  const TurnIndicatorsReport & get_turn_indicators_report() const noexcept;
  const VehicleOdometry & get_odometry() const noexcept;
  const FeatureSet & get_features() const noexcept;
  const SteeringReport & get_steering_report() noexcept;
  const VelocityReport & get_velocity_report() noexcept;

protected:
  // Used by a derived interface to provide reports
  GearReport & gear_report() noexcept;
  HandBrakeReport & hand_brake_report() noexcept;
  HazardLightsReport & hazard_lights_report() noexcept;
  HeadlightsReport & headlights_report() noexcept;
  HornReport & horn_report() noexcept;
  WipersReport & wipers_report() noexcept;
  TurnIndicatorsReport & turn_indicators_report() noexcept;
  VehicleOdometry & odometry() noexcept;
  FeatureSet & features() noexcept;
  SteeringReport & steering_report() noexcept;
  VelocityReport & velocity_report() noexcept;

private:
  // Underlying reports of vehicle state
  GearReport m_gear_report{};
  HandBrakeReport m_handbrake_report{};
  HazardLightsReport m_hazard_lights_report{};
  HeadlightsReport m_headlights_report{};
  HornReport m_horn_report{};
  WipersReport m_wipers_report{};
  TurnIndicatorsReport m_turn_indicators_report{};
  VehicleOdometry m_odometry{};
  FeatureSet m_features{};
  SteeringReport m_steering_report{};
  VelocityReport m_velocity_report{};
};
}  // namespace interface
}  // namespace vehicle
}  // namespace autoware

#endif  // BASE_INTERFACE__BASE_INTERFACE_HPP_
