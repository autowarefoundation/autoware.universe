// Copyright 2023 TIER IV, Inc.
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

#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "lanelet2_extension/projection/mgrs_projector.hpp"

#include <autoware_ad_api_specs/vehicle.hpp>
#include <component_interface_specs/vehicle.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/gear.hpp>
#include <autoware_adapi_v1_msgs/msg/hazard_lights.hpp>
#include <autoware_adapi_v1_msgs/msg/turn_indicators.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_projection/UTM.h>

#include <unordered_map>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{

class VehicleNode : public rclcpp::Node
{
public:
  explicit VehicleNode(const rclcpp::NodeOptions & options);

private:
  using GearReport = vehicle_interface::GearStatus::Message;
  using ApiGear = autoware_adapi_v1_msgs::msg::Gear;
  using TurnIndicatorsReport = vehicle_interface::TurnIndicatorStatus::Message;
  using ApiTurnIndicator = autoware_adapi_v1_msgs::msg::TurnIndicators;
  using HazardLightsReport = vehicle_interface::HazardLightStatus::Message;
  using ApiHazardLight = autoware_adapi_v1_msgs::msg::HazardLights;
  using MapProjectorInfo = vehicle_interface::MapProjectorInfo::Message;

  std::unordered_map<uint8_t, uint8_t> gear_type_ = {
    {GearReport::NONE, ApiGear::UNKNOWN},    {GearReport::NEUTRAL, ApiGear::NEUTRAL},
    {GearReport::DRIVE, ApiGear::DRIVE},     {GearReport::DRIVE_2, ApiGear::DRIVE},
    {GearReport::DRIVE_3, ApiGear::DRIVE},   {GearReport::DRIVE_4, ApiGear::DRIVE},
    {GearReport::DRIVE_5, ApiGear::DRIVE},   {GearReport::DRIVE_6, ApiGear::DRIVE},
    {GearReport::DRIVE_7, ApiGear::DRIVE},   {GearReport::DRIVE_8, ApiGear::DRIVE},
    {GearReport::DRIVE_9, ApiGear::DRIVE},   {GearReport::DRIVE_10, ApiGear::DRIVE},
    {GearReport::DRIVE_11, ApiGear::DRIVE},  {GearReport::DRIVE_12, ApiGear::DRIVE},
    {GearReport::DRIVE_13, ApiGear::DRIVE},  {GearReport::DRIVE_14, ApiGear::DRIVE},
    {GearReport::DRIVE_15, ApiGear::DRIVE},  {GearReport::DRIVE_16, ApiGear::DRIVE},
    {GearReport::DRIVE_17, ApiGear::DRIVE},  {GearReport::DRIVE_18, ApiGear::DRIVE},
    {GearReport::REVERSE, ApiGear::REVERSE}, {GearReport::REVERSE_2, ApiGear::REVERSE},
    {GearReport::PARK, ApiGear::PARK},       {GearReport::LOW, ApiGear::LOW},
    {GearReport::LOW_2, ApiGear::LOW},
  };

  std::unordered_map<uint8_t, uint8_t> turn_indicator_type_ = {
    {TurnIndicatorsReport::DISABLE, ApiTurnIndicator::DISABLE},
    {TurnIndicatorsReport::ENABLE_LEFT, ApiTurnIndicator::LEFT},
    {TurnIndicatorsReport::ENABLE_RIGHT, ApiTurnIndicator::RIGHT},
  };

  std::unordered_map<uint8_t, uint8_t> hazard_light_type_ = {
    {HazardLightsReport::DISABLE, ApiHazardLight::DISABLE},
    {HazardLightsReport::ENABLE, ApiHazardLight::ENABLE},
  };

  rclcpp::CallbackGroup::SharedPtr group_cli_;
  Pub<autoware_ad_api::vehicle::VehicleKinematics> pub_kinematics_;
  Pub<autoware_ad_api::vehicle::VehicleStatus> pub_status_;
  Sub<vehicle_interface::KinematicState> sub_kinematic_state_;
  Sub<vehicle_interface::Acceleration> sub_acceleration_;
  Sub<vehicle_interface::SteeringStatus> sub_steering_;
  Sub<vehicle_interface::GearStatus> sub_gear_state_;
  Sub<vehicle_interface::TurnIndicatorStatus> sub_turn_indicator_;
  Sub<vehicle_interface::HazardLightStatus> sub_hazard_light_;
  Sub<vehicle_interface::MapProjectorInfo> sub_map_projector_info_;
  Sub<vehicle_interface::EnergyStatus> sub_energy_level_;
  rclcpp::TimerBase::SharedPtr timer_;

  MapProjectorInfo::ConstSharedPtr map_projector_info_;
  autoware_ad_api::vehicle::VehicleKinematics::Message vehicle_kinematics_;
  autoware_ad_api::vehicle::VehicleStatus::Message vehicle_status_;

  void kinematic_state(const vehicle_interface::KinematicState::Message::ConstSharedPtr msg_ptr);
  void acceleration_status(const vehicle_interface::Acceleration::Message::ConstSharedPtr msg_ptr);
  void steering_status(const vehicle_interface::SteeringStatus::Message::ConstSharedPtr msg_ptr);
  void gear_status(const GearReport::ConstSharedPtr msg_ptr);
  void turn_indicator_status(const TurnIndicatorsReport::ConstSharedPtr msg_ptr);
  void map_projector_info(const MapProjectorInfo::ConstSharedPtr msg_ptr);
  void hazard_light_status(const HazardLightsReport::ConstSharedPtr msg_ptr);
  void energy_status(const vehicle_interface::EnergyStatus::Message::ConstSharedPtr msg_ptr);
  uint8_t mapping(
    std::unordered_map<uint8_t, uint8_t> hash_map, uint8_t input, uint8_t default_value);
  void on_timer();
  Eigen::Vector3d toBasicPoint3dPt(const geometry_msgs::msg::Point src);
};

}  // namespace default_ad_api

#endif  // VEHICLE_HPP_
