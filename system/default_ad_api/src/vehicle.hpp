// Copyright 2022 TIER IV, Inc.
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

#include <autoware_adapi_v1_msgs/msg/door_status.hpp>
#include <autoware_adapi_v1_msgs/msg/gear.hpp>
#include <autoware_adapi_v1_msgs/msg/hazard_light.hpp>
#include <autoware_adapi_v1_msgs/msg/turn_indicator.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <string>
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
  using VehicleGear = autoware_adapi_v1_msgs::msg::Gear;
  using TurnIndicatorsReport = vehicle_interface::TurnIndicatorStatus::Message;
  using VehicleTurnIndicator = autoware_adapi_v1_msgs::msg::TurnIndicator;
  using HazardLightsReport = vehicle_interface::HazardLightStatus::Message;
  using VehicleHazardLight = autoware_adapi_v1_msgs::msg::HazardLight;
  using VehicleDoorStatus = vehicle_interface::DoorStatus::Message;
  using ApiDoorStatus = autoware_adapi_v1_msgs::msg::DoorStatus;

  std::unordered_map<uint8_t, uint8_t> gear_type_ = {
    {GearReport::NONE, VehicleGear::UNKNOWN},    {GearReport::NEUTRAL, VehicleGear::NEUTRAL},
    {GearReport::DRIVE, VehicleGear::DRIVE},     {GearReport::DRIVE_2, VehicleGear::DRIVE},
    {GearReport::DRIVE_3, VehicleGear::DRIVE},   {GearReport::DRIVE_4, VehicleGear::DRIVE},
    {GearReport::DRIVE_5, VehicleGear::DRIVE},   {GearReport::DRIVE_6, VehicleGear::DRIVE},
    {GearReport::DRIVE_7, VehicleGear::DRIVE},   {GearReport::DRIVE_8, VehicleGear::DRIVE},
    {GearReport::DRIVE_9, VehicleGear::DRIVE},   {GearReport::DRIVE_10, VehicleGear::DRIVE},
    {GearReport::DRIVE_11, VehicleGear::DRIVE},  {GearReport::DRIVE_12, VehicleGear::DRIVE},
    {GearReport::DRIVE_13, VehicleGear::DRIVE},  {GearReport::DRIVE_14, VehicleGear::DRIVE},
    {GearReport::DRIVE_15, VehicleGear::DRIVE},  {GearReport::DRIVE_16, VehicleGear::DRIVE},
    {GearReport::DRIVE_17, VehicleGear::DRIVE},  {GearReport::DRIVE_18, VehicleGear::DRIVE},
    {GearReport::REVERSE, VehicleGear::REVERSE}, {GearReport::REVERSE_2, VehicleGear::REVERSE},
    {GearReport::PARK, VehicleGear::PARK},       {GearReport::LOW, VehicleGear::LOW},
    {GearReport::LOW_2, VehicleGear::LOW},
  };

  std::unordered_map<uint8_t, uint8_t> turn_indicator_type_ = {
    {TurnIndicatorsReport::DISABLE, VehicleTurnIndicator::DISABLE},
    {TurnIndicatorsReport::ENABLE_LEFT, VehicleTurnIndicator::LEFT},
    {TurnIndicatorsReport::ENABLE_RIGHT, VehicleTurnIndicator::RIGHT},
  };

  std::unordered_map<uint8_t, uint8_t> hazard_light_type_ = {
    {HazardLightsReport::DISABLE, VehicleHazardLight::DISABLE},
    {HazardLightsReport::ENABLE, VehicleHazardLight::ENABLE},
  };

  std::unordered_map<uint8_t, uint8_t> door_status_type_ = {
    {VehicleDoorStatus::UNKNOWN, ApiDoorStatus::UNKNOWN},
    {VehicleDoorStatus::DOOR_OPENED, ApiDoorStatus::OPENED},
    {VehicleDoorStatus::DOOR_CLOSED, ApiDoorStatus::CLOSED},
    {VehicleDoorStatus::DOOR_OPENING, ApiDoorStatus::OPENING},
    {VehicleDoorStatus::DOOR_CLOSING, ApiDoorStatus::CLOSING},
    {VehicleDoorStatus::NOT_APPLICABLE, ApiDoorStatus::NOT_AVAILABLE},
  };

  rclcpp::CallbackGroup::SharedPtr group_cli_;
  Pub<autoware_ad_api::vehicle::VehicleKinematic> pub_kinematic_;
  Pub<autoware_ad_api::vehicle::VehicleState> pub_state_;
  Pub<autoware_ad_api::vehicle::DoorStatusArray> pub_door_;
  Sub<vehicle_interface::KinematicState> sub_kinematic_state_;
  Sub<vehicle_interface::Acceleration> sub_acceleration_;
  Sub<vehicle_interface::SteeringStatus> sub_steering_;
  Sub<vehicle_interface::GearStatus> sub_gear_state_;
  Sub<vehicle_interface::TurnIndicatorStatus> sub_turn_indicator_;
  Sub<vehicle_interface::HazardLightStatus> sub_hazard_light_;
  Sub<vehicle_interface::MGRSGrid> sub_mgrs_grid_;
  Sub<vehicle_interface::EnergyStatus> sub_energy_level_;
  Sub<vehicle_interface::DoorStatus> sub_door_status_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string mgrs_grid_ptr;
  autoware_ad_api::vehicle::VehicleKinematic::Message vehicle_kinematic_ptr;
  autoware_ad_api::vehicle::VehicleState::Message vehicle_state_ptr;
  autoware_ad_api::vehicle::DoorStatusArray::Message vehicle_door_ptr;

  void kinematic_state(const vehicle_interface::KinematicState::Message::ConstSharedPtr msg_ptr);
  void acceleration_status(const vehicle_interface::Acceleration::Message::ConstSharedPtr msg_ptr);
  void steering_status(const vehicle_interface::SteeringStatus::Message::ConstSharedPtr msg_ptr);
  void gear_status(const GearReport::ConstSharedPtr msg_ptr);
  void turn_indicator_status(const TurnIndicatorsReport::ConstSharedPtr msg_ptr);
  void mgrs_grid_data(const vehicle_interface::MGRSGrid::Message::ConstSharedPtr msg_ptr);
  void hazard_light_status(const HazardLightsReport::ConstSharedPtr msg_ptr);
  void energy_status(const vehicle_interface::EnergyStatus::Message::ConstSharedPtr msg_ptr);
  void door_status(const VehicleDoorStatus::ConstSharedPtr msg_ptr);
  uint8_t mapping(
    std::unordered_map<uint8_t, uint8_t> hash_map, uint8_t input, uint8_t default_value);
  void on_timer();
  Eigen::Vector3d toBasicPoint3dPt(const geometry_msgs::msg::Point src);
};

}  // namespace default_ad_api

#endif  // VEHICLE_HPP_
