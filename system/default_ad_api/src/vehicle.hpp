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

#include "tier4_autoware_utils/ros/polling_subscriber.hpp"

#include <autoware_ad_api_specs/vehicle.hpp>
#include <component_interface_specs/localization.hpp>
#include <component_interface_specs/map.hpp>
#include <component_interface_specs/vehicle.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/gear.hpp>
#include <autoware_adapi_v1_msgs/msg/hazard_lights.hpp>
#include <autoware_adapi_v1_msgs/msg/turn_indicators.hpp>

#include <unordered_map>

// This file should be included after messages.
#include "utils/interface_subscriber.hpp"
#include "utils/types.hpp"

namespace default_ad_api
{

class VehicleNode : public rclcpp::Node
{
public:
  explicit VehicleNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  Pub<autoware_ad_api::vehicle::VehicleKinematics> pub_kinematics_;
  Pub<autoware_ad_api::vehicle::VehicleStatus> pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tier4_autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>>
    kinematic_state_sub_;
  std::shared_ptr<tier4_autoware_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::AccelWithCovarianceStamped>>
    acceleration_sub_;
  std::shared_ptr<tier4_autoware_utils::InterProcessPollingSubscriber<
    autoware_auto_vehicle_msgs::msg::SteeringReport>>
    steering_sub_;
  std::shared_ptr<tier4_autoware_utils::InterProcessPollingSubscriber<
    autoware_auto_vehicle_msgs::msg::GearReport>>
    gear_state_sub_;
  std::shared_ptr<tier4_autoware_utils::InterProcessPollingSubscriber<
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>>
    turn_indicator_info_sub_;
  std::shared_ptr<
    tier4_autoware_utils::InterProcessPollingSubscriber<tier4_map_msgs::msg::MapProjectorInfo>>
    map_projector_info_sub_;
  std::shared_ptr<tier4_autoware_utils::InterProcessPollingSubscriber<
    autoware_auto_vehicle_msgs::msg::HazardLightsReport>>
    hazard_light_sub_;
  std::shared_ptr<
    tier4_autoware_utils::InterProcessPollingSubscriber<tier4_vehicle_msgs::msg::BatteryStatus>>
    energy_level_sub_;

  localization_interface::KinematicState::Message::ConstSharedPtr kinematic_state_msgs_;
  localization_interface::Acceleration::Message::ConstSharedPtr acceleration_msgs_;
  vehicle_interface::SteeringStatus::Message::ConstSharedPtr steering_status_msgs_;
  vehicle_interface::GearStatus::Message::ConstSharedPtr gear_status_msgs_;
  vehicle_interface::TurnIndicatorStatus::Message::ConstSharedPtr turn_indicator_status_msgs_;
  vehicle_interface::HazardLightStatus::Message::ConstSharedPtr hazard_light_status_msgs_;
  vehicle_interface::EnergyStatus::Message::ConstSharedPtr energy_status_msgs_;
  map_interface::MapProjectorInfo::Message::ConstSharedPtr map_projector_info_;

  void kinematic_state(
    const localization_interface::KinematicState::Message::ConstSharedPtr msg_ptr);
  void acceleration_status(
    const localization_interface::Acceleration::Message::ConstSharedPtr msg_ptr);
  void steering_status(const vehicle_interface::SteeringStatus::Message::ConstSharedPtr msg_ptr);
  void gear_status(const vehicle_interface::GearStatus::Message::ConstSharedPtr msg_ptr);
  void turn_indicator_status(
    const vehicle_interface::TurnIndicatorStatus::Message::ConstSharedPtr msg_ptr);
  void map_projector_info(const map_interface::MapProjectorInfo::Message::ConstSharedPtr msg_ptr);
  void hazard_light_status(
    const vehicle_interface::HazardLightStatus::Message::ConstSharedPtr msg_ptr);
  void energy_status(const vehicle_interface::EnergyStatus::Message::ConstSharedPtr msg_ptr);
  uint8_t mapping(
    std::unordered_map<uint8_t, uint8_t> hash_map, uint8_t input, uint8_t default_value);
  void publish_kinematics();
  void publish_status();
  void on_timer();
};

}  // namespace default_ad_api

#endif  // VEHICLE_HPP_
