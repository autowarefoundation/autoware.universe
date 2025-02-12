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

#include "vehicle.hpp"

#include <autoware/geography_utils/height.hpp>
#include <autoware/geography_utils/projection.hpp>

#include <geographic_msgs/msg/geo_point.hpp>

#include <limits>
#include <unordered_map>

namespace autoware::default_adapi
{

using GearReport = autoware::component_interface_specs_universe::vehicle::GearStatus::Message;
using ApiGear = autoware_adapi_v1_msgs::msg::Gear;
using TurnIndicatorsReport =
  autoware::component_interface_specs_universe::vehicle::TurnIndicatorStatus::Message;
using ApiTurnIndicator = autoware_adapi_v1_msgs::msg::TurnIndicators;
using HazardLightsReport =
  autoware::component_interface_specs_universe::vehicle::HazardLightStatus::Message;
using ApiHazardLight = autoware_adapi_v1_msgs::msg::HazardLights;
using MapProjectorInfo =
  autoware::component_interface_specs_universe::map::MapProjectorInfo::Message;

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

VehicleNode::VehicleNode(const rclcpp::NodeOptions & options) : Node("vehicle", options)
{
  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_pub(pub_kinematics_);
  adaptor.init_pub(pub_status_);
  adaptor.init_sub(sub_kinematic_state_, this, &VehicleNode::kinematic_state);
  adaptor.init_sub(sub_acceleration_, this, &VehicleNode::acceleration_status);
  adaptor.init_sub(sub_steering_, this, &VehicleNode::steering_status);
  adaptor.init_sub(sub_gear_state_, this, &VehicleNode::gear_status);
  adaptor.init_sub(sub_turn_indicator_, this, &VehicleNode::turn_indicator_status);
  adaptor.init_sub(sub_map_projector_info_, this, &VehicleNode::map_projector_info);
  adaptor.init_sub(sub_hazard_light_, this, &VehicleNode::hazard_light_status);
  adaptor.init_sub(sub_energy_level_, this, &VehicleNode::energy_status);

  const auto rate = rclcpp::Rate(10);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
}

uint8_t VehicleNode::mapping(
  std::unordered_map<uint8_t, uint8_t> hash_map, uint8_t input, uint8_t default_value)
{
  if (hash_map.find(input) == hash_map.end()) {
    return default_value;
  } else {
    return hash_map[input];
  }
}

void VehicleNode::kinematic_state(const autoware::component_interface_specs_universe::localization::
                                    KinematicState::Message::ConstSharedPtr msg_ptr)
{
  kinematic_state_msgs_ = msg_ptr;
}

void VehicleNode::acceleration_status(const autoware::component_interface_specs_universe::
                                        localization::Acceleration::Message::ConstSharedPtr msg_ptr)
{
  acceleration_msgs_ = msg_ptr;
}

void VehicleNode::steering_status(const autoware::component_interface_specs_universe::vehicle::
                                    SteeringStatus::Message::ConstSharedPtr msg_ptr)
{
  steering_status_msgs_ = msg_ptr;
}

void VehicleNode::gear_status(const GearReport::ConstSharedPtr msg_ptr)
{
  gear_status_msgs_ = msg_ptr;
}

void VehicleNode::turn_indicator_status(const TurnIndicatorsReport::ConstSharedPtr msg_ptr)
{
  turn_indicator_status_msgs_ = msg_ptr;
}

void VehicleNode::hazard_light_status(const HazardLightsReport::ConstSharedPtr msg_ptr)
{
  hazard_light_status_msgs_ = msg_ptr;
}

void VehicleNode::energy_status(
  const autoware::component_interface_specs_universe::vehicle::EnergyStatus::Message::ConstSharedPtr
    msg_ptr)
{
  energy_status_msgs_ = msg_ptr;
}

void VehicleNode::map_projector_info(const MapProjectorInfo::ConstSharedPtr msg_ptr)
{
  map_projector_info_ = msg_ptr;
}

void VehicleNode::publish_kinematics()
{
  if (!kinematic_state_msgs_ || !acceleration_msgs_ || !map_projector_info_) return;

  autoware::adapi_specs::vehicle::VehicleKinematics::Message vehicle_kinematics;
  vehicle_kinematics.pose.header = kinematic_state_msgs_->header;
  vehicle_kinematics.pose.pose = kinematic_state_msgs_->pose;
  vehicle_kinematics.twist.header = kinematic_state_msgs_->header;
  vehicle_kinematics.twist.header.frame_id = kinematic_state_msgs_->child_frame_id;
  vehicle_kinematics.twist.twist = kinematic_state_msgs_->twist;
  if (map_projector_info_->projector_type != MapProjectorInfo::LOCAL) {
    const geographic_msgs::msg::GeoPoint projected_gps_point =
      autoware::geography_utils::project_reverse(
        kinematic_state_msgs_->pose.pose.position, *map_projector_info_);
    vehicle_kinematics.geographic_pose.header = kinematic_state_msgs_->header;
    vehicle_kinematics.geographic_pose.header.frame_id = "global";
    vehicle_kinematics.geographic_pose.position.latitude = projected_gps_point.latitude;
    vehicle_kinematics.geographic_pose.position.longitude = projected_gps_point.longitude;
    vehicle_kinematics.geographic_pose.position.altitude =
      autoware::geography_utils::convert_height(
        projected_gps_point.altitude, projected_gps_point.latitude, projected_gps_point.longitude,
        map_projector_info_->vertical_datum, MapProjectorInfo::WGS84);
  } else {
    vehicle_kinematics.geographic_pose.position.latitude = std::numeric_limits<double>::quiet_NaN();
    vehicle_kinematics.geographic_pose.position.longitude =
      std::numeric_limits<double>::quiet_NaN();
    vehicle_kinematics.geographic_pose.position.altitude = std::numeric_limits<double>::quiet_NaN();
  }
  vehicle_kinematics.accel.header = acceleration_msgs_->header;
  vehicle_kinematics.accel.accel = acceleration_msgs_->accel;
  pub_kinematics_->publish(vehicle_kinematics);
}

void VehicleNode::publish_status()
{
  if (
    !steering_status_msgs_ || !gear_status_msgs_ || !turn_indicator_status_msgs_ ||
    !hazard_light_status_msgs_)
    return;

  autoware::adapi_specs::vehicle::VehicleStatus::Message vehicle_status;
  vehicle_status.stamp = now();
  vehicle_status.steering_tire_angle = steering_status_msgs_->steering_tire_angle;
  vehicle_status.gear.status = mapping(gear_type_, gear_status_msgs_->report, ApiGear::UNKNOWN);
  vehicle_status.turn_indicators.status =
    mapping(turn_indicator_type_, turn_indicator_status_msgs_->report, ApiTurnIndicator::UNKNOWN);
  vehicle_status.hazard_lights.status =
    mapping(hazard_light_type_, hazard_light_status_msgs_->report, ApiHazardLight::UNKNOWN);
  pub_status_->publish(vehicle_status);
}

void VehicleNode::on_timer()
{
  publish_kinematics();
  publish_status();
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::VehicleNode)
