// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//// Copyright 2022 TIER IV, Inc.
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

namespace default_ad_api
{

VehicleNode::VehicleNode(const rclcpp::NodeOptions & options) : Node("vehicle", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_pub(pub_position_);
  adaptor.init_pub(pub_motion_);
  adaptor.init_pub(pub_state_);
  adaptor.init_pub(pub_door_);
  adaptor.init_sub(sub_kinematic_state_, this, &VehicleNode::kinematic_state);
  adaptor.init_sub(sub_acceleration_, this, &VehicleNode::acceleration_status);
  adaptor.init_sub(sub_steering_, this, &VehicleNode::steering_status);
  adaptor.init_sub(sub_gear_state_, this, &VehicleNode::gear_status);
  adaptor.init_sub(sub_turn_indicator_, this, &VehicleNode::turn_indicator_status);
  adaptor.init_sub(sub_hazard_light_, this, &VehicleNode::hazard_light_status);
  adaptor.init_sub(sub_energy_level_, this, &VehicleNode::energy_status);
  // adaptor.init_sub(sub_door_status_, this, &VehicleNode::door_status);

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

void VehicleNode::kinematic_state(
  const vehicle_interface::KinematicState::Message::ConstSharedPtr msg_ptr)
{
  vehicle_position_ptr.header = msg_ptr->header;
  vehicle_position_ptr.child_frame_id = msg_ptr->child_frame_id;
  vehicle_position_ptr.pose = msg_ptr->pose.pose;

  vehicle_motion_ptr.header = msg_ptr->header;
  vehicle_motion_ptr.twist = msg_ptr->twist;
}

void VehicleNode::acceleration_status(
  const vehicle_interface::Acceleration::Message::ConstSharedPtr msg_ptr)
{
  vehicle_motion_ptr.accel = msg_ptr->accel;
}

void VehicleNode::steering_status(
  const vehicle_interface::SteeringStatus::Message::ConstSharedPtr msg_ptr)
{
  vehicle_motion_ptr.steering_tire_angle = msg_ptr->steering_tire_angle;
}

void VehicleNode::gear_status(const GearReport::ConstSharedPtr msg_ptr)
{
  vehicle_state_ptr.gear.status = mapping(gear_type_, msg_ptr->report, VehicleGear::NONE);
}

void VehicleNode::turn_indicator_status(const TurnIndicatorsReport::ConstSharedPtr msg_ptr)
{
  vehicle_state_ptr.turn_indicator.status =
    mapping(turn_indicator_type_, msg_ptr->report, VehicleTurnIndicator::DISABLE);
}

void VehicleNode::hazard_light_status(const HazardLightsReport::ConstSharedPtr msg_ptr)
{
  vehicle_state_ptr.hazard_light.status =
    mapping(hazard_light_type_, msg_ptr->report, VehicleHazardLight::DISABLE);
}

void VehicleNode::energy_status(
  const vehicle_interface::EnergyStatus::Message::ConstSharedPtr msg_ptr)
{
  vehicle_state_ptr.energy_level = msg_ptr->energy_level;
}

void VehicleNode::on_timer()
{
  vehicle_state_ptr.stamp = now();
  pub_position_->publish(vehicle_position_ptr);
  pub_motion_->publish(vehicle_motion_ptr);
  pub_state_->publish(vehicle_state_ptr);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::VehicleNode)
