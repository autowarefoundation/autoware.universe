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

#include "vehicle_interface/base_interface.hpp"

namespace autoware
{
namespace vehicle
{
namespace interface
{
void BaseInterface::send_gear_command(
  const autoware_auto_vehicle_msgs::msg::GearCommand & msg)
{
  (void)msg;
  throw std::runtime_error("GearCommand not supported by this vehicle interface");
}

void BaseInterface::send_hand_brake_command(
  const autoware_auto_vehicle_msgs::msg::HandBrakeCommand & msg)
{
  (void)msg;
  throw std::runtime_error("HandBrakeCommand not supported by this vehicle interface");
}

void BaseInterface::send_hazard_lights_command(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand & msg)
{
  (void)msg;
  throw std::runtime_error("HazardLightsCommand not supported by this vehicle interface");
}

void BaseInterface::send_headlights_command(
  const autoware_auto_vehicle_msgs::msg::HeadlightsCommand & msg)
{
  (void)msg;
  throw std::runtime_error("HeadlightsCommand not supported by this vehicle interface");
}

void BaseInterface::send_horn_command(
  const autoware_auto_vehicle_msgs::msg::HornCommand & msg)
{
  (void)msg;
  throw std::runtime_error("HornCommand not supported by this vehicle interface");
}

void BaseInterface::send_wipers_command(
  const autoware_auto_vehicle_msgs::msg::WipersCommand & msg)
{
  (void)msg;
  throw std::runtime_error("WipersCommand not supported by this vehicle interface");
}

void BaseInterface::send_turn_indicators_command(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand & msg)
{
  (void)msg;
  throw std::runtime_error("TurnIndicatorsCommand not supported by this vehicle interface");
}

const GearReport & BaseInterface::get_gear_report() const noexcept
{
  return m_gear_report;
}

const HandBrakeReport & BaseInterface::get_hand_brake_report() const noexcept
{
  return m_handbrake_report;
}

const HazardLightsReport & BaseInterface::get_hazard_lights_report() const noexcept
{
  return m_hazard_lights_report;
}

const HeadlightsReport & BaseInterface::get_headlights_report() const noexcept
{
  return m_headlights_report;
}

const HornReport & BaseInterface::get_horn_report() const noexcept
{
  return m_horn_report;
}

const WipersReport & BaseInterface::get_wipers_report() const noexcept
{
  return m_wipers_report;
}

const TurnIndicatorsReport & BaseInterface::get_turn_indicators_report() const noexcept
{
  return m_turn_indicators_report;
}

const VehicleOdometry & BaseInterface::get_odometry() const noexcept
{
  return m_odometry;
}

GearReport & BaseInterface::gear_report() noexcept
{
  return m_gear_report;
}

HandBrakeReport & BaseInterface::hand_brake_report() noexcept
{
  return m_handbrake_report;
}

HazardLightsReport & BaseInterface::hazard_lights_report() noexcept
{
  return m_hazard_lights_report;
}

HeadlightsReport & BaseInterface::headlights_report() noexcept
{
  return m_headlights_report;
}

HornReport & BaseInterface::horn_report() noexcept
{
  return m_horn_report;
}

WipersReport & BaseInterface::wipers_report() noexcept
{
  return m_wipers_report;
}

TurnIndicatorsReport & BaseInterface::turn_indicators_report() noexcept
{
  return m_turn_indicators_report;
}

VehicleOdometry & BaseInterface::odometry() noexcept
{
  return m_odometry;
}

}  // namespace interface
}  // namespace vehicle
}  // namespace autoware
