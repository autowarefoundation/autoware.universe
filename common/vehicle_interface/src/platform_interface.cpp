// Copyright 2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#include "vehicle_interface/platform_interface.hpp"

#include "autoware_auto_vehicle_msgs/msg/headlights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/horn_command.hpp"

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{
const autoware_auto_vehicle_msgs::msg::VehicleStateReport &
PlatformInterface::get_state_report() const noexcept
{
  return m_state_report;
}

const autoware_auto_vehicle_msgs::msg::VehicleOdometry & PlatformInterface::get_odometry() const
noexcept
{
  return m_odometry;
}

autoware_auto_vehicle_msgs::msg::VehicleStateReport & PlatformInterface::state_report() noexcept
{
  return m_state_report;
}

const autoware_auto_vehicle_msgs::msg::HeadlightsReport &
PlatformInterface::get_headlights_report() const noexcept
{
  return m_headlights_report;
}

const autoware_auto_vehicle_msgs::msg::HornReport &
PlatformInterface::get_horn_report() const noexcept
{
  return m_horn_report;
}

const autoware_auto_vehicle_msgs::msg::WipersReport &
PlatformInterface::get_wipers_report() const noexcept
{
  return m_wipers_report;
}

autoware_auto_vehicle_msgs::msg::VehicleOdometry & PlatformInterface::odometry() noexcept
{
  return m_odometry;
}

autoware_auto_vehicle_msgs::msg::HeadlightsReport & PlatformInterface::headlights_report() noexcept
{
  return m_headlights_report;
}

autoware_auto_vehicle_msgs::msg::HornReport & PlatformInterface::horn_report() noexcept
{
  return m_horn_report;
}

autoware_auto_vehicle_msgs::msg::WipersReport & PlatformInterface::wipers_report() noexcept
{
  return m_wipers_report;
}

void PlatformInterface::send_headlights_command(
  const autoware_auto_vehicle_msgs::msg::HeadlightsCommand & msg)
{
  (void)msg;
  throw std::runtime_error("HeadlightsCommand not supported by this vehicle interface");
}

void PlatformInterface::send_horn_command(
  const autoware_auto_vehicle_msgs::msg::HornCommand & msg)
{
  (void)msg;
  throw std::runtime_error("HornCommand not supported by this vehicle interface");
}

void PlatformInterface::send_wipers_command(
  const autoware_auto_vehicle_msgs::msg::WipersCommand & msg)
{
  (void)msg;
  throw std::runtime_error("WipersCommand not supported by this vehicle interface");
}

void PlatformInterface::send_hazard_lights_command(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand & msg)
{
  (void)msg;
  throw std::runtime_error("HazardLightsCommand not supported by this vehicle interface");
}

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware
