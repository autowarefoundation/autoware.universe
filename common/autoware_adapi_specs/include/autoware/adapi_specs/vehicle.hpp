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

#ifndef AUTOWARE__ADAPI_SPECS__VEHICLE_HPP_
#define AUTOWARE__ADAPI_SPECS__VEHICLE_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/msg/door_status_array.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_kinematics.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_status.hpp>
#include <autoware_adapi_v1_msgs/srv/get_door_layout.hpp>
#include <autoware_adapi_v1_msgs/srv/get_vehicle_dimensions.hpp>
#include <autoware_adapi_v1_msgs/srv/set_door_command.hpp>

namespace autoware::adapi_specs::vehicle
{

struct VehicleKinematics
{
  using Message = autoware_adapi_v1_msgs::msg::VehicleKinematics;
  static constexpr char name[] = "/api/vehicle/kinematics";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct VehicleStatus
{
  using Message = autoware_adapi_v1_msgs::msg::VehicleStatus;
  static constexpr char name[] = "/api/vehicle/status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct Dimensions
{
  using Service = autoware_adapi_v1_msgs::srv::GetVehicleDimensions;
  static constexpr char name[] = "/api/vehicle/dimensions";
};

struct DoorCommand
{
  using Service = autoware_adapi_v1_msgs::srv::SetDoorCommand;
  static constexpr char name[] = "/api/vehicle/doors/command";
};

struct DoorLayout
{
  using Service = autoware_adapi_v1_msgs::srv::GetDoorLayout;
  static constexpr char name[] = "/api/vehicle/doors/layout";
};

struct DoorStatus
{
  using Message = autoware_adapi_v1_msgs::msg::DoorStatusArray;
  static constexpr char name[] = "/api/vehicle/doors/status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace autoware::adapi_specs::vehicle

#endif  // AUTOWARE__ADAPI_SPECS__VEHICLE_HPP_
