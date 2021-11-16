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
/// \file
/// \brief Implementation of vehicle interface for LGSVL simulator
#ifndef LGSVL_INTERFACE__LGSVL_INTERFACE_NODE_HPP_
#define LGSVL_INTERFACE__LGSVL_INTERFACE_NODE_HPP_

#include <lgsvl_interface/visibility_control.hpp>

#include <vehicle_interface/vehicle_interface_node.hpp>

#include <chrono>
#include <string>

#include "autoware_auto_vehicle_msgs/msg/headlights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/horn_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/wipers_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp"

namespace lgsvl_interface
{

/// Node wrapping LgsvlInterface.
/// For a full list of behaviors, see \ref lgsvl
class LGSVL_INTERFACE_PUBLIC LgsvlInterfaceNode
  : public ::autoware::drivers::vehicle_interface::VehicleInterfaceNode
{
public:
  /// ROS 2 parameter constructor
  /// \param[in] options An rclcpp::NodeOptions object
  explicit LgsvlInterfaceNode(const rclcpp::NodeOptions & options);
};  // class LgsvlInterfaceNode
}  // namespace lgsvl_interface

#endif  // LGSVL_INTERFACE__LGSVL_INTERFACE_NODE_HPP_
