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
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "lgsvl_msgs/msg/can_bus_data.hpp"
#include "lgsvl_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace lgsvl_interface
{
using autoware_auto_vehicle_msgs::msg::ControlModeReport;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using nav_msgs::msg::Odometry;

/// Node wrapping LgsvlInterface.
/// For a full list of behaviors, see \ref lgsvl
class LGSVL_INTERFACE_PUBLIC LgsvlInterfaceNode
  : public ::autoware::drivers::vehicle_interface::VehicleInterfaceNode
{
private:
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steer_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr pub_control_mode_report_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_report_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<lgsvl_msgs::msg::VehicleOdometry>::SharedPtr sub_vehicle_odom_;
  rclcpp::Subscription<lgsvl_msgs::msg::CanBusData>::SharedPtr sub_state_;


public:
  /// ROS 2 parameter constructor
  /// \param[in] options An rclcpp::NodeOptions object
  explicit LgsvlInterfaceNode(const rclcpp::NodeOptions & options);
};  // class LgsvlInterfaceNode
}  // namespace lgsvl_interface

#endif  // LGSVL_INTERFACE__LGSVL_INTERFACE_NODE_HPP_
