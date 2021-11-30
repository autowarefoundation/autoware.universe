// Copyright 2017-2019 Autoware Foundation
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

#ifndef VEHICLE_RAW_VEHICLE_CMD_CONVERTER_INCLUDE_RAW_VEHICLE_CMD_CONVERTER_NODE_HPP_
#define VEHICLE_RAW_VEHICLE_CMD_CONVERTER_INCLUDE_RAW_VEHICLE_CMD_CONVERTER_NODE_HPP_

#include "raw_vehicle_cmd_converter/accel_map.hpp"
#include "raw_vehicle_cmd_converter/brake_map.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "autoware_vehicle_msgs/msg/raw_vehicle_command.hpp"
#include "autoware_vehicle_msgs/msg/shift.hpp"
#include "autoware_vehicle_msgs/msg/vehicle_command.hpp"

#include <memory>
#include <string>

class AccelMapConverter : public rclcpp::Node
{
public:
  AccelMapConverter();
  ~AccelMapConverter() = default;

private:
  rclcpp::Publisher<autoware_vehicle_msgs::msg::RawVehicleCommand>::SharedPtr pub_cmd_;        //!< @brief topic publisher for low-level vehicle command
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;  //!< @brief subscriber for current velocity
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr sub_cmd_;       //!< @brief subscriber for vehicle command

  std::shared_ptr<double> current_velocity_ptr_;  // [m/s]

  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool acc_map_initialized_;  //!< @brief flag to manage validity of imported accel map files
  double max_throttle_;  //!< @brief maximum throttle that can be passed to low level controller. In general [0.0, 1.0]
  double max_brake_;  //!< @brief maximum brake value that can be passed to low level controller. In general [0.0, 1.0]

  void callbackVehicleCmd(
    const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr vehicle_cmd_ptr);
  void callbackVelocity(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void calculateAccelMap(
    const double current_velocity, const double desired_acc, double * desired_throttle,
    double * desired_brake);
};

#endif  // VEHICLE_RAW_VEHICLE_CMD_CONVERTER_INCLUDE_RAW_VEHICLE_CMD_CONVERTER_NODE_HPP_
