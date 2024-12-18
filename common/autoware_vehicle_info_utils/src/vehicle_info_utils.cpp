// Copyright 2015-2021 Autoware Foundation
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

#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <string>

namespace
{
template <class T>
T getParameter(rclcpp::Node & node, const std::string & name)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }

  try {
    return node.declare_parameter<T>(name);
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      node.get_logger(), "Failed to get parameter `%s`, please set it when you launch the node.",
      name.c_str());
    throw(ex);
  }
}
}  // namespace

namespace autoware::vehicle_info_utils
{
VehicleInfoUtils::VehicleInfoUtils(rclcpp::Node & node)
{
  const auto wheel_radius_m = getParameter<double>(node, "wheel_radius");
  const auto wheel_width_m = getParameter<double>(node, "wheel_width");
  const auto wheel_base_m = getParameter<double>(node, "wheel_base");
  const auto wheel_tread_m = getParameter<double>(node, "wheel_tread");
  const auto front_overhang_m = getParameter<double>(node, "front_overhang");
  const auto rear_overhang_m = getParameter<double>(node, "rear_overhang");
  const auto left_overhang_m = getParameter<double>(node, "left_overhang");
  const auto right_overhang_m = getParameter<double>(node, "right_overhang");
  const auto vehicle_height_m = getParameter<double>(node, "vehicle_height");
  const auto max_steer_angle_rad = getParameter<double>(node, "max_steer_angle");
  vehicle_info_ = createVehicleInfo(
    wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
    left_overhang_m, right_overhang_m, vehicle_height_m, max_steer_angle_rad);
}

VehicleInfo VehicleInfoUtils::getVehicleInfo() const
{
  return vehicle_info_;
}
}  // namespace autoware::vehicle_info_utils
