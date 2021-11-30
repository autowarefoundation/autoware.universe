// Copyright 2015-2019 Autoware Foundation
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

#include "vehicle_info_util/vehicle_info.hpp"

#include "rclcpp/rclcpp.hpp"

namespace vehicle_info_util
{
VehicleInfo::VehicleInfo(
  double wheel_radius_m, double wheel_width_m, double wheel_base_m, double wheel_tread_m,
  double front_overhang_m, double rear_overhang_m, double left_overhang_m, double right_overhang_m,
  double vehicle_height_m) noexcept
: wheel_radius_m_(wheel_radius_m),
  wheel_width_m_(wheel_width_m),
  wheel_base_m_(wheel_base_m),
  wheel_tread_m_(wheel_tread_m),
  front_overhang_m_(front_overhang_m),
  rear_overhang_m_(rear_overhang_m),
  left_overhang_m_(left_overhang_m),
  right_overhang_m_(right_overhang_m),
  vehicle_height_m_(vehicle_height_m),
  vehicle_length_m_(front_overhang_m + wheel_base_m + rear_overhang_m),
  vehicle_width_m_(wheel_tread_m + left_overhang_m + right_overhang_m),
  min_longitudinal_offset_m_(-rear_overhang_m),
  max_longitudinal_offset_m_(front_overhang_m + wheel_base_m),
  min_lateral_offset_m_(-(wheel_tread_m / 2.0 + right_overhang_m)),
  max_lateral_offset_m_(wheel_tread_m / 2.0 + left_overhang_m),
  min_height_offset_m_(0.0),
  max_height_offset_m_(vehicle_height_m)
{
}

bool VehicleInfo::parametersAlreadyDeclared(rclcpp::Node & node)
{
  return (
    node.has_parameter("wheel_radius") &&
    node.has_parameter("wheel_width") &&
    node.has_parameter("wheel_base") &&
    node.has_parameter("wheel_tread") &&
    node.has_parameter("front_overhang") &&
    node.has_parameter("rear_overhang") &&
    node.has_parameter("left_overhang") &&
    node.has_parameter("right_overhang") &&
    node.has_parameter("vehicle_height")
  );
}

VehicleInfo VehicleInfo::create(rclcpp::Node & node)
{
  if(!parametersAlreadyDeclared(node))
  {
    const double wheel_radius_m = node.declare_parameter("wheel_radius").get<double>();
    const double wheel_width_m = node.declare_parameter("wheel_width").get<double>();
    const double wheel_base_m = node.declare_parameter("wheel_base").get<double>();
    const double wheel_tread_m = node.declare_parameter("wheel_tread").get<double>();
    const double front_overhang_m = node.declare_parameter("front_overhang").get<double>();
    const double rear_overhang_m = node.declare_parameter("rear_overhang").get<double>();
    const double left_overhang_m = node.declare_parameter("left_overhang").get<double>();
    const double right_overhang_m = node.declare_parameter("right_overhang").get<double>();
    const double vehicle_height_m = node.declare_parameter("vehicle_height").get<double>();
    return VehicleInfo(
      wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
      left_overhang_m, right_overhang_m, vehicle_height_m);
  }
  else
  {
    const double wheel_radius_m = node.get_parameter("wheel_radius").as_double();
    const double wheel_width_m = node.get_parameter("wheel_width").as_double();
    const double wheel_base_m = node.get_parameter("wheel_base").as_double();
    const double wheel_tread_m = node.get_parameter("wheel_tread").as_double();
    const double front_overhang_m = node.get_parameter("front_overhang").as_double();
    const double rear_overhang_m = node.get_parameter("rear_overhang").as_double();
    const double left_overhang_m = node.get_parameter("left_overhang").as_double();
    const double right_overhang_m = node.get_parameter("right_overhang").as_double();
    const double vehicle_height_m = node.get_parameter("vehicle_height").as_double();
    return VehicleInfo(
      wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
      left_overhang_m, right_overhang_m, vehicle_height_m);
  }

}

}  // namespace vehicle_info_util
