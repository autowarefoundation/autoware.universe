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

#include "vehicle_info_util/vehicle_info_util.hpp"

#include <string>

namespace
{
template<class T>
T getParameter(rclcpp::Node & node, const std::string & name)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }

  try {
    return node.declare_parameter(name).get<T>();
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      node.get_logger(), "Failed to get parameter `%s`, please set it when you launch the node.",
      name.c_str());
  }
}
}  // namespace

namespace vehicle_info_util
{
VehicleInfoUtil::VehicleInfoUtil(rclcpp::Node & node)
{
  vehicle_info_.wheel_radius_m = getParameter<double>(node, "wheel_radius");
  vehicle_info_.wheel_width_m = getParameter<double>(node, "wheel_width");
  vehicle_info_.wheel_base_m = getParameter<double>(node, "wheel_base");
  vehicle_info_.wheel_tread_m = getParameter<double>(node, "wheel_tread");
  vehicle_info_.front_overhang_m = getParameter<double>(node, "front_overhang");
  vehicle_info_.rear_overhang_m = getParameter<double>(node, "rear_overhang");
  vehicle_info_.left_overhang_m = getParameter<double>(node, "left_overhang");
  vehicle_info_.right_overhang_m = getParameter<double>(node, "right_overhang");
  vehicle_info_.vehicle_height_m = getParameter<double>(node, "vehicle_height");
}

VehicleInfo VehicleInfoUtil::getVehicleInfo()
{
  // Base parameters
  const double wheel_radius_m = vehicle_info_.wheel_radius_m;
  const double wheel_width_m = vehicle_info_.wheel_width_m;
  const double wheel_base_m = vehicle_info_.wheel_base_m;
  const double wheel_tread_m = vehicle_info_.wheel_tread_m;
  const double front_overhang_m = vehicle_info_.front_overhang_m;
  const double rear_overhang_m = vehicle_info_.rear_overhang_m;
  const double left_overhang_m = vehicle_info_.left_overhang_m;
  const double right_overhang_m = vehicle_info_.right_overhang_m;
  const double vehicle_height_m = vehicle_info_.vehicle_height_m;

  // Derived parameters
  const double vehicle_length_m_ = front_overhang_m + wheel_base_m + rear_overhang_m;
  const double vehicle_width_m_ = wheel_tread_m + left_overhang_m + right_overhang_m;
  const double min_longitudinal_offset_m_ = -rear_overhang_m;
  const double max_longitudinal_offset_m_ = front_overhang_m + wheel_base_m;
  const double min_lateral_offset_m_ = -(wheel_tread_m / 2.0 + right_overhang_m);
  const double max_lateral_offset_m_ = wheel_tread_m / 2.0 + left_overhang_m;
  const double min_height_offset_m_ = 0.0;
  const double max_height_offset_m_ = vehicle_height_m;

  return VehicleInfo{
    // Base parameters
    wheel_radius_m,
    wheel_width_m,
    wheel_base_m,
    wheel_tread_m,
    front_overhang_m,
    rear_overhang_m,
    left_overhang_m,
    right_overhang_m,
    vehicle_height_m,
    // Derived parameters
    vehicle_length_m_,
    vehicle_width_m_,
    min_longitudinal_offset_m_,
    max_longitudinal_offset_m_,
    min_lateral_offset_m_,
    max_lateral_offset_m_,
    min_height_offset_m_,
    max_height_offset_m_,
  };
}

}  // namespace vehicle_info_util
