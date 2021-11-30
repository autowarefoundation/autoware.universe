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

#ifndef VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_
#define VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_

#include "rclcpp/rclcpp.hpp"

namespace vehicle_info_util
{
/// This is a convenience class for saving you from declaring all parameters
/// manually and calculating derived parameters. It also
///
/// It's a bit odd to have a class with public data members. However, I think
/// this is a clean solution that
/// * protects the relationship between base and derived parameters
/// * doesn't have boilerplate such as a getter for each field
/// * respects the Google C++ Style Guide
class VehicleInfo
{
public:
  /// Constructor
  ///
  /// Normally, the factory function will be preferred.
  VehicleInfo(
    double wheel_radius_m, double wheel_width_m, double wheel_base_m, double wheel_tread_m,
    double front_overhang_m, double rear_overhang_m, double left_overhang_m,
    double right_overhang_m, double vehicle_height_m) noexcept;

  /// Factory function
  ///
  /// This function will declare a parameter for each of the base parameters.
  /// This may throw an exception if not all base parameters were specified.
  static VehicleInfo create(rclcpp::Node & node);

  // Base parameters. These describe the vehicle's bounding box and the
  // position and radius of the wheels.
  const double wheel_radius_m_;
  const double wheel_width_m_;
  const double wheel_base_m_;
  const double wheel_tread_m_;
  const double front_overhang_m_;
  const double rear_overhang_m_;
  const double left_overhang_m_;
  const double right_overhang_m_;
  const double vehicle_height_m_;

  // Derived parameters, i.e. calculated from base parameters
  // The offset values are relative to the base frame origin, which is located
  // on the ground below the middle of the rear axle, and can be negative.
  const double vehicle_length_m_;
  const double vehicle_width_m_;
  const double min_longitudinal_offset_m_;
  const double max_longitudinal_offset_m_;
  const double min_lateral_offset_m_;
  const double max_lateral_offset_m_;
  const double min_height_offset_m_;
  const double max_height_offset_m_;

private:
  // Used to check if parameters are already declared or not in order to avoid
  // declaring parameters multiple times.
  static bool parametersAlreadyDeclared(rclcpp::Node & node);
  static void declearVehicleParameters(rclcpp::Node & node);
  static VehicleInfo getVehicleInfo(rclcpp::Node & node);
  static void waitVehicleInfo(rclcpp::Node & node);
};

}  // namespace vehicle_info_util

#endif  // VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_
