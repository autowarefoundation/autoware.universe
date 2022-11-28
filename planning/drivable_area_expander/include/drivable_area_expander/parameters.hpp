// Copyright 2022 TIER IV, Inc.
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

#ifndef DRIVABLE_AREA_EXPANDER__PARAMETERS_HPP_
#define DRIVABLE_AREA_EXPANDER__PARAMETERS_HPP_

#include <drivable_area_expander/types.hpp>
#include <rclcpp/node.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <string>
#include <vector>

namespace drivable_area_expander
{

struct ExpansionParameters
{
  static constexpr auto MAX_EXP_DIST_PARAM = "expansion.max_distance";
  static constexpr auto EGO_EXTRA_OFFSET_FRONT = "expansion.ego.extra_footprint_offset.front";
  static constexpr auto EGO_EXTRA_OFFSET_REAR = "expansion.ego.extra_footprint_offset.rear";
  static constexpr auto EGO_EXTRA_OFFSET_LEFT = "expansion.ego.extra_footprint_offset.left";
  static constexpr auto EGO_EXTRA_OFFSET_RIGHT = "expansion.ego.extra_footprint_offset.right";
  static constexpr auto DYN_OBJECTS_EXTRA_OFFSET_FRONT =
    "expansion.dynamic_objects.extra_footprint_offset.front";
  static constexpr auto DYN_OBJECTS_EXTRA_OFFSET_REAR =
    "expansion.dynamic_objects.extra_footprint_offset.rear";
  static constexpr auto DYN_OBJECTS_EXTRA_OFFSET_LEFT =
    "expansion.dynamic_objects.extra_footprint_offset.left";
  static constexpr auto DYN_OBJECTS_EXTRA_OFFSET_RIGHT =
    "expansion.dynamic_objects.extra_footprint_offset.right";
  static constexpr auto AVOID_DYN_OBJECTS_PARAM = "expansion.dynamic_objects.avoid";
  static constexpr auto AVOID_LINESTRING_TYPES_PARAM = "expansion.avoid_linestring_types";

  double max_expansion_distance{};
  std::vector<std::string> avoid_linestring_types{};
  bool avoid_dynamic_objects{};
  double ego_left_offset;
  double ego_right_offset;
  double ego_rear_offset;
  double ego_front_offset;
  double ego_extra_left_offset;
  double ego_extra_right_offset;
  double ego_extra_rear_offset;
  double ego_extra_front_offset;
  double dynamic_objects_extra_left_offset;
  double dynamic_objects_extra_right_offset;
  double dynamic_objects_extra_rear_offset;
  double dynamic_objects_extra_front_offset;

  ExpansionParameters() = default;
  explicit ExpansionParameters(rclcpp::Node & node)
  {
    max_expansion_distance = node.declare_parameter<double>(MAX_EXP_DIST_PARAM);
    ego_extra_front_offset = node.declare_parameter<double>(EGO_EXTRA_OFFSET_FRONT);
    ego_extra_rear_offset = node.declare_parameter<double>(EGO_EXTRA_OFFSET_REAR);
    ego_extra_left_offset = node.declare_parameter<double>(EGO_EXTRA_OFFSET_LEFT);
    ego_extra_right_offset = node.declare_parameter<double>(EGO_EXTRA_OFFSET_RIGHT);
    dynamic_objects_extra_front_offset =
      node.declare_parameter<double>(DYN_OBJECTS_EXTRA_OFFSET_FRONT);
    dynamic_objects_extra_rear_offset =
      node.declare_parameter<double>(DYN_OBJECTS_EXTRA_OFFSET_REAR);
    dynamic_objects_extra_left_offset =
      node.declare_parameter<double>(DYN_OBJECTS_EXTRA_OFFSET_LEFT);
    dynamic_objects_extra_right_offset =
      node.declare_parameter<double>(DYN_OBJECTS_EXTRA_OFFSET_RIGHT);
    avoid_linestring_types =
      node.declare_parameter<std::vector<std::string>>(AVOID_LINESTRING_TYPES_PARAM);
    avoid_dynamic_objects = node.declare_parameter<bool>(AVOID_DYN_OBJECTS_PARAM);

    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
    ego_left_offset = vehicle_info.vehicle_width_m / 2.0;
    ego_right_offset = -vehicle_info.vehicle_width_m / 2.0;
    ego_rear_offset = -vehicle_info.rear_overhang_m;
    ego_front_offset = vehicle_info.wheel_base_m + vehicle_info.front_overhang_m;
  }
};

struct PreprocessingParameters
{
  static constexpr auto DOWNSAMPLING_PARAM = "preprocessing.downsample_factor";
  static constexpr auto MAX_LENGTH_PARAM = "preprocessing.max_length";

  int downsample_factor{};
  double max_length{};

  PreprocessingParameters() = default;
  explicit PreprocessingParameters(rclcpp::Node & node)
  {
    downsample_factor = node.declare_parameter<int>(DOWNSAMPLING_PARAM);
    max_length = node.declare_parameter<double>(MAX_LENGTH_PARAM);
  }
  bool updateDownsampleFactor(const int new_downsample_factor)
  {
    if (new_downsample_factor > 0) {
      downsample_factor = new_downsample_factor;
      return true;
    }
    return false;
  }
};

}  // namespace drivable_area_expander
#endif  // DRIVABLE_AREA_EXPANDER__PARAMETERS_HPP_
