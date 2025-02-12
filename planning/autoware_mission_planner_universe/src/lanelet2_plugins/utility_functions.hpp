// Copyright 2019-2024 Autoware Foundation
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

#ifndef LANELET2_PLUGINS__UTILITY_FUNCTIONS_HPP_
#define LANELET2_PLUGINS__UTILITY_FUNCTIONS_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <vector>

namespace autoware::mission_planner_universe::lanelet2
{
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;

template <typename T>
bool exists(const std::vector<T> & vectors, const T & item)
{
  for (const auto & i : vectors) {
    if (i == item) {
      return true;
    }
  }
  return false;
}

autoware::universe_utils::Polygon2d convert_linear_ring_to_polygon(
  autoware::universe_utils::LinearRing2d footprint);
void insert_marker_array(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2);

std::vector<geometry_msgs::msg::Point> convertCenterlineToPoints(const lanelet::Lanelet & lanelet);
geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d & point, const double lane_yaw);

bool is_in_lane(const lanelet::ConstLanelet & lanelet, const lanelet::ConstPoint3d & point);
bool is_in_parking_space(
  const lanelet::ConstLineStrings3d & parking_spaces, const lanelet::ConstPoint3d & point);
bool is_in_parking_lot(
  const lanelet::ConstPolygons3d & parking_lots, const lanelet::ConstPoint3d & point);
double project_goal_to_map(
  const lanelet::ConstLanelet & lanelet_component, const lanelet::ConstPoint3d & goal_point);
geometry_msgs::msg::Pose get_closest_centerline_pose(
  const lanelet::ConstLanelets & road_lanelets, const geometry_msgs::msg::Pose & point,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info);

}  // namespace autoware::mission_planner_universe::lanelet2
#endif  // LANELET2_PLUGINS__UTILITY_FUNCTIONS_HPP_
