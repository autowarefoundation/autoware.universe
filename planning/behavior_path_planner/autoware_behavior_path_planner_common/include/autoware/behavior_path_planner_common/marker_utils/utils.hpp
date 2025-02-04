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
#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__MARKER_UTILS__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__MARKER_UTILS__UTILS_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>

#include <cstdint>
#include <string>
#include <vector>

namespace marker_utils
{
using autoware::behavior_path_planner::DrivableLanes;
using autoware::behavior_path_planner::ShiftLineArray;
using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugPair;
using autoware::behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject;
using autoware::behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObjects;
using autoware::universe_utils::Polygon2d;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

inline int64_t bitShift(int64_t original_id)
{
  return original_id << (sizeof(int32_t) * 8 / 2);
}

void addFootprintMarker(
  visualization_msgs::msg::Marker & marker, const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

MarkerArray createFootprintMarkerArray(
  const Pose & base_link_pose, const autoware::vehicle_info_utils::VehicleInfo vehicle_info,
  const std::string && ns, const int32_t & id, const float & r, const float & g, const float & b);

MarkerArray createPointsMarkerArray(
  const std::vector<Point> & points, const std::string & ns, const int32_t id, const float r,
  const float g, const float b);

MarkerArray createPoseMarkerArray(
  const Pose & pose, std::string && ns, const int32_t & id, const float & r, const float & g,
  const float & b);

MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b);

MarkerArray createShiftLineMarkerArray(
  const ShiftLineArray & shift_lines, const double & base_shift, std::string && ns, const float & r,
  const float & g, const float & b, const float & w);

MarkerArray createShiftLengthMarkerArray(
  const std::vector<double> & shift_distance, const PathWithLaneId & reference, std::string && ns,
  const float & r, const float & g, const float & b);

MarkerArray createShiftGradMarkerArray(
  const std::vector<double> & grad, const std::vector<double> & shift_distance,
  const PathWithLaneId & reference, std::string && ns, const float & r, const float & g,
  const float & b);

MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, std::string && ns, const float & r,
  const float & g, const float & b);

MarkerArray createPolygonMarkerArray(
  const Polygon & polygon, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b, const float & w = 0.3);

MarkerArray createObjectsMarkerArray(
  const PredictedObjects & objects, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b);

MarkerArray createDrivableLanesMarkerArray(
  const std::vector<DrivableLanes> & drivable_lanes, std::string && ns);

MarkerArray createPredictedPathMarkerArray(
  const PredictedPath & ego_predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, std::string && ns,
  const int32_t & id, const float & r, const float & g, const float & b);

MarkerArray showPolygon(const CollisionCheckDebugMap & obj_debug_vec, std::string && ns);

MarkerArray showPredictedPath(const CollisionCheckDebugMap & obj_debug_vec, std::string && ns);

MarkerArray showSafetyCheckInfo(const CollisionCheckDebugMap & obj_debug_vec, std::string && ns);

MarkerArray showFilteredObjects(
  const ExtendedPredictedObjects & predicted_objects, const std::string & ns,
  const ColorRGBA & color, int32_t id);
}  // namespace marker_utils

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__MARKER_UTILS__UTILS_HPP_
