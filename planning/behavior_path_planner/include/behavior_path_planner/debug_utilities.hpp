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
#ifndef BEHAVIOR_PATH_PLANNER__DEBUG_UTILITIES_HPP_
#define BEHAVIOR_PATH_PLANNER__DEBUG_UTILITIES_HPP_

#include "behavior_path_planner/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <tf2/utils.h>

#include <string>
#include <vector>

namespace marker_utils
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::ShiftPointArray;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::Pose;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

inline int64_t bitShift(int64_t original_id) { return original_id << (sizeof(int32_t) * 8 / 2); }

Marker initializeMarker(
  std::string && frame_id, const std::string & ns, const Marker::_type_type & type);

Marker initializeMarker(
  std::string && frame_id, const std::string & ns, const int id, const Marker::_type_type & type);

MarkerArray createPoseMarkerArray(
  const Pose & pose, const std::string & ns, const int id, const float r, const float g,
  const float b);

MarkerArray createPoseLineMarkerArray(
  const Pose & pose, const std::string & ns, const int64_t id, const float r, const float g,
  const float b);

MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, const std::string & ns, const int64_t lane_id, const float r,
  const float g, const float b);

MarkerArray createShiftPointMarkerArray(
  const ShiftPointArray & shift_points, const double base_shift, const std::string & ns,
  const float & r, const float & g, const float & b, const float & w);

MarkerArray createShiftLengthMarkerArray(
  const std::vector<double> & shift_distance,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & reference, const std::string & ns,
  const float r, const float g, const float b);

MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, const std::string & ns, const float r,
  const float g, const float b);

MarkerArray createFurthestLineStringMarkerArray(const lanelet::ConstLineStrings3d & linestrings);

MarkerArray createPolygonMarkerArray(
  const Polygon & polygon, const std::string & ns, const int64_t lane_id, const float r,
  const float g, const float b);

MarkerArray createObjectsMarkerArray(
  const PredictedObjects & objects, const std::string & ns, const int64_t lane_id, const float r,
  const float g, const float b);

}  // namespace marker_utils

#endif  // BEHAVIOR_PATH_PLANNER__DEBUG_UTILITIES_HPP_
