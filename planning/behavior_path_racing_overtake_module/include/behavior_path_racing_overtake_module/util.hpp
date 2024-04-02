// Copyright 2024 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__UTIL_HPP_
#define BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__UTIL_HPP_

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <limits>
#include <optional>
#include <tuple>

namespace behavior_path_planner::racing_overtake::util
{

using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;

/**
 * @brief Add lateral offset to the path
 * @param path input path
 * @param lateral_offset lateral offset to be added
 */
void addLateralOffset(PathWithLaneId * path, const double lateral_offset);

struct RivalVehicle
{
  PredictedObject object;
  double longitudinal_from_ego;
  double lateral_from_ego;
};

/**
 * @brief Detect rival vehicle in ego course
 * @param ego_pose ego pose
 * @param centerline_path centerline path
 * @param objects predicted objects
 * @param ego_course_width ego course width
 * @return std::optional<RivalVehicle> return rival vehicle if exists
 */
std::optional<RivalVehicle> detectRivalVehicleInEgoCourse(
  const Pose & ego_pose, const PathWithLaneId & centerline_path,
  const std::vector<PredictedObject> & objects,
  const double ego_course_width = std::numeric_limits<double>::max());

/**
 * @brief Calculate overtake path
 * @param reference_path reference path
 * @param object predicted object
 * @param base_shift_length base shift length
 * @return std::tuple<PathWithLaneId, Pose, double> return overtake path, overtake end pose and
 * course after overtake
 */
std::tuple<PathWithLaneId, Pose, double> calcOvertakePath(
  const PathWithLaneId & reference_path, const PredictedObject & object,
  const double current_course_shift_length = 0.0);

/**
 * @brief Calculate back to center path
 * @param reference_path reference path
 * @param ego_pose ego pose
 * @param shift_length shift length
 * @param shift_start_length shift start length
 * @param shift_end_length shift end length
 * @return std::pair<PathWithLaneId, Pose> return back to center path and back to center end pose
 */
std::pair<PathWithLaneId, Pose> calcBackToCenterPath(
  const PathWithLaneId & reference_path, const Pose & ego_pose, const double shift_length,
  const double shift_start_length, const double shift_end_length);

}  // namespace behavior_path_planner::racing_overtake::util

#endif  // BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__UTIL_HPP_
