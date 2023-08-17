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

// #ifndef BEHAVIOR_PATH_PLANNER__UTILS__START_PLANNER__UTIL_HPP_
// #define BEHAVIOR_PATH_PLANNER__UTILS__START_PLANNER__UTIL_HPP_

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__OBJECTS_FILTERING_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__OBJECTS_FILTERING_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2/utils.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner::utils::path_safety_checker
{

using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;

/**
 * @brief Separate index of the obstacles into two part based on whether the object is within
 * lanelet.
 * @return Indices of objects pair. first objects are in the lanelet, and second others are out of
 * lanelet.
 */
std::pair<std::vector<size_t>, std::vector<size_t>> separateObjectIndicesByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets);

/**
 * @brief Separate the objects into two part based on whether the object is within lanelet.
 * @return Objects pair. first objects are in the lanelet, and second others are out of lanelet.
 */
std::pair<PredictedObjects, PredictedObjects> separateObjectsByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets);

std::vector<PredictedPathWithPolygon> getPredictedPathFromObj(
  const ExtendedPredictedObject & obj, const bool & is_use_all_predicted_path);

std::vector<PoseWithVelocityStamped> convertToPredictedPath(
  const std::vector<PathPointWithLaneId> & path_points,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params);

PredictedObjects filterObjectsByVelocity(const PredictedObjects & objects, double lim_v);

PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, double min_v, double max_v);

void filterObjectsByPosition(
  const PredictedObjects & objects, const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Point & current_pose, const double forward_distance,
  const double backward_distance);

void filterObjectsByClass(
  PredictedObjects & objects, const ObjectTypesToCheck & target_object_types);

bool isCentroidWithinLanelets(
  const PredictedObject & object, const lanelet::ConstLanelets & target_lanelets);

ExtendedPredictedObject transform(
  const PredictedObject & object, const double safety_check_time_horizon,
  const double safety_check_time_resolution);

TargetObjectsOnLane createTargetObjectsOnLane(
  const std::shared_ptr<const PlannerData> & planner_data,
  const PredictedObjects & filtered_objects,
  const std::shared_ptr<ObjectsFilteringParams> & params);

PredictedObjects filterObject(
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<ObjectsFilteringParams> & params);

TargetObjectsOnLane getSafetyCheckTargetObjects(
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<ObjectsFilteringParams> & params);

}  // namespace behavior_path_planner::utils::path_safety_checker

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__OBJECTS_FILTERING_HPP_
