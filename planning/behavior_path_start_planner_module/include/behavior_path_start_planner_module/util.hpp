// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_START_PLANNER_MODULE__UTIL_HPP_
#define BEHAVIOR_PATH_START_PLANNER_MODULE__UTIL_HPP_

#include "behavior_path_planner_common/data_manager.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"

#include <route_handler/route_handler.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <utility>

namespace behavior_path_planner::start_planner_utils
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using route_handler::RouteHandler;

PathWithLaneId getBackwardPath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & current_pose, const Pose & backed_pose, const double velocity);

/**
 * @brief Get a sequence of lanelets for pulling out from the current position.
 *
 * If the ego vehicle's current position is on a shoulder lane, the function retrieves a sequence of
 * shoulder lanelets. If it is on a road lane, the function returns a sequence of road lanelets.
 *
 */
lanelet::ConstLanelets getPullOutLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length);
Pose getBackedPose(
  const Pose & current_pose, const double & yaw_shoulder_lane, const double & back_distance);

/**
 * @brief Calculate the minimum arc length distance from the ego vehicle to static objects within
 the same lanelets.
 *
 * Calculates the minimum arc length distance between the ego vehicle and static objects in given
 * lanelets. It compares each corner of the vehicle's transformed footprint with every corner of
 * object polygons to find the shortest distance within the lanelet system.
 */
double calcMinArcLengthDistanceFromEgoToObjects(
  const tier4_autoware_utils::LinearRing2d & local_vehicle_footprint, const Pose & ego_pose,
  const lanelet::ConstLanelets & lanelets, const PredictedObjects & static_objects);

double getArcLengthForPoint(
  const lanelet::ConstLanelets & lanelets, const tier4_autoware_utils::Point2d & point);

}  // namespace behavior_path_planner::start_planner_utils

#endif  // BEHAVIOR_PATH_START_PLANNER_MODULE__UTIL_HPP_
