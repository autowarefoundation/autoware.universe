// Copyright 2023 Tier IV, Inc.
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

#ifndef SCENE_MODULE__OUT_OF_LANE__DECISIONS_HPP_
#define SCENE_MODULE__OUT_OF_LANE__DECISIONS_HPP_

#include "scene_module/out_of_lane/types.hpp"

#include <route_handler/route_handler.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::out_of_lane
{
double distance_along_path(const EgoData & ego_data, const size_t target_idx);

double time_along_path(const EgoData & ego_data, const size_t target_idx);

/// @brief estimate the times when an object will enter and exit an overlapping range using its
/// predicted paths
/// @details times when the predicted paths of the object enters/exits the range are calculated
/// but may not exist (e.g,, predicted path ends before reaching the end of the range)
/// so we also calculate the min/max time inside the range.
std::optional<std::pair<double, double>> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range);

/// @brief estimate the times when an object will enter and exit an overlapping range assuming it
/// follows some lanelet
/// @details the enter/exit is relative to ego and may be inversed if the object drives in the
/// opposite direction
std::optional<std::pair<double, double>> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range,
  const lanelet::ConstLanelets & lanelets,
  const std::shared_ptr<route_handler::RouteHandler> & route_handler);

std::vector<Slowdown> calculate_decisions(
  const OverlapRanges & ranges, const EgoData & ego_data,
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const std::shared_ptr<route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & lanelets, const PlannerParam & params, DebugData & debug);

}  // namespace behavior_velocity_planner::out_of_lane

#endif  // SCENE_MODULE__OUT_OF_LANE__DECISIONS_HPP_
