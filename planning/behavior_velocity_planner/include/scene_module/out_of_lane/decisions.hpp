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
/// @brief calculate the distance along the ego path between ego and some target path index
/// @param [in] ego_data data related to the ego vehicle
/// @param [in] target_idx target ego path index
/// @return distance between ego and the target [m]
double distance_along_path(const EgoData & ego_data, const size_t target_idx);
/// @brief estimate the time when ego will reach some target path index
/// @param [in] ego_data data related to the ego vehicle
/// @param [in] target_idx target ego path index
/// @return time taken by ego to reach the target [s]
double time_along_path(const EgoData & ego_data, const size_t target_idx);
/// @brief use an object's predicted paths to estimate the times it will reach the enter and exit
/// points of an overlapping range
/// @details times when the predicted paths of the object enters/exits the range are calculated
/// but may not exist (e.g,, predicted path ends before reaching the end of the range)
/// @param [in] object dynamic object
/// @param [in] range overlapping range
/// @return an optional pair (time at enter [s], time at exit [s]). If the dynamic object drives in
/// the opposite direction, time at enter > time at exit
std::optional<std::pair<double, double>> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range);
/// @brief use the lanelet map to estimate the times when an object will reach the enter and exit
/// points of an overlapping range
/// @param [in] object dynamic object
/// @param [in] range overlapping range
/// @param [in] lanelets objects to consider
/// @param [in] route_handler route handler used to estimate the path of the dynamic object
/// @return an optional pair (time at enter [s], time at exit [s]). If the dynamic object drives in
/// the opposite direction, time at enter > time at exit.
std::optional<std::pair<double, double>> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range,
  const lanelet::ConstLanelets & lanelets,
  const std::shared_ptr<route_handler::RouteHandler> & route_handler);
/// @brief calculate decisions to stop or slowdown before some overlapping ranges
/// @param [in] ranges overlapping ranges
/// @param [in] ego_data data about the ego vehicle
/// @param [in] objects dynamic objects to consider
/// @param [in] route_handler route handler
/// @param [in] lanelets lanelets to consider
/// @param [in] params parameters
/// @return return the calculated decisions to slowdown or stop
std::vector<Slowdown> calculate_decisions(
  const OverlapRanges & ranges, const EgoData & ego_data,
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const std::shared_ptr<route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & lanelets, const PlannerParam & params);
}  // namespace behavior_velocity_planner::out_of_lane

#endif  // SCENE_MODULE__OUT_OF_LANE__DECISIONS_HPP_
