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

#ifndef SCENE_MODULE__OUT_OF_LANE__LANELETS_SELECTION_HPP_
#define SCENE_MODULE__OUT_OF_LANE__LANELETS_SELECTION_HPP_

#include "scene_module/out_of_lane/types.hpp"

#include <route_handler/route_handler.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace behavior_velocity_planner::out_of_lane
{
inline bool contains_lanelet(const lanelet::ConstLanelets & lanelets, const lanelet::Id id)
{
  return std::find_if(lanelets.begin(), lanelets.end(), [&](const auto & l) {
           return l.id() == id;
         }) != lanelets.end();
};

lanelet::ConstLanelets calculate_ignored_lanelets(
  const EgoData & ego_data, const lanelet::ConstLanelets & path_lanelets,
  const route_handler::RouteHandler & route_handler, const PlannerParam & params);

lanelet::ConstLanelets calculate_other_lanelets(
  const EgoData & ego_data, const lanelet::ConstLanelets & path_lanelets,
  const lanelet::ConstLanelets & ignored_lanelets,
  const route_handler::RouteHandler & route_handler, const PlannerParam & params);
}  // namespace behavior_velocity_planner::out_of_lane

#endif  // SCENE_MODULE__OUT_OF_LANE__LANELETS_SELECTION_HPP_
