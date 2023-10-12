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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__DRIVABLE_AREA_EXPANSION_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__DRIVABLE_AREA_EXPANSION_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"

#include <route_handler/route_handler.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>

namespace drivable_area_expansion
{
/// @brief Expand the drivable area based on the path curvature and the vehicle dimensions
/// @param[inout] path path whose drivable area will be expanded
/// @param[inout] planner_data planning data (params, dynamic objects, vehicle info, ...)
void expand_drivable_area(
  PathWithLaneId & path,
  const std::shared_ptr<const behavior_path_planner::PlannerData> planner_data);
}  // namespace drivable_area_expansion

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__DRIVABLE_AREA_EXPANSION_HPP_
