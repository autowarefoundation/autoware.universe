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

#ifndef BEHAVIOR_PATH_PLANNER__UTIL__DRIVABLE_AREA_EXPANSION__DRIVABLE_AREA_EXPANSION_HPP_
#define BEHAVIOR_PATH_PLANNER__UTIL__DRIVABLE_AREA_EXPANSION__DRIVABLE_AREA_EXPANSION_HPP_

#include "behavior_path_planner/util/drivable_area_expansion/footprints.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/types.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <string>
#include <vector>

namespace drivable_area_expansion
{
bool expandDrivableArea(
  PathWithLaneId & path, const DrivableAreaExpansionParameters & params,
  const multilinestring_t & uncrossable_lines,
  const autoware_auto_perception_msgs::msg::PredictedObjects & dynamic_objects);

/// @brief create the footprint polygon from a path
/// @param[in] path the path for which to create a footprint
/// @param[in] params expansion parameters defining how to create the footprint
/// @return footprint polygons of the path
multipolygon_t createPathFootprints(
  const PathWithLaneId & path, const DrivableAreaExpansionParameters & params);

/// @brief create footprints from the predicted paths of the given objects
/// @param[in] predicted_objects predicted objects
/// @param[in] params expansion parameters
/// @return the objects' predicted path footprint polygons
multipolygon_t createPredictedPathFootprints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & predicted_objects,
  const DrivableAreaExpansionParameters & params);
}  // namespace drivable_area_expansion

#endif  // BEHAVIOR_PATH_PLANNER__UTIL__DRIVABLE_AREA_EXPANSION__DRIVABLE_AREA_EXPANSION_HPP_
