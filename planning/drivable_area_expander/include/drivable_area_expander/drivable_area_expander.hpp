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

#ifndef DRIVABLE_AREA_EXPANDER__DRIVABLE_AREA_EXPANDER_HPP_
#define DRIVABLE_AREA_EXPANDER__DRIVABLE_AREA_EXPANDER_HPP_

#include "drivable_area_expander/obstacles.hpp"
#include "drivable_area_expander/parameters.hpp"
#include "drivable_area_expander/types.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <string>
#include <vector>

namespace drivable_area_expander
{
/// @brief build a drivable area expanded with a footprint but not crossing some lines and predicted
/// paths
/// @param[inout] left_bound left drivable area bound to expand
/// @param[inout] right_bound right drivable area bound to expand
/// @param[in] footprint polygon to make drivable
/// @param[in] predicted_paths polygons to make undrivable
/// @param[in] uncrossable_lines lines beyond which not to extend the drivable area
/// @param[in] origin ego position
multilinestring_t expandDrivableArea(
  std::vector<Point> & left_bound, std::vector<Point> & right_bound, const polygon_t & footprint,
  const multipolygon_t & predicted_paths, const multilinestring_t & uncrossable_lines,
  const point_t & origin);

/// @brief create polygons from the predicted paths of the given objects
/// @param[in] predicted_objects predicted objects
/// @param[in] params expansion parameters
/// @return the objects' predicted paths polygons
multipolygon_t createPredictedPathPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & predicted_objects,
  const ExpansionParameters & params);

/// @brief create lines around the path
/// @param[in] path input path
/// @param[in] max_expansion_distance maximum distance from the path
/// @return line around the path with the given distance
linestring_t createMaxExpansionLine(const Path & path, const double max_expansion_distance);
}  // namespace drivable_area_expander

#endif  // DRIVABLE_AREA_EXPANDER__DRIVABLE_AREA_EXPANDER_HPP_
