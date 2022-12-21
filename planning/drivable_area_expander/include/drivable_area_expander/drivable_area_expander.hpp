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

/// @brief filter the footprint polygons such that it does not cross the given predicted paths and uncrossable lines
/// @param[in] footprint ego footprints to filter
/// @param[in] predicted_paths predicted footprints of dynamic objects
/// @param[in] uncrossable_lines lines that should not be crossed by the ego footprint
/// @return the filtered polygons
multipolygon_t filterFootprint(const multipolygon_t & footprint, const multipolygon_t & predicted_paths, const multilinestring_t & uncrossable_lines);

/// @brief expand the given drivable area with the given footprint
/// @param[inout] left_bound left drivable area bound to expand
/// @param[inout] right_bound right drivable area bound to expand
/// @param[in] footprint polygon to make drivable
multilinestring_t expandDrivableArea(std::vector<Point> & left_bound, std::vector<Point> & right_bound, const multipolygon_t & footprint);

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
