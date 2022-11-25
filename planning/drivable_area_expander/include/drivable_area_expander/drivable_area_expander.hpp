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
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <string>
#include <vector>

namespace drivable_area_expander
{
// TODO(Maxime): move to other file
polygon_t rotatePolygon(const polygon_t & polygon, const double angle);
polygon_t translatePolygon(const polygon_t & polygon, const double x, const double y);
/// @brief create the footprint polygon from a path
/// @param[in] path the path for which to create a footprint
/// @param[in] params expansion parameters defining how to create the footprint
/// @return polygon footprint of the path
polygon_t createPathFootprint(const Path & path, const ExpansionParameters & params);
/// @brief make the given footprint "drivable" in the given drivable_area
/// @param[in] drivable_area input drivable_area
/// @param[in] footprint polygon to make drivable
/// @return expanded drivable area
OccupancyGrid expandDrivableArea(const OccupancyGrid & drivable_area, const polygon_t & footprint);

linestring_t createMaxExpansionLines(const Path & path, const double max_expansion_distance);
}  // namespace drivable_area_expander

#endif  // DRIVABLE_AREA_EXPANDER__DRIVABLE_AREA_EXPANDER_HPP_
