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

#ifndef DRIVABLE_AREA_EXPANDER__DEBUG_HPP_
#define DRIVABLE_AREA_EXPANDER__DEBUG_HPP_

#include "drivable_area_expander/obstacles.hpp"
#include "drivable_area_expander/types.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace drivable_area_expander
{
/// @brief make the visualization Marker of the given linestring
/// @param[in] ls linestring to turn into a marker
/// @param[in] z z-value to use in the marker
/// @return marker representing the linestring
visualization_msgs::msg::Marker makeLinestringMarker(const linestring_t & ls, const double z);

/// @brief make the visualization Marker of the given polygon
/// @param[in] polygon polygon to turn into a marker
/// @param[in] z z-value to use in the marker
/// @return marker representing the polygon
visualization_msgs::msg::Marker makePolygonMarker(const polygon_t & polygon, const double z);

/// @brief make debug marker array
/// @param[in] footprint footprint polygon
/// @param[in] uncrossable_lines uncrossable lines
/// @param[in] marker_z z-value to use for markers
/// @return marker array
visualization_msgs::msg::MarkerArray makeDebugMarkers(
  const multipolygon_t & footprint, const multilinestring_t & uncrossable_lines,
  const multipolygon_t & predicted_paths, const double marker_z);

}  // namespace drivable_area_expander
#endif  // DRIVABLE_AREA_EXPANDER__DEBUG_HPP_
