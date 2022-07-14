// Copyright 2022 Tier IV, Inc.
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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__DEBUG_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__DEBUG_HPP_

#include "apparent_safe_velocity_limiter/types.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace apparent_safe_velocity_limiter
{
/// @brief make the visualization Marker of the given linestring
/// @param[in] line linestring to turn into a marker
/// @param[in] z z-value to use in the marker
/// @return marker representing the linestring
visualization_msgs::msg::Marker makeLinestringMarker(const linestring_t & line, const Float z);

/// @brief make the visualization Markers of the given linestrings
/// @param[in] lines linestrings to turn into markers
/// @param[in] z z-value to use in the markers
/// @param[in] ns namespace to use in the markers
/// @return markers representing the linestrings
visualization_msgs::msg::MarkerArray makeLinestringMarkers(
  const multilinestring_t & lines, const Float z, const std::string & ns);

/// @brief make the Marker showing the apparent safety envelope of the given trajectory
/// @param[in] trajectory trajectory for which to make the apparent safety envelope
/// @param[in] projection_params parameters for forward projection
/// @return marker representing the apparent safety envelope of the trajectory
visualization_msgs::msg::Marker makeEnvelopeMarker(
  const Trajectory & trajectory, ProjectionParameters & projection_params);

/// @brief make debug marker array of obstacle lines and original & adjusted envelopes
/// @param[in] original_trajectory trajectory with original velocities
/// @param[in] adjusted_trajectory trajectory with adjusted velocities
/// @param[in] polygons obstacle polygons
/// @param[in] projection_params parameters for forward projection
/// @param[in] z z-value to use for markers
/// @return marker array with the original and adjusted envelope and the obstacle lines
visualization_msgs::msg::MarkerArray makeDebugMarkers(
  const multilinestring_t & lines, const std::vector<polygon_t> & footprint_polygons,
  const polygon_t & envelope_polygon, const polygon_t & safe_envelope_polygon,
  const Float marker_z);

/// @brief make debug marker with Points for the given polygons
/// @param[in] polygons footprint polygons
/// @param[in] z z-value to use for markers
/// @return marker array with the original and adjusted envelope and the obstacle lines
visualization_msgs::msg::Marker makePolygonPointsMarker(
  const std::vector<polygon_t> & polygons, const Float z);
}  // namespace apparent_safe_velocity_limiter
#endif  // APPARENT_SAFE_VELOCITY_LIMITER__DEBUG_HPP_
