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

namespace apparent_safe_velocity_limiter
{
/// @brief make the visualization Marker of the given polygon
/// @param[in] polygon polygon to turn into a marker
/// @param[in] id id of the marker
/// @return marker representing the polygon
visualization_msgs::msg::Marker makePolygonMarker(const linestring_t & polygon, const Float z);

/// @brief make the Marker showing the apparent safety envelope of the given trajectory
/// @param[in] trajectory trajectory for which to make the apparent safety envelope
/// @return marker representing the apparent safety envelope of the trajectory
visualization_msgs::msg::Marker makeEnvelopeMarker(
  const Trajectory & trajectory, const ForwardProjectionFunction & fn);

/// @brief make debug marker array of obstacle polygons and original & adjusted envelopes
/// @param[in] original_trajectory trajectory with original velocities
/// @param[in] adjusted_trajectory trajectory with adjusted velocities
/// @param[in] polygons obstacle polygons
/// @param[in] fwd_sim_fn function to do forward projection at each trajectory point
/// @param[in] polygon_z z value to use for polygons
/// @return marker array with the original and adjusted envelope and the obstacle polygons
visualization_msgs::msg::MarkerArray makeDebugMarkers(
  const Trajectory & original_trajectory, const Trajectory & adjusted_trajectory,
  const multilinestring_t & polygons, const ForwardProjectionFunction & fwd_sim_fn,
  const Float polygon_z);
}  // namespace apparent_safe_velocity_limiter
#endif  // APPARENT_SAFE_VELOCITY_LIMITER__DEBUG_HPP_
