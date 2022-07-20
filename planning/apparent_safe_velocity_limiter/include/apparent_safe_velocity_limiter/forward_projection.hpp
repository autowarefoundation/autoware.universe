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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__FORWARD_PROJECTION_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__FORWARD_PROJECTION_HPP_

#include "apparent_safe_velocity_limiter/types.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace apparent_safe_velocity_limiter
{
/// @brief generate a segment to where ego would reach with constant velocity and heading
/// @param [in] origin origin of the segment
/// @param [in] params parameters of the forward projection
/// @return segment from the origin to its position after duration + the extra_distance
segment_t forwardSimulatedSegment(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params);

/// @brief generate lines for the forward projection using the bicycle model
/// @param [in] origin origin of the segment
/// @param [in] params parameters of the forward projection
/// @return lines from the origin to its positions after forward projection
multilinestring_t bicycleProjectionLines(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params);

/// @brief generate the footprint polygon of the forward projection
/// @param [in] origin origin of the segment
/// @param [in] params parameters of the forward projection
/// @param [in] lateral_offset lateral offset around the projection to make the footprint
/// @param [out] projected_straight_segment the segment of the forward projection
/// @return lines from the origin to its positions after forward projection
polygon_t forwardSimulatedPolygon(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params,
  const double lateral_offset, segment_t & projected_straight_segment);

/// @brief generate a footprint from a segment and a lateral offset
/// @param [in] segment segment from which to create the footprint
/// @param [in] lateral_offset offset around the segment used to create the footprint
/// @return footprint polygon
polygon_t generateFootprint(const segment_t & segment, const double lateral_offset);

/// @brief generate a footprint from a linestring and a lateral offset
/// @param [in] linestring linestring from which to create the footprint
/// @param [in] lateral_offset offset around the segment used to create the footprint
/// @return footprint polygon
polygon_t generateFootprint(const linestring_t & linestring, const double lateral_offset);

/// @brief generate a footprint from multiple linestrings and a lateral offset
/// @param [in] linestrings linestring from which to create the footprint
/// @param [in] lateral_offset offset around the segment used to create the footprint
/// @return footprint polygon
polygon_t generateFootprint(const multilinestring_t & linestrings, const double lateral_offset);
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__FORWARD_PROJECTION_HPP_
