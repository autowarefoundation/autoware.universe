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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__COLLISION_DISTANCE_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__COLLISION_DISTANCE_HPP_

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <boost/assign.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

namespace apparent_safe_velocity_limiter
{
namespace bg = boost::geometry;
using point_t = bg::model::d2::point_xy<double>;
using polygon_t = bg::model::polygon<point_t>;
using multipolygon_t = bg::model::multi_polygon<polygon_t>;
using segment_t = bg::model::segment<point_t>;
using linestring_t = bg::model::linestring<point_t>;
using multilinestring_t = bg::model::multi_linestring<linestring_t>;

/// @brief generate a segment to where ego would reach with constant velocity and heading
/// @param [in] trajectory_point origin of the segment
/// @param [in] duration duration of the forward projection
/// @param [in] extra_distance distance to add ahead of the projected point
/// @return segment from the trajectory_point to its position after duration + the extra_distance
segment_t forwardSimulatedSegment(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & trajectory_point, const double duration,
  const double extra_distance);

/// @brief generate a footprint from a segment and a lateral offset
/// @param [in] segment segment from which to create the footprint
/// @param [in] lateral_offset offset around the segment used to create the footprint
/// @return footprint polygon
polygon_t forwardSimulatedFootprint(const segment_t & segment, const double lateral_offset);

/// @brief calculate the closest distance to a collision
/// @param [in] segment forard projection of a trajectory point
/// @param [in] footprint footprint of the segment
/// @param [in] obstacles set of obstacles as linestrings
/// @return distance to the closest collision if any
std::optional<double> distanceToClosestCollision(
  const segment_t & segment, const polygon_t & footprint, const multilinestring_t & obstacles);

/// @brief create a polygon from an object represented by a pose and a size
/// @param [in] pose pose of the object
/// @param [in] dimensions dimensions of the object
/// @param [in] buffer buffer to add to the dimensions of the object
/// @return polygon of the object
polygon_t createObjectPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & dimensions,
  const double buffer);

/// @brief create polygons from a set of predicted object
/// @param [in] objects objects from which to create polygons
/// @param [in] buffer buffer to add to the objects dimensions
/// @param [in] min_velocity objects with velocity lower will be ignored
/// @return polygons of the objects
multipolygon_t createObjectPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const double buffer,
  const double min_velocity);
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__COLLISION_DISTANCE_HPP_
