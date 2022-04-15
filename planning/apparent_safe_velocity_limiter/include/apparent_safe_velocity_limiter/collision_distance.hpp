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

#ifndef apparent_safe_velocity_limiter__COLLISION_DISTANCE_HPP_
#define apparent_safe_velocity_limiter__COLLISION_DISTANCE_HPP_

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
inline segment_t forwardSimulatedSegment(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & trajectory_point, const double duration,
  const double extra_distance)
{
  const auto heading = tf2::getYaw(trajectory_point.pose.orientation);
  const auto velocity = trajectory_point.longitudinal_velocity_mps;
  const auto length = velocity * duration + extra_distance;
  const auto from = point_t{trajectory_point.pose.position.x, trajectory_point.pose.position.y};
  const auto to =
    point_t{from.x() + std::cos(heading) * length, from.y() + std::sin(heading) * length};
  return segment_t{from, to};
}

/// @brief generate a footprint from a segment and a lateral offset
/// @param [in] segment segment from which to create the footprint
/// @param [in] lateral_offset offset around the segment used to create the footprint
/// @return footprint polygon
inline polygon_t forwardSimulatedFootprint(const segment_t & segment, const double lateral_offset)
{
  multipolygon_t footprint;
  namespace strategy = bg::strategy::buffer;
  bg::buffer(
    linestring_t{segment.first, segment.second}, footprint,
    strategy::distance_symmetric<double>(lateral_offset), strategy::side_straight(),
    strategy::join_miter(), strategy::end_flat(), strategy::point_square());
  return footprint[0];
}

/// @brief calculate the closest distance to a collision
/// @param [in] segment forard projection of a trajectory point
/// @param [in] footprint footprint of the segment
/// @param [in] obstacles set of obstacles as linestrings
/// @return distance to the closest collision if any
inline std::optional<double> distanceToClosestCollision(
  const segment_t & segment, const polygon_t & footprint, const multilinestring_t & obstacles)
{
  const auto segment_heading =
    std::atan2(segment.second.y() - segment.first.y(), segment.second.x() - segment.first.x());
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & obstacle : obstacles) {
    multilinestring_t intersection_lines;
    if (bg::intersection(footprint, obstacle, intersection_lines)) {
      for (const auto & intersection_line : intersection_lines) {
        for (const auto & obs_point : intersection_line) {
          // Calculate longitudinal distance to the collision point along the segment
          const auto collision_heading =
            std::atan2(obs_point.y() - segment.first.y(), obs_point.x() - segment.first.x());
          const auto angle = segment_heading - collision_heading;
          const auto hypot_length = bg::distance(obs_point, segment.first);
          const auto long_dist = std::abs(std::cos(angle)) * hypot_length;
          min_dist = std::min(min_dist, long_dist);
        }
      }
    }
  }
  std::optional<double> distance;
  if (min_dist != std::numeric_limits<double>::max()) distance = min_dist;
  return distance;
}

/// @brief create a polygon from an object represented by a pose and a size
/// @param [in] pose pose of the object
/// @param [in] dimensions dimensions of the object
/// @param [in] buffer buffer to add to the dimensions of the object
/// @return polygon of the object
inline polygon_t createObjectPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & dimensions,
  const double buffer)
{
  // (objects.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions);
  // rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double h = dimensions.x + buffer;
  const double w = dimensions.y + buffer;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  polygon_t obj_poly;
  boost::geometry::exterior_ring(obj_poly) = boost::assign::list_of<point_t>(h / 2.0, w / 2.0)(
    -h / 2.0, w / 2.0)(-h / 2.0, -w / 2.0)(h / 2.0, -w / 2.0)(h / 2.0, w / 2.0);

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  polygon_t rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  polygon_t translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

/// @brief create polygons from a set of predicted object
/// @param [in] objects objects from which to create polygons
/// @param [in] buffer buffer to add to the objects dimensions
/// @param [in] min_velocity objects with velocity lower will be ignored
/// @return polygons of the objects
inline multipolygon_t createObjectPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const double buffer,
  const double min_velocity)
{
  multipolygon_t polygons;
  for (const auto & object : objects.objects) {
    if (
      object.kinematics.initial_twist_with_covariance.twist.linear.x >= min_velocity ||
      object.kinematics.initial_twist_with_covariance.twist.linear.x <= -min_velocity) {
      polygons.push_back(createObjectPolygon(
        object.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions, buffer));
    }
  }
  return polygons;
}
}  // namespace apparent_safe_velocity_limiter

#endif  // apparent_safe_velocity_limiter__COLLISION_DISTANCE_HPP_
