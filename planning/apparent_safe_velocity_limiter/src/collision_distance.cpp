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

#include <apparent_safe_velocity_limiter/collision_distance.hpp>

namespace apparent_safe_velocity_limiter
{
segment_t forwardSimulatedSegment(
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

polygon_t generateFootprint(const segment_t & segment, const double lateral_offset)
{
  return generateFootprint(linestring_t{segment.first, segment.second}, lateral_offset);
}

polygon_t generateFootprint(const linestring_t & linestring, const double lateral_offset)
{
  multipolygon_t footprint;
  namespace strategy = bg::strategy::buffer;
  bg::buffer(
    linestring, footprint, strategy::distance_symmetric<double>(lateral_offset),
    strategy::side_straight(), strategy::join_miter(), strategy::end_flat(),
    strategy::point_square());
  return footprint[0];
}

std::optional<double> distanceToClosestCollision(
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

polygon_t createObjectPolygon(
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

multipolygon_t createObjectPolygons(
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
