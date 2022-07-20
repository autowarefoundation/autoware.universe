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

#include "apparent_safe_velocity_limiter/types.hpp"

#include <Eigen/Core>
#include <apparent_safe_velocity_limiter/collision.hpp>

#include <boost/assign.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <limits>

namespace apparent_safe_velocity_limiter
{
namespace bg = boost::geometry;

std::optional<double> distanceToClosestCollision(
  const linestring_t & projection, const polygon_t & footprint, const multilinestring_t & obstacles,
  const ProjectionParameters & params)
{
  std::optional<double> distance;
  if (projection.empty()) return distance;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & obstacle : obstacles) {
    multilinestring_t intersection_lines;
    if (bg::intersection(footprint, obstacle, intersection_lines)) {
      for (const auto & intersection_line : intersection_lines) {
        for (const auto & obs_point : intersection_line) {
          const auto euclidian_dist = bg::distance(obs_point, projection.front());
          if (params.distance_method == ProjectionParameters::EXACT) {
            if (params.model == ProjectionParameters::PARTICLE) {  // TODO(Maxime CLEMENT): 0 steer
                                                                   // angle bicycle case
              const auto collision_heading = std::atan2(
                obs_point.y() - projection.front().y(), obs_point.x() - projection.front().x());
              const auto angle = params.heading - collision_heading;
              const auto long_dist = std::abs(std::cos(angle)) * euclidian_dist;
              min_dist = std::min(min_dist, long_dist);
            } else {  // BICYCLE model with curved projection
              min_dist =
                std::min(min_dist, arcDistance(projection.front(), params.heading, obs_point));
            }
          } else {  // APPROXIMATION
            min_dist = std::min(min_dist, euclidian_dist);
          }
        }
      }
    }
  }
  if (min_dist != std::numeric_limits<double>::max()) distance = min_dist;
  return distance;
}

double arcDistance(const point_t & origin, const double heading, const point_t & target)
{
  const auto squared_dist = [](const auto & a, const auto & b) {
    return (a.x() - b.x()) * (a.x() - b.x()) + (a.y() - b.y()) * (a.y() - b.y());
  };
  // Circle passing through the origin and the target such that origin+heading is tangent
  const auto normal = Eigen::Vector2d{origin.x(), origin.y()};
  const auto d_normal = Eigen::Vector2d{-std::sin(heading), std::cos(heading)};
  const auto midpoint =
    Eigen::Vector2d{(target.x() + origin.x()) / 2, (target.y() + origin.y()) / 2};
  const auto mid_to_target = Eigen::Vector2d{target.x() - midpoint.x(), target.y() - midpoint.y()};
  const auto circle_center =
    normal + (midpoint - normal).dot(mid_to_target) / (mid_to_target.dot(d_normal)) * d_normal;
  const auto squared_radius = squared_dist(circle_center, origin);
  // Arc distance
  const auto arg = (2 * squared_radius - squared_dist(origin, target)) / (2 * squared_radius);
  const auto angle = std::acos(arg);
  return std::sqrt(squared_radius) * angle;
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
