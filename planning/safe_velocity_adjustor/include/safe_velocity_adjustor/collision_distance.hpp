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

#ifndef SAFE_VELOCITY_ADJUSTOR__COLLISION_DISTANCE_HPP_
#define SAFE_VELOCITY_ADJUSTOR__COLLISION_DISTANCE_HPP_

#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <optional>

namespace safe_velocity_adjustor
{
namespace bg = boost::geometry;
using point_t = bg::model::d2::point_xy<double>;
using segment_t = bg::model::segment<point_t>;

/// @brief generate a segment to where the vehicle body would be after some duration assuming a
/// constant velocity and heading
inline segment_t forwardSimulatedVector(
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

/// @brief calculate the distance to the closest obstacle point colliding with the footprint
inline std::optional<double> distanceToClosestCollision(
  const segment_t & vector, const double vehicle_width,
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_points)
{
  const auto traj_heading =
    std::atan2(vector.second.y() - vector.first.y(), vector.second.x() - vector.first.x());
  auto min_dist = std::numeric_limits<double>::infinity();
  for (const auto & obstacle_point : obstacle_points) {
    const auto obs_point = point_t{obstacle_point.x, obstacle_point.y};
    const auto collision_heading =
      std::atan2(obs_point.y() - vector.first.y(), obs_point.x() - vector.first.x());
    const auto angle = traj_heading - collision_heading;
    const auto hypot_length = bg::distance(obs_point, vector.first);
    const auto long_dist = std::abs(std::cos(angle)) * hypot_length;
    const auto lat_dist = std::sqrt(hypot_length * hypot_length - long_dist * long_dist);
    if (lat_dist <= vehicle_width / 2) {
      min_dist = std::min(min_dist, long_dist);
    }
  }
  return (min_dist <= bg::length(vector) ? min_dist : std::optional<double>());
}
}  // namespace safe_velocity_adjustor

#endif  // SAFE_VELOCITY_ADJUSTOR__COLLISION_DISTANCE_HPP_
