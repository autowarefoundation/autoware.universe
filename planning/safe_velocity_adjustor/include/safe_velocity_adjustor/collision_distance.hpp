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
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/strategies/agnostic/buffer_distance_symmetric.hpp>
#include <boost/geometry/strategies/cartesian/buffer_end_flat.hpp>
#include <boost/geometry/strategies/cartesian/buffer_point_square.hpp>
#include <boost/geometry/strategies/cartesian/buffer_side_straight.hpp>

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
using linestring_t = bg::model::linestring<point_t>;
using polygon_t = bg::model::polygon<point_t>;
using multipolygon_t = bg::model::multi_polygon<polygon_t>;

/// @brief generate a segment to where the vehicle body would be after some duration assuming a
/// constant velocity and heading
inline linestring_t forwardSimulatedVector(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & trajectory_point, const double duration,
  const double extra_distance)
{
  const auto heading = tf2::getYaw(trajectory_point.pose.orientation);
  const auto velocity = trajectory_point.longitudinal_velocity_mps;
  const auto length = velocity * duration + extra_distance;
  const auto from = point_t{trajectory_point.pose.position.x, trajectory_point.pose.position.y};
  const auto to =
    point_t{from.x() + std::cos(heading) * length, from.y() + std::sin(heading) * length};
  return linestring_t{from, to};
}

/// @brief generate a footprint from a segment and a vehicle width
inline polygon_t forwardSimulatedFootprint(const linestring_t & vector, const double vehicle_width)
{
  multipolygon_t footprint;
  namespace strategy = bg::strategy::buffer;
  bg::buffer(
    vector, footprint, strategy::distance_symmetric<double>(vehicle_width / 2),
    strategy::side_straight(), strategy::join_miter(), strategy::end_flat(),
    strategy::point_square());
  return footprint[0];
}

/// @brief calculate the distance to the closest obstacle point colliding with the footprint
inline std::optional<double> distanceToClosestCollision(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & trajectory_point,
  const polygon_t & footprint, const pcl::PointCloud<pcl::PointXYZ> & obstacle_points)
{
  const auto traj_point =
    point_t{trajectory_point.pose.position.x, trajectory_point.pose.position.y};
  auto min_dist = std::numeric_limits<double>::infinity();
  for (const auto & obstacle_point : obstacle_points) {
    const auto obs_point = point_t{obstacle_point.x, obstacle_point.y};
    if (bg::within(obs_point, footprint)) {
      const auto footprint_heading = tf2::getYaw(trajectory_point.pose.orientation);
      const auto collision_heading =
        std::atan2(obs_point.y() - traj_point.y(), obs_point.x() - traj_point.x());
      const auto angle = footprint_heading - collision_heading;
      const auto hypot_length = bg::distance(obs_point, traj_point);
      const auto dist = std::abs(std::cos(angle)) * hypot_length;
      min_dist = std::min(min_dist, dist);
    }
  }
  return (min_dist != std::numeric_limits<double>::infinity() ? min_dist : std::optional<double>());
}
}  // namespace safe_velocity_adjustor

#endif  // SAFE_VELOCITY_ADJUSTOR__COLLISION_DISTANCE_HPP_
