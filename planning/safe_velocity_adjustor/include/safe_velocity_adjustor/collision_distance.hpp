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

#include <Eigen/Core>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <autoware_auto_perception_msgs/msg/detail/predicted_objects__struct.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <boost/assign.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <Eigen/src/Core/util/Constants.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

namespace safe_velocity_adjustor
{
namespace bg = boost::geometry;
using point_t = bg::model::d2::point_xy<double>;
using polygon_t = bg::model::polygon<point_t>;
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

static double a{};
static double b{};
static double c{};
static double d{};
static double e{};
static double f{};
/// @brief calculate the distance to the closest obstacle point colliding with the footprint
inline std::optional<double> distanceToClosestCollision_Eigen(
  const segment_t & vector, const double vehicle_width,
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_points)
{
  tier4_autoware_utils::StopWatch watch;
  watch.tic("a");
  Eigen::ArrayX2d dist_array(obstacle_points.size(), 2);
  const auto dist_matrix = obstacle_points.getMatrixXfMap(3, 4, 0).cast<double>();
  dist_array.col(0) = dist_matrix.row(0).array() - vector.first.x();
  dist_array.col(1) = dist_matrix.row(1).array() - vector.first.y();
  a += watch.toc("a");
  watch.tic("b");
  auto collision_headings = Eigen::ArrayXd(dist_array.rows());
  for (auto row_idx = 0; row_idx < dist_array.rows(); ++row_idx)
    collision_headings(row_idx) = std::atan2(dist_array(row_idx, 1), dist_array(row_idx, 0));
  b += watch.toc("b");
  watch.tic("c");
  const auto traj_heading =
    std::atan2(vector.second.y() - vector.first.y(), vector.second.x() - vector.first.x());
  c += watch.toc("c");
  watch.tic("d");
  const auto angles = traj_heading - collision_headings;
  const auto hypot_lengths =
    (dist_array.col(0).array().square() + dist_array.col(1).array().square()).sqrt();
  d += watch.toc("d");
  watch.tic("e");
  const auto long_dists = angles.cos().abs() * hypot_lengths;
  const auto lat_dists = (hypot_lengths.square() - long_dists.square()).sqrt();
  e += watch.toc("e");
  watch.tic("f");
  const auto min_dist =
    (long_dists + bg::length(vector) * (lat_dists > vehicle_width / 2).cast<double>()).minCoeff();
  f += watch.toc("f");
  return (min_dist <= bg::length(vector) ? min_dist : std::optional<double>());
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

inline polygon_t createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size)
{
  // (objects.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions);
  // rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double h = size.x;
  const double w = size.y;
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

inline std::vector<polygon_t> createObjPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects)
{
  std::vector<polygon_t> polygons;
  for (const auto & object : objects.objects)
    polygons.push_back(createObjPolygon(
      object.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions));
  return polygons;
}

inline bool inPolygons(const point_t & point, const std::vector<polygon_t> & polygons)
{
  for (const auto & polygon : polygons)
    if (bg::distance(point, polygon) < 0.5) return true;
  return false;
}
}  // namespace safe_velocity_adjustor

#endif  // SAFE_VELOCITY_ADJUSTOR__COLLISION_DISTANCE_HPP_
