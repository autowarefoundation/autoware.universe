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

#include "apparent_safe_velocity_limiter/obstacles.hpp"

#include "apparent_safe_velocity_limiter/occupancy_grid_utils.hpp"
#include "apparent_safe_velocity_limiter/pointcloud_utils.hpp"

#include <boost/assign.hpp>

#include <tf2/utils.h>

namespace apparent_safe_velocity_limiter
{
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

Obstacles createObstacles(
  const OccupancyGrid & occupancy_grid, const PointCloud & pointcloud, const ObstacleMasks & masks,
  tier4_autoware_utils::TransformListener & transform_listener, const std::string & target_frame,
  const ObstacleParameters & obstacle_params)
{
  Obstacles obstacles;
  if (obstacle_params.dynamic_source == ObstacleParameters::OCCUPANCYGRID) {
    auto grid_map = convertToGridMap(occupancy_grid);
    threshold(grid_map, obstacle_params.occupancy_grid_threshold);
    maskPolygons(grid_map, masks);
    obstacles = extractObstacles(grid_map, occupancy_grid);
  } else {
    const auto filtered_pcd = transformPointCloud(pointcloud, transform_listener, target_frame);
    obstacles = extractObstacles(filtered_pcd, obstacle_params.pcd_cluster_max_dist);
    obstacles = filterObstacles(obstacles, masks);
  }
  return obstacles;
}

Obstacles filterObstacles(const Obstacles & obstacles, const ObstacleMasks & masks)
{
  namespace bg = boost::geometry;
  Obstacles filtered_obstacles;
  multilinestring_t masked_obstacles;
  for (auto & obstacle : obstacles) {
    masked_obstacles.clear();
    bg::difference(obstacle, masks.negative_masks, masked_obstacles);
    for (auto & masked_obstacle : masked_obstacles) {
      if (bg::is_empty(masks.positive_mask) || bg::within(masked_obstacle, masks.positive_mask))
        filtered_obstacles.emplace_back(std::move(masked_obstacle));
    }
  }
  return filtered_obstacles;
}

}  // namespace apparent_safe_velocity_limiter
