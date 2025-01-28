// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON_UNIVERSE__POLYGON_UTILS_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON_UNIVERSE__POLYGON_UTILS_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <rclcpp/time.hpp>

#include "autoware_perception_msgs/msg/predicted_path.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include <boost/geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
namespace polygon_utils
{
namespace bg = boost::geometry;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;

using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
namespace bg = boost::geometry;
using autoware::vehicle_info_utils::VehicleInfo;

struct PointWithStamp
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Point point;
};

std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Pose obj_pose, const rclcpp::Time obj_stamp, const Shape & obj_shape,
  const double dist_to_bumper);

std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const size_t collision_idx,
  const std::vector<PointWithStamp> & collision_points, const double dist_to_bumper);

std::vector<PointWithStamp> get_collision_points(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const rclcpp::Time & obstacle_stamp, const PredictedPath & predicted_path, const Shape & shape,
  const rclcpp::Time & current_time, const bool is_driving_forward,
  std::vector<size_t> & collision_index, const double max_dist = std::numeric_limits<double>::max(),
  const double max_prediction_time_for_collision_check = std::numeric_limits<double>::max());

std::vector<Polygon2d> create_one_step_polygons(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length);
}  // namespace polygon_utils
}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON_UNIVERSE__POLYGON_UTILS_HPP_
