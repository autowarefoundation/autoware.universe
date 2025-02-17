// Copyright 2023 LeoDrive A.Ş. All rights reserved.
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

#ifndef AUTOWARE__PREDICTED_PATH_CHECKER__UTILS_HPP_
#define AUTOWARE__PREDICTED_PATH_CHECKER__UTILS_HPP_

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/optional.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_path_checker
{

using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using std_msgs::msg::Header;
using PointArray = std::vector<geometry_msgs::msg::Point>;

using TrajectoryPoints = std::vector<TrajectoryPoint>;

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point);

Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double expand_width);

TrajectoryPoint calcInterpolatedPoint(
  const TrajectoryPoints & trajectory, const geometry_msgs::msg::Point & target_point,
  const size_t segment_idx, const bool use_zero_order_hold_for_twist);

std::pair<size_t, TrajectoryPoint> findStopPoint(
  TrajectoryPoints & predicted_trajectory_array, const size_t collision_idx,
  const double stop_margin, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

bool isInBrakeDistance(
  const TrajectoryPoints & trajectory, const size_t stop_idx, const double relative_velocity,
  const double relative_acceleration, const double max_deceleration, const double delay_time_sec);

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point);

bool intersectsInZAxis(const PredictedObject & object, const double z_min, const double z_max);

double getNearestPointAndDistanceForPredictedObject(
  const PointArray & points, const Pose & base_pose,
  geometry_msgs::msg::Point * nearest_collision_point);

Polygon2d convertBoundingBoxObjectToGeometryPolygon(
  const Pose & current_pose, const double & base_to_front, const double & base_to_rear,
  const double & base_to_width);

Polygon2d convertCylindricalObjectToGeometryPolygon(
  const Pose & current_pose, const autoware_perception_msgs::msg::Shape & obj_shape);

Polygon2d convertPolygonObjectToGeometryPolygon(
  const Pose & current_pose, const autoware_perception_msgs::msg::Shape & obj_shape);

Polygon2d convertObjToPolygon(const PredictedObject & obj);

double calcObstacleMaxLength(const autoware_perception_msgs::msg::Shape & shape);

void getCurrentObjectPose(
  PredictedObject & predicted_object, const rclcpp::Time & obj_base_time,
  const rclcpp::Time & current_time);

bool isFrontObstacle(const Pose & ego_pose, const geometry_msgs::msg::Point & obstacle_pos);
}  // namespace autoware::predicted_path_checker

#endif  // AUTOWARE__PREDICTED_PATH_CHECKER__UTILS_HPP_
