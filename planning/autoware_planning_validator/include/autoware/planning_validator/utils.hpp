// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware::universe_utils::Polygon2d;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using Point = autoware::universe_utils::Point2d;
using Box = boost::geometry::model::box<Point>;
using BoxTimeIndexPair = std::pair<Box, std::pair<double, std::size_t>>;
using Rtree = boost::geometry::index::rtree<BoxTimeIndexPair, boost::geometry::index::rstar<16, 4>>;

std::pair<double, size_t> getAbsMaxValAndIdx(const std::vector<double> & v);

Trajectory resampleTrajectory(const Trajectory & trajectory, const double min_interval);

void calcCurvature(
  const Trajectory & trajectory, std::vector<double> & curvatures,
  const double curvature_distance = 1.0);

void calcSteeringAngles(
  const Trajectory & trajectory, const double wheelbase, std::vector<double> & steering_array);

std::pair<double, size_t> calcMaxCurvature(const Trajectory & trajectory);

std::pair<double, size_t> calcMaxIntervalDistance(const Trajectory & trajectory);

std::pair<double, size_t> calcMaxLateralAcceleration(const Trajectory & trajectory);

std::pair<double, size_t> getMaxLongitudinalAcc(const Trajectory & trajectory);

std::pair<double, size_t> getMinLongitudinalAcc(const Trajectory & trajectory);

std::pair<double, size_t> calcMaxRelativeAngles(const Trajectory & trajectory);

std::pair<double, size_t> calcMaxSteeringAngles(
  const Trajectory & trajectory, const double wheelbase);

std::pair<double, size_t> calcMaxSteeringRates(
  const Trajectory & trajectory, const double wheelbase);

std::optional<std::vector<autoware_planning_msgs::msg::TrajectoryPoint>> check_collision(
  const PredictedObjects & predicted_objects, const Trajectory & trajectory,
  const geometry_msgs::msg::Point & current_ego_position, const VehicleInfo & vehicle_info,
  const double trajectory_to_object_distance_threshold,
  const double ego_to_object_distance_threshold, const double time_tolerance_threshold);

Rtree make_ego_footprint_rtree(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const VehicleInfo & vehicle_info);

std::optional<PredictedObjects> filter_objects(
  const PredictedObjects & objects,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double trajectory_to_object_distance_threshold,
  const double ego_to_object_distance_threshold);

std::optional<PredictedPath> find_highest_confidence_path(const PredictedObject & object);

void make_predicted_object_rtree(
  const PredictedPath & highest_confidence_path, const Shape & object_shape,
  const double predicted_time_step, std::vector<BoxTimeIndexPair> & predicted_object_rtree_nodes);

std::vector<std::pair<size_t, size_t>> detect_collisions(
  const Rtree & ego_rtree, const Rtree & predicted_object_rtree, double time_tolerance);

bool checkFinite(const TrajectoryPoint & point);

void shiftPose(geometry_msgs::msg::Pose & pose, double longitudinal);

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_
