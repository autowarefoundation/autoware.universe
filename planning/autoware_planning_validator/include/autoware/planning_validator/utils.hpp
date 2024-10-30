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

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware::universe_utils::Polygon2d;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

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

/**
 * @brief Validates trajectory for potential collisions with predicted objects
 *
 * This function checks if the planned trajectory will result in any collisions with
 * predicted objects in the environment. It performs the following steps:
 * 1. Resamples the trajectory for efficient checking
 * 2. Generates vehicle footprints along the trajectory
 * 3. Checks for intersections with predicted object paths
 *
 * @param predicted_objects List of predicted objects with their predicted paths.
 * @param trajectory Planned trajectory of the ego vehicle.
 * @param current_ego_point Current position of the ego vehicle.
 * @param vehicle_info Information about the ego vehicle (e.g., dimensions).
 * @param collision_check_distance_threshold Maximum distance to consider objects for collision
 * checking.
 * @return True if a potential collision is detected; false otherwise.
 */
bool checkCollision(
  const PredictedObjects & objects, const Trajectory & trajectory,
  const geometry_msgs::msg::Point & current_ego_point, const VehicleInfo & vehicle_info,
  const double collision_check_distance_threshold = 10.0);

Polygon2d createVehicleFootprintPolygon(
  const geometry_msgs::msg::Pose & pose, const VehicleInfo & vehicle_info);

bool checkFinite(const TrajectoryPoint & point);

void shiftPose(geometry_msgs::msg::Pose & pose, double longitudinal);

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_
