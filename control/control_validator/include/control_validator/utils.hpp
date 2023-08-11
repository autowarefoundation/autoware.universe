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

#ifndef CONTROL_VALIDATOR__UTILS_HPP_
#define CONTROL_VALIDATOR__UTILS_HPP_

#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <utility>
#include <vector>

namespace control_validator::utils
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using motion_utils::convertToTrajectoryPointArray;
// using motion_utils::
using TrajectoryPoints = std::vector<TrajectoryPoint>;

void insertPointInPredictedTrajectory(
  TrajectoryPoints & modified_trajectory, const geometry_msgs::msg::Pose & reference_pose,
  const TrajectoryPoints & predicted_trajectory);

Trajectory alignTrajectoryWithReferenceTrajectory(
  const Trajectory & trajectory, const Trajectory & predicted_trajectory);

TrajectoryPoints reverseTrajectoryPoints(const TrajectoryPoints & trajectory);

bool removeFrontTrajectoryPoint(
  const TrajectoryPoints & trajectory_points, TrajectoryPoints & modified_trajectory_points,
  const TrajectoryPoints & predicted_trajectory_points);

double calcMaxLateralDistance(
  const Trajectory & reference_trajectory, const Trajectory & predicted_trajectory);

void shiftPose(Pose & pose, double longitudinal);
}  // namespace control_validator::utils

#endif  // CONTROL_VALIDATOR__UTILS_HPP_
