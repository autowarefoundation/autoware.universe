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

#ifndef MOTION_VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_
#define MOTION_VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_

#include <iostream>
#include <map>
#include <numeric>
#include <tuple>
#include <vector>

#include "boost/optional.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/trajectory/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace motion_velocity_smoother
{
namespace trajectory_utils
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;

TrajectoryPoint calcInterpolatedTrajectoryPoint(
  const Trajectory & trajectory, const Pose & target_pose);

boost::optional<Trajectory> extractPathAroundIndex(
  const Trajectory & trajectory, const size_t index,
  const double & ahead_length, const double & behind_length);

double calcArcLength(
  const Trajectory & trajectory, const int idx1, const int idx2);

std::vector<double> calcArclengthArray(const Trajectory & trajectory);

std::vector<double> calcTrajectoryIntervalDistance(
  const Trajectory & trajectory);

boost::optional<std::vector<double>> calcTrajectoryCurvatureFrom3Points(
  const Trajectory & trajectory, const size_t & idx_dist);

void setZeroVelocity(Trajectory & trajectory);

double getMaxVelocity(const Trajectory & trajectory);

double getMaxAbsVelocity(const Trajectory & trajectory);

void applyMaximumVelocityLimit(
  const size_t from, const size_t to, const double max_vel,
  Trajectory & trajectory);

boost::optional<size_t> searchZeroVelocityIdx(
  const Trajectory & trajectory);

boost::optional<Trajectory> applyLinearInterpolation(
  const std::vector<double> & base_index,
  const Trajectory & base_trajectory,
  const std::vector<double> & out_index, const bool use_spline_for_pose = false);

bool calcStopDistWithJerkConstraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, std::map<double, double> & jerk_profile,
  double & stop_dist);

bool isValidStopDist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);

boost::optional<Trajectory> applyDecelFilterWithJerkConstraint(
  const Trajectory & input, const size_t start_index, const double v0,
  const double a0, const double min_acc, const double decel_target_vel,
  const std::map<double, double> & jerk_profile);

boost::optional<std::tuple<double, double, double, double>> updateStateWithJerkConstraint(
  const double v0, const double a0, const std::map<double, double> & jerk_profile, const double t);

}  // namespace trajectory_utils
}  // namespace motion_velocity_smoother

#endif  // MOTION_VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_
