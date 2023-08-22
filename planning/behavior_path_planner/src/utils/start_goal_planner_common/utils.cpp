// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/utils/start_goal_planner_common/utils.hpp"

namespace behavior_path_planner::utils::start_goal_planner_common
{

using motion_utils::calcDecelDistWithJerkAndAccConstraints;

boost::optional<double> calcFeasibleDecelDistance(
  std::shared_ptr<const PlannerData> planner_data, const double acc_lim, const double jerk_lim,
  const double target_velocity)
{
  const auto v_now = planner_data->self_odometry->twist.twist.linear.x;
  const auto a_now = planner_data->self_acceleration->accel.accel.linear.x;

  if (v_now < target_velocity) {
    return 0.0;
  }

  auto min_stop_distance = calcDecelDistWithJerkAndAccConstraints(
    v_now, target_velocity, a_now, -acc_lim, jerk_lim, -1.0 * jerk_lim);

  if (!min_stop_distance) {
    return {};
  }

  min_stop_distance = std::max(*min_stop_distance, 0.0);

  return min_stop_distance;
}

}  // namespace behavior_path_planner::utils::start_goal_planner_common
