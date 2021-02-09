/*
 * Copyright 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FRENET_PLANNER__TRAJECTORY_REUSE_HPP
#define FRENET_PLANNER__TRAJECTORY_REUSE_HPP

#include "frenet_planner/structures.hpp"

namespace frenet_planner
{
/// @brief try to reuse part of a trajectory
/// @param [in] traj_to_reuse trajectory to check
/// @param [in] current_pose current pose of ego
/// @param [in] max_reuse_duration maximum duration of the trajectory to reuse [s]
/// @param [in] max_reuse_length maximum length of the trajectory to reuse [m]
/// @param [in] max_deviation maximum deviation from the trajectory [m]
/// @param [in] constraints constraints to check if the trajectory is still feasible
/// @param [out] reusable_traj reusable trajectory
/// @return true if the trajectory can be reused, else false
bool tryToReuseTrajectory(
  const Trajectory & traj_to_reuse, const Point & current_pose, const double max_reuse_duration,
  const double max_reuse_length, const double max_deviation, const Constraints & constraints,
  Trajectory & reusable_traj);
}  // namespace frenet_planner

#endif  // FRENET_PLANNER__TRAJECTORY_REUSE_HPP
