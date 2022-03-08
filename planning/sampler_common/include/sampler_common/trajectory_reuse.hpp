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

#ifndef SAMPLER_COMMON__TRAJECTORY_REUSE_HPP
#define SAMPLER_COMMON__TRAJECTORY_REUSE_HPP

#include "sampler_common/structures.hpp"

namespace sampler_common
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

/// @brief try to reuse part of a path
/// @param [in] path_to_reuse trajectory to check
/// @param [in] current_state current state of ego
/// @param [in] max_reuse_length maximum length of the path to reuse [m]
/// @param [in] max_deviation maximum deviation from the path [m]
/// @param [in] constraints constraints to check if the path is still feasible
/// @param [out] reusable_path reusable path
/// @return true if the path can be reused, else false
bool tryToReusePath(
  const Path & path_to_reuse, const State & current_state, const double max_reuse_length,
  const double max_deviation, const Constraints & constraints, Path & reusable_path);
}  // namespace sampler_common

#endif  // SAMPLER_COMMON__TRAJECTORY_REUSE_HPP
