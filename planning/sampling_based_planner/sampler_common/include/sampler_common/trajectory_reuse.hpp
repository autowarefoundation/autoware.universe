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

#ifndef SAMPLER_COMMON__TRAJECTORY_REUSE_HPP_
#define SAMPLER_COMMON__TRAJECTORY_REUSE_HPP_

#include "sampler_common/structures.hpp"

#include <boost/geometry/algorithms/distance.hpp>

#include <algorithm>
#include <vector>

namespace sampler_common
{
struct ReusableTrajectory
{
  Trajectory trajectory;                 // base trajectory
  Configuration planning_configuration;  // planning configuration at the end of the trajectory
};

inline std::vector<ReusableTrajectory> calculateReusableTrajectories(
  const Trajectory & trajectory, const std::vector<double> & target_times)
{
  std::vector<ReusableTrajectory> reusable_trajectories;
  if (trajectory.points.empty() || target_times.empty()) return reusable_trajectories;
  reusable_trajectories.reserve(target_times.size());
  ReusableTrajectory reusable;
  for (const auto t : target_times) {
    auto to_idx = 0lu;
    while (to_idx + 1 < trajectory.times.size() && trajectory.times[to_idx] < t &&
           trajectory.longitudinal_velocities[to_idx] > 0.1)
      ++to_idx;
    if (to_idx == 0) continue;

    reusable.trajectory = *trajectory.subset(0, to_idx);
    reusable.planning_configuration.pose = reusable.trajectory.points.back();
    reusable.planning_configuration.velocity = reusable.trajectory.longitudinal_velocities.back();
    reusable.planning_configuration.acceleration =
      reusable.trajectory.longitudinal_accelerations.back();
    reusable.planning_configuration.heading = reusable.trajectory.yaws.back();
    reusable.planning_configuration.curvature = reusable.trajectory.curvatures.back();
    reusable_trajectories.push_back(reusable);
  }
  return reusable_trajectories;
}
}  // namespace sampler_common

#endif  // SAMPLER_COMMON__TRAJECTORY_REUSE_HPP_
