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

#include "sampler_common/trajectory_reuse.hpp"

#include "sampler_common/constraints/hard_constraint.hpp"
#include "sampler_common/structures.hpp"

#include <boost/geometry/algorithms/distance.hpp>

#include <iostream>
#include <limits>

namespace sampler_common
{
bool tryToReuseTrajectory(
  const Trajectory & traj_to_reuse, const Point & current_pose, const double max_reuse_duration,
  const double max_reuse_length, const double max_deviation, const Constraints & constraints,
  Trajectory & reusable_traj)
{
  if (traj_to_reuse.points.empty()) return false;

  reusable_traj.clear();
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < traj_to_reuse.points.size(); ++i) {
    const auto dist = boost::geometry::distance(current_pose, traj_to_reuse.points[i]);
    if (dist < min_dist) {
      closest_idx = i;
      min_dist = dist;
    }
  }

  // TODO(Maxime CLEMENT): make this a parameter
  constexpr auto min_reuse_velocity = 0.25;
  if (
    min_dist > max_deviation ||
    traj_to_reuse.longitudinal_velocities[closest_idx] < min_reuse_velocity) {
    return false;
  }

  size_t end_index = closest_idx;
  double duration = 0.0;
  for (size_t i = closest_idx; i < traj_to_reuse.points.size() - 2; ++i) {
    duration += traj_to_reuse.times[i];
    if (duration > max_reuse_duration || traj_to_reuse.lengths[i] > max_reuse_length) {
      end_index = i;
      break;
    }
  }
  size_t start_index = closest_idx;
  while (start_index > 0 &&
         traj_to_reuse.lengths[start_index] < 5.0) {  // TODO(Maxime CLEMENT): from param
    const auto & p1 = traj_to_reuse.points[start_index];
    --start_index;
    const auto & p2 = traj_to_reuse.points[start_index];
  }
  reusable_traj = *traj_to_reuse.subset(start_index, end_index);
  constraints::checkHardConstraints(reusable_traj, constraints);
  // TODO(Maxime CLEMENT): points become empty when all trajectories are invalid but this should be
  // detected earlier
  return reusable_traj.constraint_results.isValid() && !reusable_traj.points.empty();
}
}  // namespace sampler_common
