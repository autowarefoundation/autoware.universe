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
  // TODO(Maxime CLEMENT): use interpolation if we want better precision
  if (traj_to_reuse.points.empty()) {
    return false;
  }

  reusable_traj.clear();
  size_t start_index = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < traj_to_reuse.points.size(); ++i) {
    const auto dist = boost::geometry::distance(current_pose, traj_to_reuse.points[i]);
    if (dist < min_dist) {
      start_index = i;
      min_dist = dist;
    }
  }

  if (min_dist > max_deviation) {
    return false;
  }

  size_t end_index = start_index;
  double distance = 0.0;
  double duration = 0.0;
  for (size_t i = start_index; i < traj_to_reuse.points.size() - 2; ++i) {
    duration += traj_to_reuse.times[i];
    distance += traj_to_reuse.intervals[i];
    if (duration > max_reuse_duration || distance > max_reuse_length) {
      end_index = i;
      break;
    }
  }
  reusable_traj = *traj_to_reuse.subset(start_index, end_index);
  constraints::checkHardConstraints(reusable_traj, constraints);
  // TODO(Maxime CLEMENT): points become empty when all trajectories are invalid but this should be
  // detected earlier
  return reusable_traj.valid && !reusable_traj.points.empty();
}
}  // namespace sampler_common
