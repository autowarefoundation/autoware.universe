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

#include "sampler_common/constraints/soft_constraint.hpp"

#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <numeric>

namespace sampler_common::constraints
{
void calculateCurvatureCost(Path & path, const Constraints & constraints)
{
  double curvature_sum = 0.0;
  for (const auto curvature : path.curvatures) {
    curvature_sum += std::abs(curvature);
  }
  path.cost +=
    constraints.soft.curvature_weight * curvature_sum / static_cast<double>(path.curvatures.size());
}
void calculateLengthCost(Path & path, const Constraints & constraints)
{
  if (!path.lengths.empty()) path.cost -= constraints.soft.length_weight * path.lengths.back();
}
void calculateYawRateCost(Trajectory & traj, const Constraints & constraints)
{
  double yaw_rate_sum = 0.0;
  for (auto i = 0lu; i < traj.curvatures.size(); ++i)
    yaw_rate_sum += std::abs(traj.curvatures[i] * traj.longitudinal_velocities[i]);
  traj.cost +=
    constraints.soft.yaw_rate_weight * yaw_rate_sum / static_cast<double>(traj.curvatures.size());
}

void calculateCost(
  Path & path, const Constraints & constraints, const transform::Spline2D & reference)
{
  if (path.points.empty()) return;
  calculateCurvatureCost(path, constraints);
  calculateLengthCost(path, constraints);
  // calculateLateralDeviationCost(path, constraints, reference);
  double lateral_deviation_sum = 0.0;
  const auto fp = reference.frenet(path.points.back());
  lateral_deviation_sum += std::abs(fp.d);
  path.cost += constraints.soft.lateral_deviation_weight * lateral_deviation_sum;
}

void calculateCost(
  Trajectory & trajectory, const Constraints & constraints, const transform::Spline2D & reference)
{
  Path & path = trajectory;
  calculateCost(path, constraints, reference);

  // calculateVelocityCost(trajectory, constraints);
  const auto avg_vel =
    std::accumulate(
      trajectory.longitudinal_velocities.begin(), trajectory.longitudinal_velocities.end(), 0.0) /
    trajectory.longitudinal_velocities.size();
  trajectory.cost += avg_vel * constraints.soft.velocity_weight;
  // lateral deviation
  auto deviation = 0.0;
  for (const auto & p : trajectory.points) deviation += std::abs(reference.frenet(p, 0.2).d);
  trajectory.cost +=
    (deviation / trajectory.points.size()) * constraints.soft.lateral_deviation_weight;
  // jerk
  auto jerk_sum = 0.0;
  for (const auto jerk : trajectory.jerks) jerk_sum += std::abs(jerk);
  const auto avg_jerk = jerk_sum / trajectory.jerks.size();
  trajectory.cost += avg_jerk * constraints.soft.jerk_weight;
}
}  // namespace sampler_common::constraints
