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

#include "frenet_planner/constraints/soft_constraint.hpp"

#include "frenet_planner/transform/spline_transform.hpp"

#include <numeric>

namespace frenet_planner::constraints
{
void calculateCost(std::vector<Trajectory> & trajectories, const Constraints & constraints)
{
  for (auto & trajectory : trajectories) {
    trajectory.cost = 0.0f;
    // lateral deviation
    double lateral_deviation_sum = 0.0;
    for (const auto & fp : trajectory.frenet_points) {
      lateral_deviation_sum += std::abs(fp.d);
    }
    trajectory.cost += constraints.soft.lateral_deviation_weight * lateral_deviation_sum /
                       static_cast<double>(trajectory.frenet_points.size());
    // curvature
    double curvature_sum = 0.0;
    for (const auto curvature : trajectory.curvatures) {
      curvature_sum += std::abs(curvature);
    }
    trajectory.cost += constraints.soft.curvature_weight * curvature_sum /
                       static_cast<double>(trajectory.curvatures.size());
  }
}
}  // namespace frenet_planner::constraints
