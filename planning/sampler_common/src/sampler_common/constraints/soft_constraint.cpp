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

#include "sampler_common/transform/spline_transform.hpp"

#include <numeric>

namespace sampler_common::constraints
{
void calculateCost(Path & path, const Constraints & constraints)
{
  path.cost = 0.0f;
  // lateral deviation
  /*
  double lateral_deviation_sum = 0.0;
  for (const auto & fp : path.frenet_points) {
    lateral_deviation_sum += std::abs(fp.d);
  }
  path.cost += constraints.soft.lateral_deviation_weight * lateral_deviation_sum /
                      static_cast<double>(path.frenet_points.size());
  */
  // curvature
  double curvature_sum = 0.0;
  for (const auto curvature : path.curvatures) {
    curvature_sum += std::abs(curvature);
  }
  path.cost +=
    constraints.soft.curvature_weight * curvature_sum / static_cast<double>(path.curvatures.size());
}
}  // namespace sampler_common::constraints
