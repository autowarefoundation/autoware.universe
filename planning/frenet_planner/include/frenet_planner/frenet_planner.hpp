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

#ifndef FRENET_PLANNER__FRENET_PLANNER_HPP
#define FRENET_PLANNER__FRENET_PLANNER_HPP

#include "frenet_planner/structures.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <optional>
#include <vector>

namespace frenet_planner
{
/// @brief generate all trajectories
std::vector<Trajectory> generateTrajectories(
  const sampler_common::transform::Spline2D & reference_spline, const FrenetState & initial_state,
  const SamplingParameters & sampling_parameters, const sampler_common::Constraints & constraints,
  Debug & debug);
/// @brief generate all paths
std::vector<sampler_common::Path> generatePaths(
  const sampler_common::transform::Spline2D & reference_spline, const FrenetState & initial_state,
  const SamplingParameters & sampling_parameters);
/// @brief generate a candidate path
Path generateCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double s_resolution);
/// @brief generate a candidate trajectory
Trajectory generateCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double duration,
  const double time_resolution);
/// @brief generate candidate trajectories
std::vector<Trajectory> generateCandidates(
  const FrenetState & initial_state, const SamplingParameters & sp);
/// @brief calculate the cartesian frame of the given path
void calculateCartesian(const sampler_common::transform::Spline2D & reference, Path & path);
/// @brief calculate the cartesian frame of the given trajectory
void calculateCartesian(
  const sampler_common::transform::Spline2D & reference, Trajectory & trajectory);

}  // namespace frenet_planner

#endif  // FRENET_PLANNER__FRENET_PLANNER_HPP
