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

#ifndef FRENET_PLANNER__CONSTRAINTS__HARD_CONSTRAINT_HPP
#define FRENET_PLANNER__CONSTRAINTS__HARD_CONSTRAINT_HPP

#include "frenet_planner/structures.hpp"

#include <vector>

namespace frenet_planner::constraints
{
// @brief Check if the trajectories satisfy hard constraints defined in Frenet frame
void checkFrenetHardConstraints(
  std::vector<Trajectory> & trajectories, const Constraints & constraints, Debug & debug);

// @brief Check if the trajectories satisfy hard constraints defined in Cartesian frame
void checkCartesianHardConstraints(
  std::vector<Trajectory> & trajectories, const Constraints & constraints, Debug & debug);
}  // namespace frenet_planner::constraints

#endif  // FRENET_PLANNER__CONSTRAINTS__HARD_CONSTRAINT_HPP
