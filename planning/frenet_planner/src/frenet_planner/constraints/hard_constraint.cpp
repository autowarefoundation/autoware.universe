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

#include "frenet_planner/constraints/hard_constraint.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/distance/interface.hpp>
#include <boost/geometry/core/cs.hpp>

#include <algorithm>

namespace frenet_planner::constraints
{
bool satisfyMinMax(const std::vector<double> & values, const double min, const double max)
{
  for (const auto value : values) {
    if (value < min || value > max) return false;
  }
  return true;
}

void checkFrenetHardConstraints(
  std::vector<Trajectory> & trajectories, const Constraints & constraints, Debug & debug)
{
  for (auto & trajectory : trajectories) {
    if (
      std::find_if(
        trajectory.frenet_points.begin(), trajectory.frenet_points.end(), [&](const auto & fp) {
          return fp.d < constraints.hard.min_lateral_deviation ||
                 fp.d > constraints.hard.max_lateral_deviation;
        }) != trajectory.frenet_points.end()) {
      trajectory.valid = false;
      ++debug.nb_constraint_violations.lateral_deviation;
    }
    if (!satisfyMinMax(
          trajectory.longitudinal_velocities, constraints.hard.min_velocity,
          constraints.hard.max_velocity)) {
      trajectory.valid = false;
      ++debug.nb_constraint_violations.velocity;
    }
  }
}

bool collideWithPolygons(const Trajectory & trajectory, const std::vector<Polygon> & polygons)
{
  for (const auto & polygon : polygons) {
    for (const auto & point : trajectory.points) {
      if (boost::geometry::distance(point, polygon) < 2.0) {
        return true;
      }
    }
  }
  return false;
}

void checkCartesianHardConstraints(
  std::vector<Trajectory> & trajectories, const Constraints & constraints, Debug & debug)
{
  for (auto & trajectory : trajectories) {
    if (collideWithPolygons(trajectory, constraints.obstacle_polygons)) {
      ++debug.nb_constraint_violations.collision;
      trajectory.valid = false;
    }
    if (!satisfyMinMax(
          trajectory.curvatures, constraints.hard.min_curvature, constraints.hard.max_curvature)) {
      ++debug.nb_constraint_violations.curvature;
      trajectory.valid = false;
    }
  }
}

}  // namespace frenet_planner::constraints
