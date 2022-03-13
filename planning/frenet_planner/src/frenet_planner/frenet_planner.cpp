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

#include "frenet_planner/frenet_planner.hpp"

#include <frenet_planner/polynomials.hpp>
#include <frenet_planner/structures.hpp>
#include <sampler_common/constraints/hard_constraint.hpp>
#include <sampler_common/constraints/soft_constraint.hpp>
#include <sampler_common/structures.hpp>
#include <sampler_common/transform/spline_transform.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace frenet_planner
{
std::vector<Trajectory> generateTrajectories(
  const sampler_common::transform::Spline2D & reference_spline, const FrenetState & initial_state,
  const SamplingParameters & sampling_parameters, const sampler_common::Constraints & constraints,
  Debug & debug)
{
  auto candidates = generateCandidates(initial_state, sampling_parameters);
  for (auto & candidate : candidates) {
    calculateCartesian(reference_spline, candidate);
    // Check hard constraints (Cartesian)
    const auto nb_of_violation =
      sampler_common::constraints::checkHardConstraints(candidate, constraints);
    debug.nb_constraint_violations.collision += nb_of_violation.collision;
    debug.nb_constraint_violations.curvature += nb_of_violation.curvature;
    // Calculate objective function
    sampler_common::constraints::calculateCost(candidate, constraints, reference_spline);
  }
  return candidates;
}

std::vector<Path> generatePaths(
  const sampler_common::transform::Spline2D & reference_spline, const FrenetState & initial_state,
  const SamplingParameters & sampling_parameters)
{
  std::vector<Path> candidates;
  FrenetState target_state;
  const auto & sp = sampling_parameters;
  for (const auto target_s : sp.target_longitudinal_positions) {
    target_state.position.s = target_s;
    for (const auto target_d : sp.target_lateral_positions) {
      target_state.position.d = target_d;
      for (const auto target_d_vel : sp.target_lateral_velocities) {
        target_state.lateral_velocity = target_d_vel;
        for (const auto target_d_acc : sp.target_lateral_accelerations) {
          target_state.lateral_acceleration = target_d_acc;
          auto candidate =
            generateCandidate(initial_state, target_state, sampling_parameters.time_resolution);
          calculateCartesian(reference_spline, candidate);
          candidates.push_back(candidate);
        }
      }
    }
  }
  return candidates;
}

std::vector<Trajectory> generateCandidates(
  const FrenetState & initial_state, const SamplingParameters & sp)
{
  std::vector<Trajectory> candidates;
  Trajectory trajectory;
  FrenetState target_state;
  for (const auto target_s : sp.target_longitudinal_positions) {
    target_state.position.s = target_s;
    for (const auto target_d : sp.target_lateral_positions) {
      target_state.position.d = target_d;
      for (const auto target_s_vel : sp.target_longitudinal_velocities) {
        target_state.longitudinal_velocity = target_s_vel;
        for (const auto target_d_vel : sp.target_lateral_velocities) {
          target_state.lateral_velocity = target_d_vel;
          for (const auto target_s_acc : sp.target_longitudinal_accelerations) {
            target_state.longitudinal_acceleration = target_s_acc;
            for (const auto target_d_acc : sp.target_lateral_accelerations) {
              target_state.lateral_acceleration = target_d_acc;
              for (const auto duration : sp.target_durations) {
                candidates.push_back(
                  generateCandidate(initial_state, target_state, duration, sp.time_resolution));
              }
            }
          }
        }
      }
    }
  }
  return candidates;
}

Trajectory generateCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double duration,
  const double time_resolution)
{
  Trajectory trajectory;
  trajectory.duration = duration;
  trajectory.longitudinal_polynomial = Polynomial(
    initial_state.position.s, initial_state.longitudinal_velocity,
    initial_state.longitudinal_acceleration, target_state.position.s,
    target_state.longitudinal_velocity, target_state.longitudinal_acceleration, duration);
  trajectory.lateral_polynomial = Polynomial(
    initial_state.position.d, initial_state.lateral_velocity, initial_state.lateral_acceleration,
    target_state.position.d, target_state.lateral_velocity, target_state.lateral_acceleration,
    duration);
  for (double t = 0.0; t <= duration; t += time_resolution) {
    trajectory.times.push_back(t);
    trajectory.frenet_points.emplace_back(
      trajectory.longitudinal_polynomial->position(t), trajectory.lateral_polynomial->position(t));
  }
  return trajectory;
}

Path generateCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double s_resolution)
{
  const auto delta_s = target_state.position.s - initial_state.position.s;
  Path path;
  path.lateral_polynomial = Polynomial(
    initial_state.position.d, initial_state.lateral_velocity, initial_state.lateral_acceleration,
    target_state.position.d, target_state.lateral_velocity, target_state.lateral_acceleration,
    delta_s);
  for (double s = 0; s <= delta_s; s += s_resolution) {
    path.frenet_points.emplace_back(
      initial_state.position.s + s, path.lateral_polynomial->position(s));
  }
  return path;
}

void calculateCartesian(const sampler_common::transform::Spline2D & reference, Path & path)
{
  if (path.valid && !path.frenet_points.empty()) {
    path.points.reserve(path.frenet_points.size());
    path.yaws.reserve(path.frenet_points.size() - 1);
    path.intervals.reserve(path.frenet_points.size() - 1);
    path.curvatures.reserve(path.frenet_points.size() - 1);
    // Calculate cartesian positions
    for (const auto & fp : path.frenet_points) {
      path.points.push_back(reference.cartesian(fp));
    }
    // TODO(Maxime CLEMENT): more precise calculations are proposed in Appendix I of the paper:
    // Optimal path Generation for Dynamic Street Scenarios in a Frenet Frame (Werling2010)
    // Calculate cartesian yaw and interval values
    for (auto it = path.points.begin(); it != std::prev(path.points.end()); ++it) {
      const auto dx = std::next(it)->x() - it->x();
      const auto dy = std::next(it)->y() - it->y();
      path.yaws.push_back(std::atan2(dy, dx));
      path.intervals.push_back(std::hypot(dx, dy));
    }
    // Calculate curvatures, velocities, accelerations
    for (size_t i = 1; i < path.yaws.size(); ++i) {
      const auto dyaw = path.yaws[i] - path.yaws[i - 1];
      path.curvatures.push_back(dyaw / path.intervals[i - 1]);
    }
  }
}
void calculateCartesian(
  const sampler_common::transform::Spline2D & reference, Trajectory & trajectory)
{
  if (trajectory.valid && !trajectory.frenet_points.empty()) {
    trajectory.points.reserve(trajectory.frenet_points.size());
    trajectory.yaws.reserve(trajectory.frenet_points.size() - 1);
    trajectory.intervals.reserve(trajectory.frenet_points.size() - 1);
    trajectory.curvatures.reserve(trajectory.frenet_points.size() - 1);
    // Calculate cartesian positions
    for (const auto & fp : trajectory.frenet_points) {
      trajectory.points.push_back(reference.cartesian(fp));
    }
    // TODO(Maxime CLEMENT): more precise calculations are proposed in Appendix I of the paper:
    // Optimal trajectory Generation for Dynamic Street Scenarios in a Frenet Frame (Werling2010)
    // Calculate cartesian yaw and interval values
    for (auto it = trajectory.points.begin(); it != std::prev(trajectory.points.end()); ++it) {
      const auto dx = std::next(it)->x() - it->x();
      const auto dy = std::next(it)->y() - it->y();
      trajectory.yaws.push_back(std::atan2(dy, dx));
      trajectory.intervals.push_back(std::hypot(dx, dy));
    }
    // Calculate curvatures, velocities, accelerations
    for (size_t i = 1; i < trajectory.yaws.size(); ++i) {
      const auto dyaw = trajectory.yaws[i] - trajectory.yaws[i - 1];
      trajectory.curvatures.push_back(dyaw / trajectory.intervals[i - 1]);
      trajectory.longitudinal_velocities.push_back(
        trajectory.longitudinal_polynomial->velocity(trajectory.times[i - 1]));
      trajectory.longitudinal_accelerations.push_back(
        trajectory.longitudinal_polynomial->acceleration(trajectory.times[i - 1]));
      trajectory.lateral_velocities.push_back(
        trajectory.lateral_polynomial->velocity(trajectory.times[i - 1]));
    }
  }
}

}  // namespace frenet_planner
