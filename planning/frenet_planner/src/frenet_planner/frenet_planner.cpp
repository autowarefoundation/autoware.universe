/*
 * Copyright 2021 Autoware Foundation. All rights reserved.
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

#include "frenet_planner/constraints/hard_constraint.hpp"
#include "frenet_planner/constraints/soft_constraint.hpp"
#include "frenet_planner/polynomials.hpp"
#include "frenet_planner/structures.hpp"
#include "frenet_planner/transform/spline_transform.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace frenet_planner
{
std::optional<Trajectory> generateTrajectory(
  const transform::Spline2D & reference_spline, const FrenetState & initial_state,
  const SamplingParameters & sampling_parameters, const Constraints & constraints)
{
  // Generate candidate trajectories
  // TODO(Maxime CLEMENT): dont use vector to improve erase performances
  Debug debug;
  auto trajectories =
    generateTrajectories(reference_spline, initial_state, sampling_parameters, constraints, debug);
  std::sort(trajectories.begin(), trajectories.end(), [](const auto & t1, const auto & t2) {
    return (t1.valid && !t2.valid) || (t1.cost < t2.cost);
  });

  std::optional<Trajectory> trajectory;
  if (trajectories.front().valid) trajectory = trajectories.front();
  return trajectory;
}

std::vector<Trajectory> generateTrajectories(
  const transform::Spline2D & reference_spline, const FrenetState & initial_state,
  const SamplingParameters & sampling_parameters, const Constraints & constraints, Debug & debug)
{
  // Generate candidate trajectories
  // TODO(Maxime CLEMENT): dont use vector to improve erase performances
  auto candidates = generateCandidates(initial_state, sampling_parameters);
  constraints::checkFrenetHardConstraints(candidates, constraints, debug);
  // Calculate cartesian part of trajectories
  calculateCartesian(reference_spline, candidates);
  // Check hard constraints (Cartesian)
  constraints::checkCartesianHardConstraints(candidates, constraints, debug);
  // Calculate objective function
  constraints::calculateCost(candidates, constraints);
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

void calculateCartesian(
  const transform::Spline2D & reference, std::vector<Trajectory> & trajectories)
{
  for (auto & trajectory : trajectories) {
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
      // Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame (Werling2010)
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
}

}  // namespace frenet_planner
