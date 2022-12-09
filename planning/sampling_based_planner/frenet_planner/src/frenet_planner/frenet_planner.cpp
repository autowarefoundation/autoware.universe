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

#include <frenet_planner/conversions.hpp>
#include <frenet_planner/polynomials.hpp>
#include <frenet_planner/structures.hpp>
#include <helper_functions/angle_utils.hpp>
#include <sampler_common/structures.hpp>
#include <sampler_common/transform/spline_transform.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace frenet_planner
{
std::vector<Trajectory> generateTrajectories(
  const sampler_common::transform::Spline2D & reference_spline, const FrenetState & initial_state,
  const SamplingParameters & sampling_parameters)
{
  std::vector<Trajectory> trajectories;
  trajectories.reserve(sampling_parameters.parameters.size());
  for (const auto & parameter : sampling_parameters.parameters) {
    auto trajectory = generateCandidate(
      initial_state, parameter.target_state, parameter.target_duration,
      sampling_parameters.resolution);
    calculateCartesian(reference_spline, trajectory);
    trajectories.push_back(trajectory);
  }
  return trajectories;
}

std::vector<Trajectory> generateLowVelocityTrajectories(
  const sampler_common::transform::Spline2D & reference_spline, const FrenetState & initial_state,
  const SamplingParameters & sampling_parameters)
{
  std::vector<Trajectory> trajectories;
  trajectories.reserve(sampling_parameters.parameters.size());
  for (const auto & parameter : sampling_parameters.parameters) {
    auto trajectory = generateLowVelocityCandidate(
      initial_state, parameter.target_state, parameter.target_duration,
      sampling_parameters.resolution);
    calculateCartesian(reference_spline, trajectory);
    trajectories.push_back(trajectory);
  }
  return trajectories;
}

std::vector<Path> generatePaths(
  const sampler_common::transform::Spline2D & reference_spline, const FrenetState & initial_state,
  const SamplingParameters & sampling_parameters)
{
  std::vector<Path> candidates;
  candidates.reserve(sampling_parameters.parameters.size());
  for (const auto parameter : sampling_parameters.parameters) {
    auto candidate =
      generateCandidate(initial_state, parameter.target_state, sampling_parameters.resolution);
    calculateCartesian(reference_spline, candidate);
    candidates.push_back(candidate);
  }
  return candidates;
}

Trajectory generateCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double duration,
  const double time_resolution)
{
  Trajectory trajectory;
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

Trajectory generateLowVelocityCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double duration,
  const double time_resolution)
{
  Trajectory trajectory;
  trajectory.longitudinal_polynomial = Polynomial(
    initial_state.position.s, initial_state.longitudinal_velocity,
    initial_state.longitudinal_acceleration, target_state.position.s,
    target_state.longitudinal_velocity, target_state.longitudinal_acceleration, duration);
  const auto delta_s = target_state.position.s - initial_state.position.s;
  trajectory.lateral_polynomial = Polynomial(
    initial_state.position.d, initial_state.lateral_velocity, initial_state.lateral_acceleration,
    target_state.position.d, target_state.lateral_velocity, target_state.lateral_acceleration,
    delta_s);
  for (double t = 0.0; t <= duration; t += time_resolution) {
    trajectory.times.push_back(t);
    const auto s = trajectory.longitudinal_polynomial->position(t);
    const auto ds = s - initial_state.position.s;
    const auto d = trajectory.lateral_polynomial->position(ds);
    trajectory.frenet_points.emplace_back(s, d);
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
  for (double s = s_resolution; s <= delta_s; s += s_resolution) {
    path.frenet_points.emplace_back(
      initial_state.position.s + s, path.lateral_polynomial->position(s));
  }
  return path;
}

void calculateCartesian(const sampler_common::transform::Spline2D & reference, Path & path)
{
  if (!path.frenet_points.empty()) {
    path.points.reserve(path.frenet_points.size());
    path.yaws.reserve(path.frenet_points.size());
    path.lengths.reserve(path.frenet_points.size());
    path.curvatures.reserve(path.frenet_points.size());
    // Calculate cartesian positions
    for (const auto & fp : path.frenet_points) {
      path.points.push_back(reference.cartesian(fp));
    }
    // TODO(Maxime CLEMENT): more precise calculations are proposed in Appendix I of the paper:
    // Optimal path Generation for Dynamic Street Scenarios in a Frenet Frame (Werling2010)
    // Calculate cartesian yaw and interval values
    path.lengths.push_back(0.0);
    for (auto it = path.points.begin(); it != std::prev(path.points.end()); ++it) {
      const auto dx = std::next(it)->x() - it->x();
      const auto dy = std::next(it)->y() - it->y();
      path.yaws.push_back(std::atan2(dy, dx));
      path.lengths.push_back(path.lengths.back() + std::hypot(dx, dy));
    }
    path.yaws.push_back(path.yaws.back());
    // Calculate curvatures
    for (size_t i = 1; i < path.yaws.size(); ++i) {
      const auto dyaw =
        autoware::common::helper_functions::wrap_angle(path.yaws[i] - path.yaws[i - 1]);
      path.curvatures.push_back(dyaw / (path.lengths[i - 1], path.lengths[i]));
    }
    path.curvatures.push_back(path.curvatures.back());
  }
}
void calculateCartesian(
  const sampler_common::transform::Spline2D & reference, Trajectory & trajectory)
{
  if (!trajectory.frenet_points.empty()) {
    trajectory.points.reserve(trajectory.frenet_points.size());
    // Calculate cartesian positions
    for (const auto & fp : trajectory.frenet_points)
      trajectory.points.push_back(reference.cartesian(fp));
    calculateLengthsAndYaws(trajectory);
    std::vector<double> dyaws;
    dyaws.reserve(trajectory.yaws.size());
    for (size_t i = 0; i + 1 < trajectory.yaws.size(); ++i)
      dyaws.push_back(autoware::common::helper_functions::wrap_angle(
        trajectory.yaws[i + 1] - trajectory.yaws[i]));
    dyaws.push_back(0.0);
    // Calculate curvatures
    for (size_t i = 1; i < trajectory.yaws.size(); ++i) {
      const auto curvature = trajectory.lengths[i] == trajectory.lengths[i - 1]
                               ? 0.0
                               : dyaws[i] / (trajectory.lengths[i] - trajectory.lengths[i - 1]);
      trajectory.curvatures.push_back(curvature);
    }
    const auto last_curvature = trajectory.curvatures.empty() ? 0.0 : trajectory.curvatures.back();
    trajectory.curvatures.push_back(last_curvature);
    // Calculate velocities, accelerations, jerk
    for (size_t i = 0; i < trajectory.times.size(); ++i) {
      const auto time = trajectory.times[i];
      const auto s = trajectory.frenet_points[i].s;
      const auto d = trajectory.frenet_points[i].d;
      const auto curvature = reference.curvature(s);
      const auto curvd = (1 - curvature * d);
      const auto s_vel = trajectory.longitudinal_polynomial->velocity(time);
      const auto s_acc = trajectory.longitudinal_polynomial->acceleration(time);
      const auto d_vel = trajectory.lateral_polynomial->velocity(time);
      // const auto d_acc = trajectory.lateral_polynomial->acceleration(time);
      const auto cos_dyaw = std::cos(dyaws[i]);
      const auto tan_dyaw = std::tan(dyaws[i]);
      trajectory.longitudinal_velocities.push_back(
        std::sqrt(curvd * curvd * s_vel * s_vel + d_vel * d_vel));
      trajectory.lateral_velocities.push_back(curvd * std::tan(dyaws[i]));
      if (i > 0lu) {
        const auto ds = s - trajectory.frenet_points[i - 1].s;
        const auto ddyaw = (dyaws[i] - dyaws[i - 1]) / ds;
        const auto dcurv =
          (curvature - reference.curvature(trajectory.frenet_points[i - 1].s)) / ds;
        const auto dcurvd_curvdd = dcurv * d + curvature * trajectory.lateral_velocities[i];
        trajectory.longitudinal_accelerations.push_back(
          s_acc * curvd / cos_dyaw +
          s_vel * s_vel / cos_dyaw * (curvd * tan_dyaw * ddyaw - dcurvd_curvdd));
        // TODO(Maxime): the 1st 'curvature' variable should be the curvature of the trajectory, not
        // the one of the reference path
        trajectory.lateral_accelerations.push_back(
          -dcurvd_curvdd * tan_dyaw +
          curvd / (cos_dyaw * cos_dyaw) * (curvature * (curvd / cos_dyaw) - curvature));
      }
      trajectory.jerks.push_back(
        trajectory.longitudinal_polynomial->jerk(time) + trajectory.lateral_polynomial->jerk(time));
    }
    if (trajectory.longitudinal_accelerations.empty()) {
      trajectory.longitudinal_accelerations.push_back(0.0);
      trajectory.lateral_accelerations.push_back(0.0);
    } else {
      trajectory.longitudinal_accelerations.push_back(trajectory.longitudinal_accelerations.back());
      trajectory.lateral_accelerations.push_back(trajectory.lateral_accelerations.back());
    }
  }
}

}  // namespace frenet_planner
