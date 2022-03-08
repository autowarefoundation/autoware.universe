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

#ifndef FRENET_PLANNER__STRUCTURES_HPP
#define FRENET_PLANNER__STRUCTURES_HPP

#include "frenet_planner/polynomials.hpp"

#include <sampler_common/structures.hpp>

#include <optional>
#include <vector>

namespace frenet_planner
{
struct Path : sampler_common::Path
{
  std::vector<sampler_common::FrenetPoint> frenet_points{};
  std::optional<Polynomial> lateral_polynomial{};

  Path() = default;
  explicit Path(const sampler_common::Path & path) : sampler_common::Path(path) {}

  void clear() override
  {
    sampler_common::Path::clear();
    frenet_points.clear();
    lateral_polynomial.reset();
  }

  void reserve(const size_t size) override
  {
    sampler_common::Path::reserve(size);
    frenet_points.reserve(size);
  }

  [[nodiscard]] Path extend(const Path & path) const
  {
    Path extended_traj(sampler_common::Path::extend(path));
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), frenet_points.begin(), frenet_points.end());
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), path.frenet_points.begin(), path.frenet_points.end());
    // TODO(Maxime CLEMENT): direct copy from the 2nd trajectory. might need to be improved
    extended_traj.lateral_polynomial = path.lateral_polynomial;
    return extended_traj;
  }

  [[nodiscard]] Path * subset(const size_t from_idx, const size_t to_idx) const override
  {
    auto * subpath = new Path(*sampler_common::Path::subset(from_idx, to_idx));
    assert(to_idx >= from_idx);
    subpath->reserve(to_idx - from_idx);

    const auto copy_subset = [&](const auto & from, auto & to) {
      to.insert(to.end(), std::next(from.begin(), from_idx), std::next(from.begin(), to_idx));
    };
    copy_subset(frenet_points, subpath->frenet_points);
    return subpath;
  };
};

struct Trajectory : sampler_common::Trajectory
{
  std::vector<sampler_common::FrenetPoint> frenet_points{};
  std::optional<Polynomial> lateral_polynomial{};
  std::optional<Polynomial> longitudinal_polynomial{};

  Trajectory() = default;
  explicit Trajectory(const sampler_common::Trajectory & traj) : sampler_common::Trajectory(traj) {}

  void clear() override
  {
    sampler_common::Trajectory::clear();
    frenet_points.clear();
    lateral_polynomial.reset();
    longitudinal_polynomial.reset();
  }

  void reserve(const size_t size) override
  {
    sampler_common::Trajectory::reserve(size);
    frenet_points.reserve(size);
  }

  [[nodiscard]] Trajectory extend(const Trajectory & traj) const
  {
    Trajectory extended_traj(sampler_common::Trajectory::extend(traj));
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), frenet_points.begin(), frenet_points.end());
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), traj.frenet_points.begin(), traj.frenet_points.end());
    // TODO(Maxime CLEMENT): direct copy from the 2nd trajectory. might need to be improved
    extended_traj.lateral_polynomial = traj.lateral_polynomial;
    extended_traj.longitudinal_polynomial = traj.longitudinal_polynomial;
    return extended_traj;
  }

  [[nodiscard]] Trajectory * subset(const size_t from_idx, const size_t to_idx) const override
  {
    auto * subtraj = new Trajectory(*sampler_common::Trajectory::subset(from_idx, to_idx));
    assert(to_idx >= from_idx);
    subtraj->reserve(to_idx - from_idx);

    const auto copy_subset = [&](const auto & from, auto & to) {
      to.insert(to.end(), std::next(from.begin(), from_idx), std::next(from.begin(), to_idx));
    };
    copy_subset(frenet_points, subtraj->frenet_points);
    return subtraj;
  };
};

struct FrenetState
{
  sampler_common::FrenetPoint position = {0, 0};
  double lateral_velocity{};
  double longitudinal_velocity{};
  double lateral_acceleration{};
  double longitudinal_acceleration{};
};

struct SamplingParameters
{
  std::vector<double> target_durations;
  std::vector<double> target_lateral_positions;
  std::vector<double> target_longitudinal_positions;
  std::vector<double> target_lateral_velocities;
  std::vector<double> target_longitudinal_velocities;
  std::vector<double> target_lateral_accelerations;
  std::vector<double> target_longitudinal_accelerations;
  double time_resolution;
};

struct Debug
{
  struct
  {
    size_t collision{};
    size_t curvature{};
    size_t velocity{};
    size_t lateral_deviation{};
  } nb_constraint_violations;
};
}  // namespace frenet_planner

#endif  // FRENET_PLANNER__STRUCTURES_HPP
