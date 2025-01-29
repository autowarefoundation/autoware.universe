// Copyright 2024 TIER IV, Inc.
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

#include "autoware/trajectory/utils/crossed.hpp"

#include <optional>
#include <vector>

namespace autoware::trajectory::detail::impl
{

std::optional<double> crossed_with_constraint_impl(
  const std::function<Eigen::Vector2d(const double & s)> & trajectory_compute,
  const std::vector<double> & bases,  //
  const Eigen::Vector2d & line_start, const Eigen::Vector2d & line_end,
  const std::function<bool(const double &)> & constraint)
{
  Eigen::Vector2d line_dir = line_end - line_start;

  for (size_t i = 1; i < bases.size(); ++i) {
    const Eigen::Vector2d p0 = trajectory_compute(bases.at(i - 1));
    const Eigen::Vector2d p1 = trajectory_compute(bases.at(i));

    Eigen::Vector2d segment_dir = p1 - p0;

    const double det = segment_dir.x() * line_dir.y() - segment_dir.y() * line_dir.x();

    if (std::abs(det) < 1e-10) {
      continue;
    }

    Eigen::Vector2d p0_to_line_start = line_start - p0;

    const double t =
      (p0_to_line_start.x() * line_dir.y() - p0_to_line_start.y() * line_dir.x()) / det;
    const double u =
      (p0_to_line_start.x() * segment_dir.y() - p0_to_line_start.y() * segment_dir.x()) / det;

    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
      double intersection = bases.at(i - 1) + t * (bases.at(i) - bases.at(i - 1));
      if (constraint(intersection)) {
        return intersection;
      }
    }
  }

  return std::nullopt;
}

std::vector<double> crossed_with_constraint_impl(
  const std::function<Eigen::Vector2d(const double & s)> & trajectory_compute,
  const std::vector<double> & bases,  //
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> & linestring,
  const std::function<bool(const double &)> & constraint)
{
  using trajectory::detail::to_point;

  std::vector<double> intersections;

  for (const auto & line : linestring) {
    const Eigen::Vector2d & line_start = line.first;
    const Eigen::Vector2d & line_end = line.second;

    std::optional<double> intersection =
      crossed_with_constraint_impl(trajectory_compute, bases, line_start, line_end, constraint);

    if (intersection) {
      intersections.push_back(*intersection);
    }
  }

  return intersections;
}

}  // namespace autoware::trajectory::detail::impl
