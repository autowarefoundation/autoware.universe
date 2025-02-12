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

#ifndef AUTOWARE__TRAJECTORY__UTILS__CROSSED_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__CROSSED_HPP_
#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <Eigen/Core>

#include <utility>
#include <vector>

namespace autoware::trajectory
{
namespace detail::impl
{

/**
 * @brief Internal implementation to find intersections between a trajectory and a linestring with
 * constraints.
 * @param trajectory_compute A function that computes a 2D point on the trajectory for a given
 * parameter `s`.
 * @param bases A vector of double values representing the sequence of bases for the trajectory.
 * @param linestring A vector of pairs representing the linestring as a sequence of 2D line
 * segments.
 * @param constraint A function that evaluates whether a given parameter `s` satisfies the
 * constraint.
 * @return A vector of double values representing the parameters `s` where the trajectory intersects
 * the linestring and satisfies the constraint.
 */
std::vector<double> crossed_with_constraint_impl(
  const std::function<Eigen::Vector2d(const double & s)> & trajectory_compute,
  const std::vector<double> & bases,  //
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> & linestring,
  const std::function<bool(const double &)> & constraint);
}  // namespace detail::impl

/**
 * @brief Finds intersections between a trajectory and a linestring where the given constraint is
 * satisfied.
 * @tparam TrajectoryPointType The type of points in the trajectory.
 * @tparam LineStringType The type of the linestring.
 * @tparam Constraint A callable type that evaluates a constraint on a trajectory point.
 * @param trajectory The trajectory to evaluate.
 * @param linestring The linestring to intersect with the trajectory.
 * @param constraint The constraint to apply to each point in the trajectory.
 * @return A vector of double values representing the parameters `s` where the trajectory intersects
 * the linestring and satisfies the constraint.
 */
template <class TrajectoryPointType, class LineStringType, class Constraint>
[[nodiscard]] std::vector<double> crossed_with_constraint(
  const trajectory::Trajectory<TrajectoryPointType> & trajectory, const LineStringType & linestring,
  const Constraint & constraint)
{
  using autoware::trajectory::detail::to_point;

  std::function<Eigen::Vector2d(const double & s)> trajectory_compute =
    [&trajectory](const double & s) {
      TrajectoryPointType point = trajectory.compute(s);
      Eigen::Vector2d result;
      result << to_point(point).x, to_point(point).y;
      return result;
    };

  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> linestring_eigen;

  if (linestring.end() - linestring.begin() < 2) {
    return {};
  }

  auto point_it = linestring.begin();
  auto point_it_next = linestring.begin() + 1;

  for (; point_it_next != linestring.end(); ++point_it, ++point_it_next) {
    Eigen::Vector2d start;
    Eigen::Vector2d end;
    start << point_it->x(), point_it->y();
    end << point_it_next->x(), point_it_next->y();
    linestring_eigen.emplace_back(start, end);
  }

  return detail::impl::crossed_with_constraint_impl(
    trajectory_compute, trajectory.get_internal_bases(), linestring_eigen,
    [&constraint, &trajectory](const double & s) { return constraint(trajectory.compute(s)); });
}

/**
 * @brief Finds intersections between a trajectory and a linestring.
 * @tparam TrajectoryPointType The type of points in the trajectory.
 * @tparam LineStringType The type of the linestring.
 * @param trajectory The trajectory to evaluate.
 * @param linestring The linestring to intersect with the trajectory.
 * @return A vector of double values representing the parameters `s` where the trajectory intersects
 * the linestring.
 */
template <class TrajectoryPointType, class LineStringType>
[[nodiscard]] std::vector<double> crossed(
  const trajectory::Trajectory<TrajectoryPointType> & trajectory, const LineStringType & linestring)
{
  return crossed_with_constraint(
    trajectory, linestring, [](const TrajectoryPointType &) { return true; });
}

}  // namespace autoware::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__CROSSED_HPP_
