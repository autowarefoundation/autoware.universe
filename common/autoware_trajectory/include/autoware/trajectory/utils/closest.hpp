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

#ifndef AUTOWARE__TRAJECTORY__UTILS__CLOSEST_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__CLOSEST_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"

#include <Eigen/Core>

#include <functional>
#include <optional>
#include <vector>

namespace autoware::trajectory
{
namespace detail::impl
{
/**
 * @brief Internal implementation to find the closest point on a trajectory to a given point with
 * constraints.
 * @param trajectory_compute A function that computes a 2D point on the trajectory for a given
 * parameter `s`.
 * @param bases A vector of double values representing the sequence of bases for the trajectory.
 * @param point The 2D point to which the closest point on the trajectory is to be found.
 * @param constraint A function that evaluates whether a given parameter `s` satisfies the
 * constraint.
 * @return An optional double value representing the parameter `s` of the closest point on the
 * trajectory that satisfies the constraint, or `std::nullopt` if no such point exists.
 */
std::optional<double> closest_with_constraint_impl(
  const std::function<Eigen::Vector2d(const double & s)> & trajectory_compute,
  const std::vector<double> & bases, const Eigen::Vector2d & point,
  const std::function<bool(const double &)> & constraint);
}  // namespace detail::impl

/**
 * @brief Finds the closest point on a trajectory to a given point where the given constraint is
 * satisfied.
 * @tparam TrajectoryPointType The type of points in the trajectory.
 * @tparam ArgPointType The type of the input point.
 * @tparam Constraint A callable type that evaluates a constraint on a trajectory point.
 * @param trajectory The trajectory to evaluate.
 * @param point The point to which the closest point on the trajectory is to be found.
 * @param constraint The constraint to apply to each point in the trajectory.
 * @return An optional double value representing the parameter `s` of the closest point on the
 * trajectory that satisfies the constraint, or `std::nullopt` if no such point exists.
 */
template <class TrajectoryPointType, class ArgPointType, class Constraint>
std::optional<double> closest_with_constraint(
  const trajectory::Trajectory<TrajectoryPointType> & trajectory, const ArgPointType & point,
  const Constraint & constraint)
{
  using autoware::trajectory::detail::to_point;

  return detail::impl::closest_with_constraint_impl(
    [&trajectory](const double & s) {
      TrajectoryPointType point = trajectory.compute(s);
      Eigen::Vector2d result;
      result << to_point(point).x, to_point(point).y;
      return result;
    },
    trajectory.get_internal_bases(), {to_point(point).x, to_point(point).y},
    [&constraint, &trajectory](const double & s) { return constraint(trajectory.compute(s)); });
}

/**
 * @brief Finds the closest point on a trajectory to a given point.
 * @tparam TrajectoryPointType The type of points in the trajectory.
 * @tparam ArgPointType The type of the input point.
 * @param trajectory The trajectory to evaluate.
 * @param point The point to which the closest point on the trajectory is to be found.
 * @return The parameter `s` of the closest point on the trajectory.
 */
template <class TrajectoryPointType, class ArgPointType>
double closest(
  const trajectory::Trajectory<TrajectoryPointType> & trajectory, const ArgPointType & point)
{
  std::optional<double> s =
    *closest_with_constraint(trajectory, point, [](const TrajectoryPointType &) { return true; });
  if (!s) {
    throw std::runtime_error("No closest point found.");  // This Exception should not be thrown.
  }
  return *s;
}
}  // namespace autoware::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__CLOSEST_HPP_
