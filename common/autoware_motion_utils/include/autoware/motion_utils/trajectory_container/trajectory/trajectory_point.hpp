// Copyright 2024 Tier IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POINT_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POINT_HPP_

#include "autoware/motion_utils/trajectory_container/interpolator/interpolator.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory.hpp"

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{

/**
 * @brief Trajectory class for geometry_msgs::msg::Point
 */
template <>
class TrajectoryContainer<geometry_msgs::msg::Point>
: public detail::CropTrajectoryImpl<geometry_msgs::msg::Point>
{
  friend class CropTrajectoryImpl<geometry_msgs::msg::Point>;

protected:
  Eigen::VectorXd axis_;
  double start_, end_;
  std::shared_ptr<interpolator::Interpolator<double>> x_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> y_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> z_interpolator_;

  using ConstraintFunction = std::function<bool(const double & s)>;

public:
  TrajectoryContainer();

  TrajectoryContainer(const TrajectoryContainer & other) = default;

  /**
   * @brief Build trajectory from points
   * @param points Vector of points
   * @return Reference to this object
   */
  TrajectoryContainer & build(const std::vector<geometry_msgs::msg::Point> & points);

  /**
   * @brief Set interpolator for x and y coordinates
   * @param interpolator Interpolator object
   * @return Reference to this object
   */
  TrajectoryContainer & set_xy_interpolator(
    const interpolator::Interpolator<double> & interpolator);

  /**
   * @brief Set interpolator for z coordinate
   * @param interpolator Interpolator object
   * @return Reference to this object
   */
  TrajectoryContainer & set_z_interpolator(const interpolator::Interpolator<double> & interpolator);

  /**
   * @brief Get the length of the trajectory
   * @return Length of the trajectory
   */
  double length() const;

  /**
   * @brief Compute the point on the trajectory at a given s value
   * @param s Arc length
   * @return Point on the trajectory
   */
  geometry_msgs::msg::Point compute(const double & s) const;

  /**
   * @brief Get the direction at a given s value
   * @param s Arc length
   * @return Direction in radians
   */
  double direction(const double & s) const;

  /**
   * @brief Get the curvature at a given s value
   * @param s Arc length
   * @return Curvature
   */
  double curvature(const double & s) const;

  /**
   * @brief Find the nearest point with constraint
   * @tparam InputPointType Type of input point
   * @param p Input point
   * @param constraints Constraint function
   * @return Optional arc length of the nearest point
   */
  template <typename InputPointType>
  std::optional<double> nearest_with_constraint(
    const InputPointType & p, const ConstraintFunction & constraints) const;

  /**
   * @brief Find the nearest point
   * @tparam InputPointType Type of input point
   * @param p Input point
   * @return Arc length of the nearest point
   */
  template <typename InputPointType>
  double nearest(const InputPointType & p) const;
  /**
   * @brief Find the crossing point
   * @tparam InputPointType Type of input point
   * @param start Start point
   * @param end End point
   * @return Optional arc length of the crossing point
   */
  template <typename InputPointType>
  std::optional<double> crossed(const InputPointType & start, const InputPointType & end) const;

  /**
   * @brief Restore the trajectory points
   * @return Vector of points
   */
  std::vector<geometry_msgs::msg::Point> restore() const;
};

}  // namespace autoware::motion_utils::trajectory_container::trajectory

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_POINT_HPP_
