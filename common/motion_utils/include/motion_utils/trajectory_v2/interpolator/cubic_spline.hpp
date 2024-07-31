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

#ifndef MOTION_UTILS__TRAJECTORY_V2__INTERPOLATOR__CUBIC_SPLINE_HPP_
#define MOTION_UTILS__TRAJECTORY_V2__INTERPOLATOR__CUBIC_SPLINE_HPP_

#include "motion_utils/trajectory_v2/interpolator/interpolator.hpp"

#include <Eigen/Dense>

#include <vector>

namespace motion_utils::trajectory_v2::interpolator
{

/**
 * @brief Class for cubic spline interpolation.
 *
 * This class provides methods to perform cubic spline interpolation on a set of data points.
 */
class CubicSpline : public detail::InterpolatorCRTP<CubicSpline, double>
{
private:
  Eigen::VectorXd axis;        ///< Axis values for the interpolation.
  Eigen::VectorXd a, b, c, d;  ///< Coefficients for the cubic spline.
  Eigen::VectorXd h;           ///< Interval sizes between axis points.

  /**
   * @brief Compute the spline parameters.
   *
   * This method computes the coefficients for the cubic spline.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   */
  void compute_parameters(
    const Eigen::Ref<const Eigen::VectorXd> & axis,
    const Eigen::Ref<const Eigen::VectorXd> & values);

  /**
   * @brief Build the interpolator with the given axis and values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   */
  void build_(
    const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<double> & values) override;

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  double compute_(const double & s) const override;

  /**
   * @brief Compute the first derivative at the given point.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  double compute_first_derivative_(const double & s) const override;

  /**
   * @brief Compute the second derivative at the given point.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  double compute_second_derivative_(const double & s) const override;

public:
  /**
   * @brief Get the minimum number of required points for the interpolator.
   *
   * @return The minimum number of required points.
   */
  size_t minimum_required_points() const override { return 3; }
};

}  // namespace motion_utils::trajectory_v2::interpolator

#endif  // MOTION_UTILS__TRAJECTORY_V2__INTERPOLATOR__CUBIC_SPLINE_HPP_
