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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__NEAREST_NEIGHBOR_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__NEAREST_NEIGHBOR_HPP_

#include "autoware/motion_utils/trajectory_container/interpolator/interpolator.hpp"

#include <Eigen/Dense>

#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator
{

/**
 * @brief Template class for nearest neighbor interpolation.
 *
 * This class provides methods to perform nearest neighbor interpolation on a set of data points.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class NearestNeighbor;

namespace detail
{

/**
 * @brief Base class for nearest neighbor interpolation.
 *
 * This class implements the core functionality for nearest neighbor interpolation.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class NearestNeighbor_ : public InterpolatorCRTP<NearestNeighbor<T>, T>
{
protected:
  Eigen::VectorXd axis;   ///< Axis values for the interpolation.
  std::vector<T> values;  ///< Interpolation values.

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  T compute_(const double & s) const override;

  /**
   * @brief Build the interpolator with the given axis and values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   */
  void build_(
    const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values) override;

public:
  /**
   * @brief Default constructor.
   */
  NearestNeighbor_() = default;

  /**
   * @brief Destructor.
   */
  virtual ~NearestNeighbor_() = default;

  /**
   * @brief Get the minimum number of required points for the interpolator.
   *
   * @return The minimum number of required points.
   */
  size_t minimum_required_points() const override { return 1; }
};

}  // namespace detail

/**
 * @brief Template class for nearest neighbor interpolation.
 *
 * This class provides the interface for nearest neighbor interpolation.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class NearestNeighbor : public detail::NearestNeighbor_<T>
{
};

/**
 * @brief Specialization of NearestNeighbor for double values.
 *
 * This class provides methods to perform nearest neighbor interpolation on double values.
 */
template <>
class NearestNeighbor<double> : public detail::NearestNeighbor_<double>
{
private:
  /**
   * @brief Compute the first derivative at the given point.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  double compute_first_derivative_(const double &) const override { return 0.0; }

  /**
   * @brief Compute the second derivative at the given point.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  double compute_second_derivative_(const double &) const override { return 0.0; }
};

}  // namespace autoware::motion_utils::trajectory_container::interpolator

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__NEAREST_NEIGHBOR_HPP_
