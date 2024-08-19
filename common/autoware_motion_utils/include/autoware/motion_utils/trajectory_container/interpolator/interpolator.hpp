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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__INTERPOLATOR_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__INTERPOLATOR_HPP_

#include <Eigen/Dense>
#include <rclcpp/logging.hpp>

#include <fmt/format.h>

#include <iterator>
#include <stdexcept>
#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator
{

namespace detail
{
/**
 * @brief Base class for interpolation implementations.
 *
 * This class provides the basic interface and common functionality for different types
 * of interpolation. It is intended to be subclassed by specific interpolation algorithms.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class InterpolatorCommonImpl
{
private:
  bool is_built_ = false;  ///< Flag indicating if the interpolator has been built.

protected:
  Eigen::VectorXd axis_;  ///< Axis values for the interpolation.

  /**
   * @brief Get the start of the interpolation range.
   */
  [[nodiscard]] double start() const { return axis_(0); }

  /**
   * @brief Get the end of the interpolation range.
   */
  [[nodiscard]] double end() const { return axis_(axis_.size() - 1); }

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * This method should be overridden by subclasses to provide the specific interpolation logic.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  [[nodiscard]] virtual T compute_impl(const double & s) const = 0;

  /**
   * @brief Build the interpolator with the given values.
   *
   * This method should be overridden by subclasses to provide the specific build logic.
   *
   * @param values The values to interpolate.
   */
  virtual void build_impl(const std::vector<T> & values) = 0;

  /**
   * @brief Validate the input to the build method.
   *
   * Checks that the axis and values have the same size and that there are enough points.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   * @throw std::runtime_error if the inputs are invalid.
   */
  void validate_build_input(
    const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values) const
  {
    if (static_cast<size_t>(axis.size()) != values.size()) {
      throw std::runtime_error(
        fmt::format(
          "Axis and values must have the same size. Axis size: {}, values size: {}.", axis.size(),
          values.size())
          .c_str());
    }
    if (static_cast<size_t>(axis.size()) < minimum_required_points()) {
      throw std::runtime_error(
        fmt::format(
          "Axis and values must have at least {} points. Axis size: {}, values size: {}.",
          minimum_required_points(), axis.size(), values.size())
          .c_str());
    }
  }

  /**
   * @brief Validate the input to the compute method.
   *
   * Checks that the interpolator has been built and that the input value is within range.
   *
   * @param s The input value.
   * @throw std::runtime_error if the interpolator has not been built.
   */
  void validate_compute_input(const double & s) const
  {
    if (!is_built_) {
      throw std::runtime_error("Interpolator has not been built yet.");
    }
    if (s < start() || s > end()) {
      RCLCPP_WARN(
        rclcpp::get_logger("InterpolatorBase"),
        "Input value %f is outside the range of the interpolator [%f, %f].", s, start(), end());
    }
  }

public:
  /**
   * @brief Build the interpolator with the given axis and values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   * @return Reference to the interpolator.
   */
  InterpolatorCommonImpl & build(
    const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values)
  {
    validate_build_input(axis, values);
    this->axis_ = axis;
    build_impl(values);
    this->is_built_ = true;
    return *this;
  };

  /**
   * @brief Build the interpolator with the given axis and values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   * @return Reference to the interpolator.
   */
  InterpolatorCommonImpl & build(const std::vector<double> & axis, const std::vector<T> & values)
  {
    Eigen::Map<const Eigen::VectorXd> axis_map(axis.data(), static_cast<Eigen::Index>(axis.size()));
    return build(axis_map, values);
  }

  /**
   * @brief Get the minimum number of required points for the interpolator.
   *
   * This method should be overridden by subclasses to return the specific requirement.
   *
   * @return The minimum number of required points.
   */
  [[nodiscard]] virtual size_t minimum_required_points() const = 0;

  [[nodiscard]] int get_index(const double & s, bool include_end = true) const
  {
    if (include_end && s == end()) {
      return static_cast<int>(axis_.size()) - 2;
    }
    auto comp = [](const double & a, const double & b) { return a <= b; };
    return std::distance(axis_.begin(), std::lower_bound(axis_.begin(), axis_.end(), s, comp)) - 1;
  }

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  [[nodiscard]] T compute(const double & s) const
  {
    validate_compute_input(s);
    return compute_impl(s);
  }
};

}  // namespace detail

/**
 * @brief Template class for interpolation.
 *
 * This class serves as the base class for specific interpolation types.
 *
 * @tparam T The type of the values being interpolated. (e.g. double, int, etc.)
 */
template <typename T>
class Interpolator : public detail::InterpolatorCommonImpl<T>
{
};

/**
 * @brief Specialization of Interpolator for double values.
 *
 * This class adds methods for computing first and second derivatives.
 */
template <>
class Interpolator<double> : public detail::InterpolatorCommonImpl<double>
{
protected:
  /**
   * @brief Compute the first derivative at the given point.
   *
   * This method should be overridden by subclasses to provide the specific logic.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  [[nodiscard]] virtual double compute_first_derivative_impl(const double & s) const = 0;

  /**
   * @brief Compute the second derivative at the given point.
   *
   * This method should be overridden by subclasses to provide the specific logic.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  [[nodiscard]] virtual double compute_second_derivative_impl(const double & s) const = 0;

public:
  Interpolator() = default;

  /**
   * @brief Compute the first derivative at the given point.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  [[nodiscard]] double compute_first_derivative(const double & s) const
  {
    this->validate_compute_input(s);
    return compute_first_derivative_impl(s);
  }

  /**
   * @brief Compute the second derivative at the given point.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  [[nodiscard]] double compute_second_derivative(const double & s) const
  {
    this->validate_compute_input(s);
    return compute_second_derivative_impl(s);
  }
};

}  // namespace autoware::motion_utils::trajectory_container::interpolator

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__INTERPOLATOR_HPP_
