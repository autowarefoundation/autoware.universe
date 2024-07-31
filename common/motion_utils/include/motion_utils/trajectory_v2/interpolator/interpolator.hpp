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

#ifndef MOTION_UTILS__TRAJECTORY_V2__INTERPOLATOR__INTERPOLATOR_HPP_
#define MOTION_UTILS__TRAJECTORY_V2__INTERPOLATOR__INTERPOLATOR_HPP_

#include <Eigen/Dense>
#include <rclcpp/logging.hpp>

#include <fmt/format.h>

#include <stdexcept>
#include <vector>

namespace motion_utils::trajectory_v2::interpolator
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
class Interpolator_
{
private:
  bool is_built_ = false;  ///< Flag indicating if the interpolator has been built.
  double start_;           ///< Start of the interpolation range.
  double end_;             ///< End of the interpolation range.

protected:
  /**
   * @brief Get the start of the interpolation range.
   */
  double start() const { return start_; }

  /**
   * @brief Get the end of the interpolation range.
   */
  double end() const { return end_; }

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * This method should be overridden by subclasses to provide the specific interpolation logic.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  virtual T compute_(const double & s) const = 0;

  /**
   * @brief Build the interpolator with the given axis and values.
   *
   * This method should be overridden by subclasses to provide the specific build logic.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   */
  virtual void build_(
    const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values) = 0;

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
    if (s < start_ || s > end_) {
      RCLCPP_WARN(
        rclcpp::get_logger("InterpolatorBase"),
        fmt::format(
          "Input value {} is outside the range of the interpolator [{}, {}].", s, start_, end_)
          .c_str());
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
  Interpolator_ & build(
    const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values)
  {
    validate_build_input(axis, values);
    build_(axis, values);
    this->is_built_ = true;
    start_ = axis(0);
    end_ = axis(axis.size() - 1);
    return *this;
  };

  /**
   * @brief Build the interpolator with the given axis and values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   * @return Reference to the interpolator.
   */
  Interpolator_ & build(const std::vector<double> & axis, const std::vector<T> & values)
  {
    Eigen::Map<const Eigen::VectorXd> axis_map(axis.data(), axis.size());
    return build(axis_map, values);
  }

  /**
   * @brief Get the minimum number of required points for the interpolator.
   *
   * This method should be overridden by subclasses to return the specific requirement.
   *
   * @return The minimum number of required points.
   */
  virtual size_t minimum_required_points() const = 0;

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  T compute(const double & s) const
  {
    validate_compute_input(s);
    return compute_(s);
  }

  /**
   * @brief Create a clone of the interpolator.
   *
   * This method should be overridden by subclasses to return a copy of the interpolator.
   *
   * @return A pointer to the cloned interpolator.
   */
  virtual Interpolator_ * clone() const = 0;

  /**
   * @brief Destructor for the interpolator.
   */
  virtual ~Interpolator_() = default;
};

}  // namespace detail

/**
 * @brief Template class for interpolation.
 *
 * This class serves as the base class for specific interpolation types.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class Interpolator : public detail::Interpolator_<T>
{
public:
  Interpolator<T> * clone() const override = 0;
};

/**
 * @brief Specialization of Interpolator for double values.
 *
 * This class adds methods for computing first and second derivatives.
 */
template <>
class Interpolator<double> : public detail::Interpolator_<double>
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
  virtual double compute_first_derivative_(const double & s) const = 0;

  /**
   * @brief Compute the second derivative at the given point.
   *
   * This method should be overridden by subclasses to provide the specific logic.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  virtual double compute_second_derivative_(const double & s) const = 0;

public:
  Interpolator() = default;

  Interpolator<double> * clone() const override = 0;

  /**
   * @brief Compute the first derivative at the given point.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  double compute_first_derivative(const double & s) const
  {
    this->validate_compute_input(s);
    return compute_first_derivative_(s);
  }

  /**
   * @brief Compute the second derivative at the given point.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  double compute_second_derivative(const double & s) const
  {
    this->validate_compute_input(s);
    return compute_second_derivative_(s);
  }

  /**
   * @brief Destructor for the interpolator.
   */
  virtual ~Interpolator() = default;
};

namespace detail
{

/**
 * @brief CRTP base class for interpolation implementations.
 *
 * This class provides the common functionality for interpolation implementations using the CRTP
 * pattern.
 *
 * @tparam Derived The derived class.
 * @tparam T The type of the values being interpolated.
 */
template <typename Derived, typename T>
class InterpolatorCRTP : public Interpolator<T>
{
public:
  /**
   * @brief Create a clone of the interpolator.
   *
   * @return A pointer to the cloned interpolator.
   */
  Interpolator<T> * clone() const override
  {
    return new Derived(static_cast<const Derived &>(*this));
  }

  /**
   * @brief Build the interpolator with the given axis and values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   * @return Reference to the derived interpolator.
   */
  Derived & build(const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values)
  {
    return static_cast<Derived &>(Interpolator<T>::build(axis, values));
  };

  /**
   * @brief Build the interpolator with the given axis and values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   * @return Reference to the derived interpolator.
   */
  Derived & build(const std::vector<double> & axis, const std::vector<T> & values)
  {
    return static_cast<Derived &>(Interpolator<T>::build(axis, values));
  }

  /**
   * @brief Destructor for the interpolator.
   */
  virtual ~InterpolatorCRTP() = default;
};

}  // namespace detail

}  // namespace motion_utils::trajectory_v2::interpolator

#endif  // MOTION_UTILS__TRAJECTORY_V2__INTERPOLATOR__INTERPOLATOR_HPP_
