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

// clang-format off
#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__DETAIL__MANIPULABLE_INTERPOLATED_ARRAY_HPP_  // NOLINT
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__DETAIL__MANIPULABLE_INTERPOLATED_ARRAY_HPP_  // NOLINT
// clang-format on

#include "autoware/motion_utils/trajectory_container/interpolator/interpolator.hpp"

#include <Eigen/Core>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory::detail
{

template <typename T>
class ManipulableInterpolatedArray;

/**
 * @brief Class for setting values in a specific range of a ManipulableInterpolatedArray.
 * @tparam T The type of values stored in the array.
 */
template <typename T>
class RangeSetter
{
  friend class ManipulableInterpolatedArray<T>;

public:
  /**
   * @brief Assign a value to the specified range.
   * @param value Value to be assigned.
   * @return Reference to the RangeSetter object.
   */
  RangeSetter & operator=(const T & value)
  {
    Eigen::VectorXd & axis = parent_.axis_;
    std::vector<T> & values = parent_.values_;

    auto insert_if_not_present = [&](double val) -> typename std::vector<T>::iterator {
      auto axis_it = std::lower_bound(axis.begin(), axis.end(), val) - 1;
      auto values_it = values.begin() + (axis_it - axis.begin()) + 1;
      if (axis_it == (axis.end() - 1) || *axis_it != val) {
        int old_size = axis.size();
        int insert_pos = axis_it - axis.begin() + 1;

        // Resize and shift elements in axis
        axis.conservativeResize(old_size + 1);
        for (int i = old_size; i > insert_pos; --i) {
          axis(i) = axis(i - 1);
        }
        axis(insert_pos) = val;

        // Insert new value and return its iterator
        return values.insert(values_it, value);
      }
      return values_it;
    };
    // Insert start if not present
    auto start_it = insert_if_not_present(start_);

    // Insert end if not present
    auto end_it = insert_if_not_present(end_);

    // Set value in the specified range
    std::fill(start_it, end_it + 1, value);

    parent_.interpolator_->build(axis, values);

    return *this;
  }

private:
  /**
   * @brief Construct a RangeSetter.
   * @param parent Reference to the ManipulableInterpolatedArray.
   * @param start Start of the range.
   * @param end End of the range.
   */
  RangeSetter(ManipulableInterpolatedArray<T> & parent, double start, double end)
  : parent_(parent), start_(start), end_(end)
  {
  }

  ManipulableInterpolatedArray<T> & parent_;
  double start_;
  double end_;
};

/**
 * @brief Class representing an array with interpolatable values that can be manipulated.
 * @tparam T The type of values stored in the array.
 */
template <typename T>
class ManipulableInterpolatedArray
{
  friend class RangeSetter<T>;

  using InterpolatorType = interpolator::Interpolator<T>;

private:
  Eigen::VectorXd axis_;
  std::vector<T> values_;
  std::shared_ptr<interpolator::Interpolator<T>> interpolator_;

public:
  /**
   * @brief Construct a ManipulableInterpolatedArray with a given interpolator.
   * @param interpolator Shared pointer to the interpolator.
   */
  explicit ManipulableInterpolatedArray(const std::shared_ptr<InterpolatorType> & interpolator)
  : interpolator_(interpolator)
  {
  }

  /**
   * @brief Copy constructor.
   * @param other The ManipulableInterpolatedArray to copy from.
   */
  ManipulableInterpolatedArray(const ManipulableInterpolatedArray & other)
  : axis_(other.axis_), values_(other.values_), interpolator_(other.interpolator_->clone())
  {
  }

  ManipulableInterpolatedArray(ManipulableInterpolatedArray && other) = default;

  bool build(const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values)
  {
    axis_ = axis;
    values_ = values;
    return interpolator_->build(axis_, values_);
  }

  /**
   * @brief Move constructor.
   * @param other The ManipulableInterpolatedArray to move from.
   */
  ManipulableInterpolatedArray & operator=(ManipulableInterpolatedArray && other) = default;

  /**
   * @brief Copy assignment operator.
   * @param other The ManipulableInterpolatedArray to copy from.
   * @return Reference to this ManipulableInterpolatedArray.
   */
  ManipulableInterpolatedArray & operator=(const ManipulableInterpolatedArray & other)
  {
    axis_ = other.axis_;
    values_ = other.values_;
    interpolator_ = other.interpolator_->clone();
    return *this;
  }

  // Destructor
  ~ManipulableInterpolatedArray() = default;

  /**
   * @brief Get the start value of the axis.
   * @return The start value.
   */
  [[nodiscard]] double start() const { return axis_(0); }

  /**
   * @brief Get the end value of the axis.
   * @return The end value.
   */
  [[nodiscard]] double end() const { return axis_(axis_.size() - 1); }

  /**
   * @brief Get a RangeSetter object for the specified range.
   * @param start Start of the range.
   * @param end End of the range.
   * @return RangeSetter object.
   */
  RangeSetter<T> operator()(double start, double end)
  {
    if (start < this->start() || end > this->end()) {
      RCLCPP_WARN(
        rclcpp::get_logger("ManipulableInterpolatedArray"),
        "The range [%f, %f] is out of the array range [%f, %f]", start, end, this->start(),
        this->end());
      start = std::max(start, this->start());
      end = std::min(end, this->end());
    }
    return RangeSetter<T>{*this, start, end};
  }

  /**
   * @brief Assign a value to the entire array.
   * @param value Value to be assigned.
   * @return Reference to the ManipulableInterpolatedArray object.
   */
  ManipulableInterpolatedArray & operator=(const T & value)
  {
    std::fill(values_.begin(), values_.end(), value);
    interpolator_->build(axis_, values_);
    return *this;
  }

  /**
   * @brief Compute the interpolated value at a given position.
   * @param x The position to compute the value at.
   * @return The interpolated value.
   */
  T compute(const double & x) const { return interpolator_->compute(x); }

  /**
   * @brief Get the underlying data of the array.
   * @return A pair containing the axis and values.
   */
  std::pair<Eigen::VectorXd, std::vector<T>> get_data() const { return {axis_, values_}; }
};

}  // namespace autoware::motion_utils::trajectory_container::trajectory::detail

// clang-format off
#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__DETAIL__MANIPULABLE_INTERPOLATED_ARRAY_HPP_  // NOLINT
// clang-format on
