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

#ifndef MOTION_UTILS__TRAJECTORY_V2__DETAIL__MANIPULABLE_INTERPOLATED_ARRAY_HPP_
#define MOTION_UTILS__TRAJECTORY_V2__DETAIL__MANIPULABLE_INTERPOLATED_ARRAY_HPP_

#include "motion_utils/trajectory_v2/interpolator/interpolator.hpp"

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace motion_utils::trajectory_v2::trajectory
{
template <typename T>
class TrajectoryV2;
}  // namespace motion_utils::trajectory_v2::trajectory

namespace motion_utils::trajectory_v2::detail
{

/**
 * @brief Forward declaration of ManipulableInterpolatedArray class.
 */
template <typename T>
class ManipulableInterpolatedArray;

/**
 * @brief Class for setting values in a specific range of a ManipulableInterpolatedArray.
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
  RangeSetter & operator=(const T & value);

private:
  /**
   * @brief Construct a RangeSetter.
   * @param parent Reference to the ManipulableInterpolatedArray.
   * @param start Start of the range.
   * @param end End of the range.
   * @throws std::invalid_argument if start is not less than end.
   */
  RangeSetter(ManipulableInterpolatedArray<T> & parent, double start, double end);

  ManipulableInterpolatedArray<T> & parent_;
  double start_;
  double end_;
};

/**
 * @brief Class representing an array with interpolatable values that can be manipulated.
 */
template <typename T>
class ManipulableInterpolatedArray
{
  template <typename U>
  friend class motion_utils::trajectory_v2::trajectory::TrajectoryV2;

  friend class RangeSetter<T>;

private:
  bool is_built_ = false;
  std::vector<double> axis_;
  std::vector<T> values_;
  std::shared_ptr<interpolator::Interpolator<T>> interpolator_;

public:
  ManipulableInterpolatedArray() = default;

  /**
   * @brief Set the interpolator for the array.
   * @param interpolator The interpolator to be set.
   * @return Reference to the ManipulableInterpolatedArray object.
   */
  ManipulableInterpolatedArray & setInterpolator(
    const interpolator::Interpolator<T> & interpolator);

  /**
   * @brief Build the array with specified axis and values.
   * @param axis The axis values.
   * @param values The corresponding values.
   * @return Reference to the ManipulableInterpolatedArray object.
   * @throws std::invalid_argument if axis and values sizes do not match or are empty.
   */
  ManipulableInterpolatedArray & build(
    const std::vector<double> & axis, const std::vector<T> & values);

  /**
   * @brief Build the array with specified axis and values.
   * @param axis The axis values in Eigen::VectorXd format.
   * @param values The corresponding values.
   * @return Reference to the ManipulableInterpolatedArray object.
   */
  ManipulableInterpolatedArray & build(const Eigen::VectorXd & axis, const std::vector<T> & values);

  /**
   * @brief Get a RangeSetter object for the specified range.
   * @param start Start of the range.
   * @param end End of the range.
   * @return RangeSetter object.
   */
  RangeSetter<T> operator()(const double & start, const double & end);

  /**
   * @brief Assign a value to the entire array.
   * @param value Value to be assigned.
   * @return Reference to the ManipulableInterpolatedArray object.
   * @throws std::runtime_error if the values array is empty.
   */
  ManipulableInterpolatedArray & operator=(const T & value);

  /**
   * @brief Compute the interpolated value at a given position.
   * @param x The position to compute the value at.
   * @return The interpolated value.
   * @throws std::runtime_error if the interpolator is not set or the array has not been built.
   */
  T compute(const double & x) const;
};

}  // namespace motion_utils::trajectory_v2::detail

#endif  // MOTION_UTILS__TRAJECTORY_V2__DETAIL__MANIPULABLE_INTERPOLATED_ARRAY_HPP_
