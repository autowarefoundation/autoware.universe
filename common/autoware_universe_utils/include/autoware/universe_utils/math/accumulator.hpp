// Copyright 2021-2024 Tier IV, Inc.
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

#include <iostream>
#include <limits>

#ifndef AUTOWARE__UNIVERSE_UTILS__MATH__ACCUMULATOR_HPP_
#define AUTOWARE__UNIVERSE_UTILS__MATH__ACCUMULATOR_HPP_

namespace autoware::universe_utils
{
/**
 * @brief class to accumulate statistical data, supporting min, max and mean.
 * @typedef T type of the values (default to double)
 */
template <typename T = double>
class Accumulator
{
public:
  /**
   * @brief add a value
   * @param value value to add
   */
  void add(const T & value)
  {
    if (value < min_) {
      min_ = value;
    }
    if (value > max_) {
      max_ = value;
    }
    ++count_;
    mean_ = mean_ + (value - mean_) / count_;
  }

  /**
   * @brief get the mean value
   */
  long double mean() const { return mean_; }

  /**
   * @brief get the minimum value
   */
  T min() const { return min_; }

  /**
   * @brief get the maximum value
   */
  T max() const { return max_; }

  /**
   * @brief get the number of values used to build this statistic
   */
  unsigned int count() const { return count_; }

  template <typename U>
  friend std::ostream & operator<<(std::ostream & os, const Accumulator<U> & accumulator);

private:
  T min_ = std::numeric_limits<T>::max();
  T max_ = std::numeric_limits<T>::lowest();
  long double mean_ = 0.0;
  unsigned int count_ = 0;
};

/**
 * @brief overload << operator for easy print to output stream
 */
template <typename T>
std::ostream & operator<<(std::ostream & os, const Accumulator<T> & accumulator)
{
  if (accumulator.count() == 0) {
    os << "None None None";
  } else {
    os << accumulator.min() << " " << accumulator.max() << " " << accumulator.mean();
  }
  return os;
}

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__MATH__ACCUMULATOR_HPP_
