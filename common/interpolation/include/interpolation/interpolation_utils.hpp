// Copyright 2021 Tier IV, Inc.
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

#ifndef INTERPOLATION__INTERPOLATION_UTILS_HPP_
#define INTERPOLATION__INTERPOLATION_UTILS_HPP_

#include <array>
#include <stdexcept>
#include <vector>
#include <algorithm>

namespace interpolation_utils
{
constexpr double EPS = std::numeric_limits<double>::epsilon();

template<class T, typename std::enable_if_t<std::is_floating_point_v<T>, bool> * = nullptr>
bool isEqual(T a, T b)
{
  return abs(a - b) < std::numeric_limits<T>::epsilon();
}

// Strictly monotonic increasing
inline bool isIncreasing(const std::vector<double> &x)
{
  if (x.empty())
  {
    throw std::invalid_argument("The vector x is empty.");
  }

  // x(i) <= x(i+1) condition check
  auto const &is_increasing_it = std::adjacent_find(x.begin(), x.end(), std::greater_equal<>());

  return is_increasing_it == x.cend();
}

// Strictly increasing and monotonic
inline bool isNotDecreasing(const std::vector<double> &x)
{
  if (x.empty())
  {
    throw std::invalid_argument("The vector x is empty.");
  }

  // x(i) < x(i+1) condition check ( x(i) == x(i+1) is allowed)
  auto const &is_strictly_increasing_it = std::adjacent_find(x.begin(), x.end(), std::greater<>());

  return is_strictly_increasing_it == x.cend();
}

inline bool isStrictlyMonotonic(const std::vector<double> &x)
{
  if (x.empty())
  {
    throw std::invalid_argument("The vector x is empty.");
  }

  // x(i) <= or >= x(i+1) condition check
  auto const &is_strictly_increasing_it = std::adjacent_find(x.begin(), x.end(), std::greater_equal<>());
  auto const &is_strictly_decreasing_it = std::adjacent_find(x.begin(), x.end(), std::less_equal<>());

  // if cannot find <= or >= conditions, the vector is strictly monotonic.
  bool const is_strictly_increasing = is_strictly_increasing_it == x.cend();
  bool const is_strictly_decreasing = is_strictly_decreasing_it == x.cend();

  bool const is_monotonic = (is_strictly_increasing && !is_strictly_decreasing) ||
                            (!is_strictly_increasing && is_strictly_decreasing);

  return is_monotonic;
}

inline void validateKeys(
  const std::vector<double> &base_keys,
  const std::vector<double> &query_keys,
  const bool &extrapolate_end_points = false)
{
  // when vectors are empty
  if (base_keys.empty() || query_keys.empty())
  {
    throw std::invalid_argument("Either the base_keys or the query_keys is empty.");
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2)
  {
    throw std::invalid_argument(
      "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()));
  }

  // when indices are not sorted
  if (!isIncreasing(base_keys) || !isNotDecreasing(query_keys))
  {
    throw std::invalid_argument("Either base_keys or query_keys is not sorted.");
  }

  // When query_keys is out of base_keys (This function does not allow extrapolation when extrapolation boolean is false  // ).
  if ((query_keys.front() < base_keys.front() || base_keys.back() < query_keys.back()) && !extrapolate_end_points)
  {
    throw std::invalid_argument("The query_keys is out of the range of base_keys, consider to extrapolate option");
  }
}

template<class T>
void validateKeysAndValues(
  const std::vector<double> &base_keys, const std::vector<T> &base_values)
{
  // when vectors are empty
  if (base_keys.empty() || base_values.empty())
  {
    throw std::invalid_argument("Either the base_keys or the query_keys is empty");
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2 || base_values.size() < 2)
  {
    throw std::invalid_argument(
      "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()) +
      ", base_values.size() = " + std::to_string(base_values.size()));
  }

  // when sizes of indices and values are not same
  if (base_keys.size() != base_values.size())
  {
    throw std::invalid_argument("The size of base_keys and base_values are not the same.");
  }
}

/**
 * Linear extrapolation
 * */
template<typename T, typename std::enable_if_t<std::is_floating_point_v<T>> * = nullptr>
void lerp_extrapolate(
  std::vector<T> const &base_keys, std::vector<T> const &base_values, T const &query_key, T &query_value)
{
  if (query_key < base_keys[0])
  {
    auto const &t0 = base_keys[0];
    auto const &t1 = base_keys[1];

    auto const &y0 = base_values[0];
    auto const &y1 = base_values[1];

    auto const &ratio = (t0 - query_key) / (t1 - t0);
    query_value = y0 - ratio * (y1 - y0);
  }

  if (query_key > base_keys.back())
  {
    auto const &tn = base_keys.rbegin()[0];
    auto const &tn_1 = base_keys.rbegin()[1];

    auto const &yn = base_values.rbegin()[0];
    auto const &yn_1 = base_values.rbegin()[1];

    auto const &ratio = (query_key - tn) / (tn - tn_1);
    query_value = yn + ratio * (yn - yn_1); // extrapolation and lerp have different semantics.
  }
}

}  // namespace interpolation_utils

#endif  // INTERPOLATION__INTERPOLATION_UTILS_HPP_
