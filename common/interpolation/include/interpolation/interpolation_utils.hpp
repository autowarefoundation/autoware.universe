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

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

namespace interpolation_utils
{

template <class T, typename std::enable_if_t<std::is_floating_point_v<T>, bool> * = nullptr>
bool isEqual(T a, T b)
{
  return std::abs(a - b) < std::numeric_limits<T>::epsilon();
}

// Strictly monotonic increasing
inline bool isIncreasing(const std::vector<double> & x)
{
  if (x.empty()) {
    throw std::invalid_argument("The vector x is empty.");
  }

  // x(i) <= x(i+1) condition check
  const auto & is_increasing_it = std::adjacent_find(x.begin(), x.end(), std::greater_equal<>());

  return is_increasing_it == x.cend();
}

// Strictly increasing and monotonic
inline bool isNotDecreasing(const std::vector<double> & x)
{
  if (x.empty()) {
    throw std::invalid_argument("The vector x is empty.");
  }

  // x(i) < x(i+1) condition check ( x(i) == x(i+1) is allowed)
  const auto & is_strictly_increasing_it = std::adjacent_find(x.begin(), x.end(), std::greater<>());

  return is_strictly_increasing_it == x.cend();
}

inline bool isStrictlyMonotonic(const std::vector<double> & x)
{
  if (x.empty()) {
    throw std::invalid_argument("The vector x is empty.");
  }

  // x(i) <= or >= x(i+1) condition check
  const auto is_strictly_increasing_it =
    std::adjacent_find(x.begin(), x.end(), std::greater_equal<>());
  const auto is_strictly_decreasing_it =
    std::adjacent_find(x.begin(), x.end(), std::less_equal<>());

  // if cannot find <= or >= conditions, the vector is strictly monotonic.
  const bool is_strictly_increasing = is_strictly_increasing_it == x.cend();
  const bool is_strictly_decreasing = is_strictly_decreasing_it == x.cend();

  const bool is_monotonic = (is_strictly_increasing && !is_strictly_decreasing) ||
                            (!is_strictly_increasing && is_strictly_decreasing);

  return is_monotonic;
}

inline std::vector<double> validateKeys(
  const std::vector<double> & base_keys, const std::vector<double> & query_keys,
  const bool extrapolate_end_points = false)
{
  // when vectors are empty
  if (base_keys.empty() || query_keys.empty()) {
    throw std::invalid_argument("Either the base_keys or the query_keys is empty.");
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2) {
    throw std::invalid_argument(
      "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()));
  }

  // when indices are not sorted
  if (!isIncreasing(base_keys) || !isNotDecreasing(query_keys)) {
    throw std::invalid_argument("Either base_keys or query_keys is not sorted.");
  }

  if (extrapolate_end_points) {
    return query_keys;
  }

  // When query_keys is out of base_keys (This function does not allow extrapolation when
  // extrapolation boolean is false).
  if (constexpr double epsilon = 1e-3; (query_keys.front() < base_keys.front() - epsilon ||
                                        base_keys.back() + epsilon < query_keys.back()) &&
                                       !extrapolate_end_points) {
    throw std::invalid_argument(
      "The query_keys is out of the range of base_keys, consider to extrapolate option");
  }

  // NOTE: Due to calculation error of double, a query key may be slightly out of base keys.
  //       Therefore, query keys are cropped here.
  auto validated_query_keys = query_keys;
  validated_query_keys.front() = std::max(validated_query_keys.front(), base_keys.front());
  validated_query_keys.back() = std::min(validated_query_keys.back(), base_keys.back());

  return validated_query_keys;
}

template <class T>
void validateKeysAndValues(
  const std::vector<double> & base_keys, const std::vector<T> & base_values)
{
  // when vectors are empty
  if (base_keys.empty() || base_values.empty()) {
    throw std::invalid_argument("Either the base_keys or the query_keys is empty");
  }

  // when size of vectors are less than 2
  if (base_keys.size() < 2 || base_values.size() < 2) {
    throw std::invalid_argument(
      "The size of points is less than 2. base_keys.size() = " + std::to_string(base_keys.size()) +
      ", base_values.size() = " + std::to_string(base_values.size()));
  }

  // when sizes of indices and values are not same
  if (base_keys.size() != base_values.size()) {
    throw std::invalid_argument("The size of base_keys and base_values are not the same.");
  }
}
}  // namespace interpolation_utils

#endif  // INTERPOLATION__INTERPOLATION_UTILS_HPP_
