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

// Strictly monotonic increasing
inline bool isIncreasing(const std::vector<double> &x)
{
  if (x.empty())
  {
    throw std::invalid_argument("Points is empty.");
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
    throw std::invalid_argument("Points is empty.");
  }

  // x(i) < x(i+1) condition check ( x(i) == x(i+1) is allowed)
  auto const &is_strictly_increasing_it = std::adjacent_find(x.begin(), x.end(), std::greater<>());

  return is_strictly_increasing_it == x.cend();
}

inline bool isStrictlyMonotonic(const std::vector<double> &x)
{
  if (x.empty())
  {
    throw std::invalid_argument("Points is empty.");
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
  const std::vector<double> &base_keys, const std::vector<double> &query_keys)
{
  // when vectors are empty
  if (base_keys.empty() || query_keys.empty())
  {
    throw std::invalid_argument("Points is empty.");
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

  // when query_keys is out of base_keys (This function does not allow exterior division.)
  if (query_keys.front() < base_keys.front() || base_keys.back() < query_keys.back())
  {
    throw std::invalid_argument("query_keys is out of base_keys");
  }
}

template<class T>
void validateKeysAndValues(
  const std::vector<double> &base_keys, const std::vector<T> &base_values)
{
  // when vectors are empty
  if (base_keys.empty() || base_values.empty())
  {
    throw std::invalid_argument("Points is empty.");
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
}  // namespace interpolation_utils

#endif  // INTERPOLATION__INTERPOLATION_UTILS_HPP_
