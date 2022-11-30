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

#include "interpolation/linear_interpolation.hpp"

#include <vector>

namespace interpolation
{

double lerp(const double src_val, const double dst_val, const double ratio)
{
  return src_val + (dst_val - src_val) * ratio;
}

std::vector<double> lerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys, bool const & extrapolate_end_points)
{
  // throw exception for invalid arguments
  interpolation_utils::validateKeys(base_keys, query_keys, extrapolate_end_points);
  interpolation_utils::validateKeysAndValues(base_keys, base_values);

  // calculate linear interpolation
  std::vector<double> query_values;
  size_t key_index = 0;

  for (const auto query_key : query_keys) {
    if ((query_key < base_keys.front() || query_key > base_keys.back()) && extrapolate_end_points) {
      double extrapolated_value{};
      interpolation_utils::lerp_extrapolate(base_keys, base_values, query_key, extrapolated_value);
      query_values.emplace_back(extrapolated_value);
      continue;
    }

    while (base_keys.at(key_index + 1) < query_key) {
      ++key_index;
    }

    const double src_val = base_values.at(key_index);
    const double dst_val = base_values.at(key_index + 1);
    const double ratio = (query_key - base_keys.at(key_index)) /
                         (base_keys.at(key_index + 1) - base_keys.at(key_index));

    const double interpolated_val = lerp(src_val, dst_val, ratio);
    query_values.push_back(interpolated_val);
  }

  return query_values;
}

double lerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values, double query_key,
  bool const & extrapolate_end_points)
{
  // throw exception for invalid arguments
  interpolation_utils::validateKeys(
    base_keys, std::vector<double>{query_key}, extrapolate_end_points);
  interpolation_utils::validateKeysAndValues(base_keys, base_values);

  if ((query_key < base_keys.front() || query_key > base_keys.back()) && extrapolate_end_points) {
    double extrapolated_value{};
    interpolation_utils::lerp_extrapolate(base_keys, base_values, query_key, extrapolated_value);

    return extrapolated_value;
  }

  // Binary search the minimum element in the base_key in such way that : query_key <= base_key[i]
  auto const lower_bound_it = std::lower_bound(base_keys.cbegin(), base_keys.cend(), query_key);

  if (lower_bound_it == base_keys.cend()) {
    throw std::domain_error("In LERP scalar : the query key is out of the range.");
  }

  // k1 is the index query_key <=base_keys[k1]
  auto const & k1 = std::distance(base_keys.cbegin(), lower_bound_it);
  auto const & k0 = k1 == 0 ? k1 : k1 - 1;  // if upper bound coincides with the first element

  // if k0 and k1 are the first index of the base_key
  if (k0 == k1) {
    return base_values[k0];
  }

  auto const & ratio = (query_key - base_keys.at(k0)) / (base_keys.at(k1) - base_keys.at(k0));

  // Get terminal data items.
  auto const & query_value = base_values.at(k0) + ratio * (base_values.at(k1) - base_values.at(k0));
  return query_value;
}
}  // namespace interpolation
