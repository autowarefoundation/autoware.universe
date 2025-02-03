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

#include "autoware/trajectory/utils/find_intervals.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::trajectory::detail::impl
{

std::vector<Interval> find_intervals_impl(
  const std::vector<double> & bases, const std::function<bool(const double &)> & constraint)
{
  std::vector<Interval> intervals;

  std::optional<double> start = std::nullopt;
  for (size_t i = 0; i < bases.size(); ++i) {
    if (!start && constraint(bases.at(i))) {
      start = bases.at(i);  // Start a new interval
    } else if (start && (!constraint(bases.at(i)) || i == bases.size() - 1)) {
      // End the current interval if the constraint fails or it's the last element
      intervals.emplace_back(Interval{start.value(), bases.at(i - !constraint(bases.at(i)))});
      start = std::nullopt;  // Reset the start
    }
  }
  return intervals;
}

}  // namespace autoware::trajectory::detail::impl
