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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__DETAIL__MERGE_VECTOR_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__DETAIL__MERGE_VECTOR_HPP_

#include <set>
#include <vector>

namespace autoware::motion_utils::trajectory_container::detail
{
/**
 * @brief Merge multiple vectors into one, keeping only unique elements.
 * @tparam Vectors Variadic template parameter for vector types.
 * @param vectors Vectors to be merged.
 * @return Merged vector with unique elements.
 */
template <typename... Vectors>
std::vector<double> merge_vectors(const Vectors &... vectors)
{
  std::set<double> unique_elements;

  // Helper function to insert elements into the set
  auto insert_elements = [&unique_elements](const auto & vec) {
    unique_elements.insert(vec.begin(), vec.end());
  };

  // Expand the parameter pack and insert elements from each vector
  (insert_elements(vectors), ...);

  // Convert the set back to a vector
  std::vector<double> result(unique_elements.begin(), unique_elements.end());

  return result;
}

}  // namespace autoware::motion_utils::trajectory_container::detail

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__DETAIL__MERGE_VECTOR_HPP_
