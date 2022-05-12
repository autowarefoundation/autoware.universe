// Copyright 2020 Tier IV, Inc.
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

#ifndef MAP_TF_GENERATOR__UNIFORM_RANDOM_HPP_
#define MAP_TF_GENERATOR__UNIFORM_RANDOM_HPP_

#include <random>
#include <vector>

std::vector<int> UniformRandom(const int max_excluded, const int n)
{
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(0, max_excluded - 1);

  std::vector<int> v(n);
  for (int i = 0; i < n; ++i) {
    v[i] = distribution(generator);
  }
  return v;
}

#endif  // MAP_TF_GENERATOR__UNIFORM_RANDOM_HPP_
