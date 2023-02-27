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

#ifndef TIER4_AUTOWARE_UTILS__MATH__PRECOMPUTE_HPP_
#define TIER4_AUTOWARE_UTILS__MATH__PRECOMPUTE_HPP_

#include <cmath>
#include <vector>

namespace tier4_autoware_utils
{

class PrecomputedTrigFunc
{
  const float PI = std::acos(-1);

  size_t table_size_;
  std::vector<float> sin_table_;
  std::vector<float> cos_table_;

public:
  PrecomputedTrigFunc(size_t table_size) : table_size_(table_size)
  {
    sin_table_.resize(table_size_);
    cos_table_.resize(table_size_);

    for (size_t i = 0; i < table_size_; i++) {
      float degree = 360.f * i / table_size_;
      float radian = degree * (PI / 180.f);
      sin_table_[i] = std::sin(radian);
      cos_table_[i] = std::cos(radian);
    }
  }

  float sin(float radian)
  {
    float degree = radian * (180.f / PI) * (table_size_ / 360.f);
    return sin_table_[(std::lround(degree) % table_size_ + table_size_) % table_size_];
  }

  inline float cos(float radian)
  {
    float degree = radian * (180.f / PI) * (table_size_ / 360.f);
    return cos_table_[(std::lround(degree) % table_size_ + table_size_) % table_size_];
  }
};

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__MATH__PRECOMPUTE_HPP_
