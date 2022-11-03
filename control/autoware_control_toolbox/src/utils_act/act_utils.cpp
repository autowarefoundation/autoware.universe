// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "utils_act/act_utils.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace ns_utils
{

/**
 * @brief some numerator and denominator can be defined by leading zeros like [0, 0, 1], and we want
 * to strip away the leading zeros.
 * */
void stripVectorZerosFromLeft(std::vector<double> & num_or_den)
{
  for (auto it = num_or_den.begin(); it != num_or_den.end();) {
    auto const & value_of_it = *it;
    if (std::fabs(value_of_it) <= std::numeric_limits<double>::epsilon()) {
      num_or_den.erase(it);
    } else {
      return;
    }
  }
}

}  // namespace ns_utils
