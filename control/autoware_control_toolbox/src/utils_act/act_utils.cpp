/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "utils_act/act_utils.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace ns_utils
{
// Time methods.
double tic()
{
  struct timespec t{};
  clock_gettime(CLOCK_REALTIME, &t);
  return static_cast<double>(t.tv_sec) * 1000. + static_cast<double>(t.tv_nsec) / 1000000.;
}

double toc(double start)
{ return tic() - start; }

template<typename T>
void computeYawFromXY(const std::vector<T> &xvect, const std::vector<T> &yvect, std::vector<T> &yaw_vect)
{
  for (size_t k = 1; k < xvect.size(); ++k)
  {
    auto const &&dx = xvect[k] - xvect[k - 1];
    auto const &&dy = yvect[k] - yvect[k - 1];

    T const &&yaw_angle = std::atan2(dy, dx);
    yaw_vect.template emplace_back(yaw_angle);
  }

  if (yaw_vect.size() > 1)
  {
    yaw_vect.template emplace_back(yaw_vect.back());
  }
}

/**
 * @brief some numerator and denominator can be defined by leading zeros like [0, 0, 1], and we want
 * to strip away the leading zeros.
 * */
void stripVectorZerosFromLeft(std::vector<double> &num_or_den)
{
  for (auto it = num_or_den.begin(); it != num_or_den.end();)
  {
    auto const &value_of_it = *it;
    if (std::fabs(value_of_it) <= std::numeric_limits<double>::epsilon())
    {
      num_or_den.erase(it);
    } else
    {
      return;
    }
  }
}

}  // namespace ns_utils
