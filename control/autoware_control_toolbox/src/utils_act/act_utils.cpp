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
  struct timespec t
  {
  };
  clock_gettime(CLOCK_REALTIME, &t);
  return static_cast<double>(t.tv_sec) * 1000. + static_cast<double>(t.tv_nsec) / 1000000.;
}

double toc(double start) { return tic() - start; }

void interp1d_map_linear(
  std::vector<double> const & sbase_coord_vect, std::vector<double> const & base_data_vect,
  std::vector<double> const & snew_coords, std::vector<double> & new_data_to_be_interpolated)
{
  // Prepare the data container to be interpolated.
  new_data_to_be_interpolated.

    clear();

  new_data_to_be_interpolated.reserve(snew_coords.

                                      size()

  );

  auto const && EPS = std::numeric_limits<double>::epsilon();

  // For each coordinate in the new coordinate vector.
  for (double const & sk : snew_coords) {
    // Get indices.
    size_t left = binary_index_search(sk, sbase_coord_vect);
    size_t right = left + 1;  // We guaranteed the existence of right.

    // find interval length and ratio.
    auto const & s0 = sbase_coord_vect[left];
    auto const & s1 = sbase_coord_vect[right];

    auto const & ds_interval = std::max(s1 - s0, EPS);  // to prevent zero division
    auto const & ratio = clamp(ds_interval, 0.0, 1.0);

    // Get terminal data items.
    auto const && xk =
      base_data_vect[left] + ratio * (base_data_vect[right] - base_data_vect[left]);

    // Push back.
    new_data_to_be_interpolated.emplace_back(xk);
  }
}

void interp1d_linear(
  std::vector<double> const & sbase_coord_vector, std::vector<double> const & base_data_data_vector,
  double const & snew_coord, double & new_data_to_be_interpolated)
{
  auto const && EPS = std::numeric_limits<double>::epsilon();

  // For each coordinate in the new coordinate vector.
  // Get indices.
  size_t const && left = binary_index_search(snew_coord, sbase_coord_vector);
  size_t const && right = left + 1;  // We guaranteed the existence of right.

  // find interval length and ratio.
  auto const & s0 = sbase_coord_vector[left];
  auto const & s1 = sbase_coord_vector[right];

  auto const & ds_interval = std::max(s1 - s0, EPS);  // to prevent zero division
  auto const & ratio = clamp(ds_interval, 0.0, 1.0);

  // Get terminal data items.
  auto const && xk = base_data_data_vector[left] +
                     ratio * (base_data_data_vector[right] - base_data_data_vector[left]);

  // Push back.
  new_data_to_be_interpolated = xk;
}

template <typename T>
void computeYawFromXY(
  const std::vector<T> & xvect, const std::vector<T> & yvect, std::vector<T> & yaw_vect)
{
  for (size_t k = 1; k < xvect.size(); ++k) {
    auto const && dx = xvect[k] - xvect[k - 1];
    auto const && dy = yvect[k] - yvect[k - 1];

    T const && yaw_angle = std::atan2(dy, dx);
    yaw_vect.template emplace_back(yaw_angle);
  }

  if (yaw_vect.size() > 1) {
    yaw_vect.template emplace_back(yaw_vect.back());
  }
}

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
