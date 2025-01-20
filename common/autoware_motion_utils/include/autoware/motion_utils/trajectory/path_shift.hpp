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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY__PATH_SHIFT_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY__PATH_SHIFT_HPP_

namespace autoware::motion_utils
{
/**
 * @brief Calculates the velocity required for shifting
 * @param lateral lateral distance
 * @param jerk lateral jerk
 * @param longitudinal_distance longitudinal distance
 * @return velocity
 */
double calc_feasible_velocity_from_jerk(
  const double lateral, const double jerk, const double longitudinal_distance);

/**
 * @brief Calculates the lateral distance required for shifting
 * @param longitudinal longitudinal distance
 * @param jerk lateral jerk
 * @param velocity velocity
 * @return lateral distance
 */
double calc_lateral_dist_from_jerk(
  const double longitudinal, const double jerk, const double velocity);

/**
 * @brief Calculates the lateral distance required for shifting
 * @param lateral lateral distance
 * @param jerk lateral jerk
 * @param velocity velocity
 * @return longitudinal distance
 */
double calc_longitudinal_dist_from_jerk(
  const double lateral, const double jerk, const double velocity);

/**
 * @brief Calculates the total time required for shifting
 * @param lateral lateral distance
 * @param jerk lateral jerk
 * @param acc lateral acceleration
 * @return time
 */
double calc_shift_time_from_jerk(const double lateral, const double jerk, const double acc);

/**
 * @brief Calculates the required jerk from lateral/longitudinal distance
 * @param lateral lateral distance
 * @param longitudinal longitudinal distance
 * @param velocity velocity
 * @return jerk
 */
double calc_jerk_from_lat_lon_distance(
  const double lateral, const double longitudinal, const double velocity);

}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY__PATH_SHIFT_HPP_
