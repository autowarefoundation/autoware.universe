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

#include "autoware/motion_utils/trajectory/path_shift.hpp"

#include <gtest/gtest.h>

TEST(path_shift_test, calc_feasible_velocity_from_jerk)
{
  using autoware::motion_utils::calc_feasible_velocity_from_jerk;

  double longitudinal_distance = 0.0;
  double lateral_distance = 0.0;
  double lateral_jerk = 0.0;

  // Condition: zero lateral jerk
  EXPECT_DOUBLE_EQ(
    calc_feasible_velocity_from_jerk(lateral_distance, lateral_jerk, longitudinal_distance),
    1.0e10);

  // Condition: zero lateral distance
  lateral_jerk = 1.0;
  EXPECT_DOUBLE_EQ(
    calc_feasible_velocity_from_jerk(lateral_distance, lateral_jerk, longitudinal_distance),
    1.0e10);

  // Condition: zero longitudinal distance
  lateral_distance = 2.0;
  EXPECT_DOUBLE_EQ(
    calc_feasible_velocity_from_jerk(lateral_distance, lateral_jerk, longitudinal_distance), 0.0);

  // Condition: random condition
  longitudinal_distance = 50.0;
  EXPECT_DOUBLE_EQ(
    calc_feasible_velocity_from_jerk(lateral_distance, lateral_jerk, longitudinal_distance), 12.5);
}

TEST(path_shift_test, calc_lateral_dist_from_jerk)
{
  using autoware::motion_utils::calc_lateral_dist_from_jerk;

  double longitudinal_distance = 0.0;
  double lateral_jerk = 0.0;
  double velocity = 0.0;

  // Condition: zero lateral jerk
  EXPECT_DOUBLE_EQ(
    calc_lateral_dist_from_jerk(longitudinal_distance, lateral_jerk, velocity), 1.0e10);

  // Condition: zero velocity
  lateral_jerk = 2.0;
  EXPECT_DOUBLE_EQ(
    calc_lateral_dist_from_jerk(longitudinal_distance, lateral_jerk, velocity), 1.0e10);

  // Condition: zero longitudinal distance
  velocity = 10.0;
  EXPECT_DOUBLE_EQ(calc_lateral_dist_from_jerk(longitudinal_distance, lateral_jerk, velocity), 0.0);

  // Condition: random condition
  longitudinal_distance = 100.0;
  EXPECT_DOUBLE_EQ(
    calc_lateral_dist_from_jerk(longitudinal_distance, lateral_jerk, velocity), 62.5);
}

TEST(path_shift_test, calc_longitudinal_dist_from_jerk)
{
  using autoware::motion_utils::calc_longitudinal_dist_from_jerk;

  double lateral_distance = 0.0;
  double lateral_jerk = 0.0;
  double velocity = 0.0;

  // Condition: zero lateral jerk
  EXPECT_DOUBLE_EQ(
    calc_longitudinal_dist_from_jerk(lateral_distance, lateral_jerk, velocity), 1.0e10);

  // Condition: zero lateral distance
  lateral_jerk = -1.0;
  velocity = 10.0;
  EXPECT_DOUBLE_EQ(calc_longitudinal_dist_from_jerk(lateral_distance, lateral_jerk, velocity), 0.0);

  // Condition: zero velocity
  velocity = 0.0;
  lateral_distance = 54.0;
  EXPECT_DOUBLE_EQ(calc_longitudinal_dist_from_jerk(lateral_distance, lateral_jerk, velocity), 0.0);

  // Condition: random
  velocity = 8.0;
  EXPECT_DOUBLE_EQ(
    calc_longitudinal_dist_from_jerk(lateral_distance, lateral_jerk, velocity), 96.0);
}
