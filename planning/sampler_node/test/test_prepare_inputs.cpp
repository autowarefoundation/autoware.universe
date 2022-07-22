/*
 * Copyright 2022 TierIV. All rights reserved.
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

#include <sampler_node/prepare_inputs.hpp>

#include <gtest/gtest.h>

TEST(prepare_inputs, preparePreviousTrajectory)
{
  constexpr auto TOL = 1e-3;
  frenet_planner::Trajectory trajectory;
  trajectory.points = {{0, 0}, {1, 1}, {1, 2}};
  sampler_common::transform::Spline2D path_spline({0, 1, 2}, {0, 0, 0});
  const auto prev_traj = sampler_node::preparePreviousTrajectory(trajectory, path_spline);
  ASSERT_EQ(trajectory.points.size(), prev_traj.points.size());
  EXPECT_NEAR(prev_traj.frenet_points[0].s, 0, TOL);
  EXPECT_NEAR(prev_traj.frenet_points[0].d, 0, TOL);
  EXPECT_NEAR(prev_traj.frenet_points[1].s, 1, TOL);
  EXPECT_NEAR(prev_traj.frenet_points[1].d, 1, TOL);
  EXPECT_NEAR(prev_traj.frenet_points[2].s, 1, TOL);
  EXPECT_NEAR(prev_traj.frenet_points[2].d, 2, TOL);

  sampler_common::transform::Spline2D path_spline2({-2, 0, 1, 2}, {1, 1, 1, 1});
  const auto prev_traj2 = sampler_node::preparePreviousTrajectory(prev_traj, path_spline2);
  ASSERT_EQ(prev_traj.points.size(), prev_traj2.points.size());
  EXPECT_NEAR(prev_traj2.frenet_points[0].s, 2, TOL);
  EXPECT_NEAR(prev_traj2.frenet_points[0].d, -1, TOL);
  EXPECT_NEAR(prev_traj2.frenet_points[1].s, 3, TOL);
  EXPECT_NEAR(prev_traj2.frenet_points[1].d, 0, TOL);
  EXPECT_NEAR(prev_traj2.frenet_points[2].s, 3, TOL);
  EXPECT_NEAR(prev_traj2.frenet_points[2].d, 1, TOL);
}
