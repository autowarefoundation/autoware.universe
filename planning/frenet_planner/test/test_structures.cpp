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

#include <frenet_planner/structures.hpp>

#include <gtest/gtest.h>

TEST(Trajectory, extendTrajectory)
{
  using frenet_planner::Trajectory;
  Trajectory traj1;
  Trajectory traj2;
  Trajectory traj3 = traj1.extend(traj2);
  EXPECT_TRUE(traj3.frenet_points.empty());
  EXPECT_TRUE(traj3.points.empty());

  traj2.frenet_points = {{0, 0}, {1, 1}};
  traj2.points = {{0, 0}, {1, 1}};
  traj3 = traj1.extend(traj2);
  ASSERT_EQ(traj3.frenet_points.size(), traj2.frenet_points.size());
  for (size_t i = 0; i < traj1.frenet_points.size(); ++i) {
    EXPECT_EQ(traj3.frenet_points[i].s, traj2.frenet_points[i].s);
    EXPECT_EQ(traj3.frenet_points[i].d, traj2.frenet_points[i].d);
    EXPECT_EQ(traj3.points[i].x(), traj2.points[i].x());
    EXPECT_EQ(traj3.points[i].y(), traj2.points[i].y());
  }

  traj2.frenet_points = {{2, 2}, {3, 3}};
  traj2.points = {{2, 2}, {3, 3}};
  traj3 = traj3.extend(traj2);
  ASSERT_EQ(traj3.frenet_points.size(), 4ul);
  for (size_t i = 0; i < traj1.frenet_points.size(); ++i) {
    EXPECT_EQ(traj3.frenet_points[i].s, i);
    EXPECT_EQ(traj3.frenet_points[i].d, i);
    EXPECT_EQ(traj3.points[i].x(), i);
    EXPECT_EQ(traj3.points[i].y(), i);
  }
}
