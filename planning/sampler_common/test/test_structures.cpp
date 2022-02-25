// Copyright 2022 Tier IV, Inc.
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

#include <sampler_common/structures.hpp>

#include <gtest/gtest.h>

TEST(Path, extendPath)
{
  using sampler_common::Path;
  Path traj1;
  Path traj2;
  Path traj3 = traj1.extend(traj2);
  EXPECT_TRUE(traj3.points.empty());

  traj2.points = {{0, 0}, {1, 1}};
  traj3 = traj1.extend(traj2);
  for (size_t i = 0; i < traj1.points.size(); ++i) {
    EXPECT_EQ(traj3.points[i].x(), traj2.points[i].x());
    EXPECT_EQ(traj3.points[i].y(), traj2.points[i].y());
  }

  traj2.points = {{2, 2}, {3, 3}};
  traj3 = traj3.extend(traj2);
  ASSERT_EQ(traj3.points.size(), 4ul);
  for (size_t i = 0; i < traj1.points.size(); ++i) {
    EXPECT_EQ(traj3.points[i].x(), i);
    EXPECT_EQ(traj3.points[i].y(), i);
  }
}
