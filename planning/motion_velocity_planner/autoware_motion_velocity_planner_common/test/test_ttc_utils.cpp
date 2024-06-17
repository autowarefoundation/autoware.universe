// Copyright 2024 Tier IV, Inc.
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

#include "autoware/motion_velocity_planner_common/ttc_utils.hpp"

#include <boost/geometry/geometry.hpp>

#include <gtest/gtest.h>

using autoware::motion_velocity_planner::CollisionChecker;
using autoware::motion_velocity_planner::CollisionTimeRange;
using autoware::universe_utils::Line2d;
using autoware::universe_utils::MultiLineString2d;
using autoware::universe_utils::MultiPoint2d;
using autoware::universe_utils::MultiPolygon2d;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;

TEST(TestTTCUtils, CollisionTimeRange)
{
  // throw when start of range is after the end
  EXPECT_ANY_THROW(CollisionTimeRange(3.0, 1.0));
  EXPECT_ANY_THROW(CollisionTimeRange(999.0, 998.999));
  EXPECT_ANY_THROW(CollisionTimeRange(0.0, -1.0));
  EXPECT_ANY_THROW(CollisionTimeRange(-1.0, -3.0));
  EXPECT_NO_THROW(CollisionTimeRange(1.0, 3.0));
  EXPECT_NO_THROW(CollisionTimeRange(0.0, 0.0));
  EXPECT_NO_THROW(CollisionTimeRange(-0.0, -0.0));
  EXPECT_NO_THROW(CollisionTimeRange(998.999, 999.0));
  EXPECT_NO_THROW(CollisionTimeRange(-1.0, 0.0));
  EXPECT_NO_THROW(CollisionTimeRange(-3.0, -1.0));

  const CollisionTimeRange r0(-10.0, 10.0);
  const CollisionTimeRange r1(-5.0, -4.0);
  const CollisionTimeRange r2(-4.0, 4.0);
  const CollisionTimeRange r3(3.0, 5.0);
  // r0 overlaps all other ranges
  EXPECT_DOUBLE_EQ(r0.time_to_collision(r1), 0.0);
  EXPECT_DOUBLE_EQ(r0.time_to_collision(r2), 0.0);
  EXPECT_DOUBLE_EQ(r0.time_to_collision(r3), 0.0);
  // r1 ends when r2 starts; far from r3
  EXPECT_DOUBLE_EQ(r1.time_to_collision(r2), 0.0);
  EXPECT_DOUBLE_EQ(r1.time_to_collision(r3), 7.0);
  // r2 overlaps r3
  EXPECT_DOUBLE_EQ(r2.time_to_collision(r3), 0.0);
}

TEST(TestTTCUtils, CalculateTTC)
{
}
