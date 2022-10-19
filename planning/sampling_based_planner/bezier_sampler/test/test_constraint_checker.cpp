/*
 * Copyright 2021 TierIV. All rights reserved.
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

#include <bezier_sampler/bezier.hpp>
#include <bezier_sampler/constraint_checker.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <gtest/gtest.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

TEST(isCollisionFree, position_square)
{
  using bezier_sampler::Bezier;
  using bezier_sampler::ConstraintChecker;
  using bezier_sampler::ConstraintParameters;
  // OccupancyGrid
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 3;
  grid.info.height = 3;
  grid.info.resolution = 1.0;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 3.0;
  grid.info.origin.orientation = tf2::toMsg(tf2::Quaternion({0.0, 0.0, 1.0}, 0.0));
  grid.data = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Parameters
  ConstraintParameters params;
  // Tests
  {
    ConstraintChecker cc(grid, params);
    ASSERT_TRUE(cc.isCollisionFree({0.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({0.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({0.5, 2.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 2.5}));
    ASSERT_TRUE(cc.isCollisionFree({2.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({2.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({2.5, 2.5}));
  }
  {
    grid.data = {100, 100, 100, 100, 100, 100, 100, 100, 100};
    ConstraintChecker cc(grid, params);
    ASSERT_FALSE(cc.isCollisionFree({0.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 2.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 2.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 2.5}));
  }
  {
    grid.data = {0, 100, 100, 100, 100, 100, 100, 100, 100};
    ConstraintChecker cc(grid, params);
    ASSERT_FALSE(cc.isCollisionFree({0.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({0.5, 2.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 2.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 2.5}));
  }
  {
    grid.data = {100, 100, 100, 0, 0, 0, 100, 100, 100};
    ConstraintChecker cc(grid, params);
    ASSERT_TRUE(cc.isCollisionFree({0.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({2.5, 1.5}));
    for (double x = 0.5; x <= 2.5; ++x)
      for (double y = 0.5; y <= 2.5; y += 2)  // skip the middle row
        ASSERT_FALSE(cc.isCollisionFree({x, y}));
  }
  // Change origin
  grid.info.origin.position.x = 3.0;
  grid.info.origin.position.y = 3.0;
  {
    ConstraintChecker cc(grid, params);
    ASSERT_TRUE(cc.isCollisionFree({3.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({4.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({5.5, 1.5}));
    for (double x = 3.5; x <= 5.5; ++x)
      for (double y = 0.5; y <= 2.5; y += 2)  // skip the middle row
        ASSERT_FALSE(cc.isCollisionFree({x, y}));
  }
  // Reset origin
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 3.0;
  // Change orientation by 90 degree
  grid.info.origin.orientation = tf2::toMsg(tf2::Quaternion({0.0, 0.0, 1.0}, M_PI / 2));
  {
    ConstraintChecker cc(grid, params);
    ASSERT_TRUE(cc.isCollisionFree({1.5, 3.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 4.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 5.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 3.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 4.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 5.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 3.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 4.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 5.5}));
  }
  // Change origin (and keep the 90 degrees orientation)
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  {
    ConstraintChecker cc(grid, params);
    ASSERT_TRUE(cc.isCollisionFree({1.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 2.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 2.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 2.5}));
  }
}

TEST(isCollisionFree, position_rectangle)
{
  using bezier_sampler::Bezier;
  using bezier_sampler::ConstraintChecker;
  using bezier_sampler::ConstraintParameters;
  // OccupancyGrid
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 3;
  grid.info.height = 2;
  grid.info.resolution = 1.0;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 2.0;
  grid.info.origin.orientation = tf2::toMsg(tf2::Quaternion({0.0, 0.0, 1.0}, 0.0));
  grid.data = {0, 0, 0, 0, 0, 0};

  // Parameters
  ConstraintParameters params;
  // Tests
  {
    ConstraintChecker cc(grid, params);
    ASSERT_TRUE(cc.isCollisionFree({0.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({2.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({0.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 1.5}));
    ASSERT_TRUE(cc.isCollisionFree({2.5, 1.5}));
  }
  {
    grid.data = {100, 100, 100, 100, 100, 100};
    ConstraintChecker cc(grid, params);
    ASSERT_FALSE(cc.isCollisionFree({0.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 1.5}));
  }
  {
    grid.data = {0, 100, 100, 100, 100, 100};
    ConstraintChecker cc(grid, params);
    ASSERT_FALSE(cc.isCollisionFree({0.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({0.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 1.5}));
  }
  {
    grid.data = {100, 100, 100, 0, 0, 0};
    ConstraintChecker cc(grid, params);
    ASSERT_TRUE(cc.isCollisionFree({0.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({2.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({1.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 1.5}));
  }
  // Change origin
  grid.info.origin.position.x = 2.0;
  grid.info.origin.position.y = 2.0;
  {
    ConstraintChecker cc(grid, params);
    ASSERT_TRUE(cc.isCollisionFree({2.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({3.5, 0.5}));
    ASSERT_TRUE(cc.isCollisionFree({4.5, 0.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({3.5, 1.5}));
    ASSERT_FALSE(cc.isCollisionFree({4.5, 1.5}));
  }
  // Reset origin
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 2.0;
  // Change orientation by 90 degree
  grid.info.origin.orientation = tf2::toMsg(tf2::Quaternion({0.0, 0.0, 1.0}, M_PI / 2));
  {
    ConstraintChecker cc(grid, params);
    ASSERT_FALSE(cc.isCollisionFree({0.5, 2.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 3.5}));
    ASSERT_FALSE(cc.isCollisionFree({0.5, 4.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 2.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 3.5}));
    ASSERT_TRUE(cc.isCollisionFree({1.5, 4.5}));
  }
  // Change origin (and keep the 90 degree orientation)
  grid.info.origin.position.x = 2.0;
  grid.info.origin.position.y = 2.0;
  {
    ConstraintChecker cc(grid, params);
    ASSERT_FALSE(cc.isCollisionFree({2.5, 2.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 3.5}));
    ASSERT_FALSE(cc.isCollisionFree({2.5, 4.5}));
    ASSERT_TRUE(cc.isCollisionFree({3.5, 2.5}));
    ASSERT_TRUE(cc.isCollisionFree({3.5, 3.5}));
    ASSERT_TRUE(cc.isCollisionFree({3.5, 4.5}));
  }
}
