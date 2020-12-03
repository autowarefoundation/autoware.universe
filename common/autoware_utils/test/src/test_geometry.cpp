/*
 * Copyright 2020 TierIV. All rights reserved.
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

#include <gtest/gtest.h>

#include <autoware_utils/geometry.hpp>
#include <autoware_utils/unit_conversion.hpp>

TEST(geometry, getPoint)
{
  using autoware_utils::getPoint;

  const double x_ans = 1.0;
  const double y_ans = 2.0;
  const double z_ans = 3.0;

  {
    geometry_msgs::Point p;
    p.x = x_ans;
    p.y = y_ans;
    p.z = z_ans;
    const geometry_msgs::Point p_out = getPoint(p);
    EXPECT_DOUBLE_EQ(p_out.x, x_ans);
    EXPECT_DOUBLE_EQ(p_out.y, y_ans);
    EXPECT_DOUBLE_EQ(p_out.z, z_ans);
  }

  {
    geometry_msgs::Pose p;
    p.position.x = x_ans;
    p.position.y = y_ans;
    p.position.z = z_ans;
    const geometry_msgs::Point p_out = getPoint(p);
    EXPECT_DOUBLE_EQ(p_out.x, x_ans);
    EXPECT_DOUBLE_EQ(p_out.y, y_ans);
    EXPECT_DOUBLE_EQ(p_out.z, z_ans);
  }

  {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = x_ans;
    p.pose.position.y = y_ans;
    p.pose.position.z = z_ans;
    const geometry_msgs::Point p_out = getPoint(p);
    EXPECT_DOUBLE_EQ(p_out.x, x_ans);
    EXPECT_DOUBLE_EQ(p_out.y, y_ans);
    EXPECT_DOUBLE_EQ(p_out.z, z_ans);
  }

  {
    autoware_planning_msgs::PathPoint p;
    p.pose.position.x = x_ans;
    p.pose.position.y = y_ans;
    p.pose.position.z = z_ans;
    const geometry_msgs::Point p_out = getPoint(p);
    EXPECT_DOUBLE_EQ(p_out.x, x_ans);
    EXPECT_DOUBLE_EQ(p_out.y, y_ans);
    EXPECT_DOUBLE_EQ(p_out.z, z_ans);
  }

  {
    autoware_planning_msgs::TrajectoryPoint p;
    p.pose.position.x = x_ans;
    p.pose.position.y = y_ans;
    p.pose.position.z = z_ans;
    const geometry_msgs::Point p_out = getPoint(p);
    EXPECT_DOUBLE_EQ(p_out.x, x_ans);
    EXPECT_DOUBLE_EQ(p_out.y, y_ans);
    EXPECT_DOUBLE_EQ(p_out.z, z_ans);
  }
}

TEST(geometry, createPoint)
{
  using autoware_utils::createPoint;

  const geometry_msgs::Point p_out = createPoint(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(p_out.x, 1.0);
  EXPECT_DOUBLE_EQ(p_out.y, 2.0);
  EXPECT_DOUBLE_EQ(p_out.z, 3.0);
}

TEST(geometry, createQuaternionFromRPY)
{
  using autoware_utils::createQuaternionFromRPY;
  using autoware_utils::deg2rad;

  {
    const tf2::Quaternion q_out = createQuaternionFromRPY(0, 0, 0);
    EXPECT_DOUBLE_EQ(q_out.x(), 0.0);
    EXPECT_DOUBLE_EQ(q_out.y(), 0.0);
    EXPECT_DOUBLE_EQ(q_out.z(), 0.0);
    EXPECT_DOUBLE_EQ(q_out.w(), 1.0);
  }

  {
    const tf2::Quaternion q_out = createQuaternionFromRPY(0, 0, deg2rad(90));
    EXPECT_DOUBLE_EQ(q_out.x(), 0.0);
    EXPECT_DOUBLE_EQ(q_out.y(), 0.0);
    EXPECT_DOUBLE_EQ(q_out.z(), 0.70710678118654757);
    EXPECT_DOUBLE_EQ(q_out.w(), 0.70710678118654757);
  }

  {
    const tf2::Quaternion q_out = createQuaternionFromRPY(deg2rad(30), deg2rad(30), deg2rad(30));
    EXPECT_DOUBLE_EQ(q_out.x(), 0.17677669529663687);
    EXPECT_DOUBLE_EQ(q_out.y(), 0.30618621784789724);
    EXPECT_DOUBLE_EQ(q_out.z(), 0.17677669529663692);
    EXPECT_DOUBLE_EQ(q_out.w(), 0.91855865354369193);
  }
}

TEST(geometry, calcDistance2d)
{
  using autoware_utils::calcDistance2d;

  geometry_msgs::Point point;
  point.x = 1.0;
  point.y = 2.0;
  point.z = 3.0;

  geometry_msgs::Pose pose;
  pose.position.x = 5.0;
  pose.position.y = 5.0;
  pose.position.z = 4.0;

  EXPECT_DOUBLE_EQ(calcDistance2d(point, pose), 5.0);
}

TEST(geometry, calcDistance3d)
{
  using autoware_utils::calcDistance3d;

  geometry_msgs::Point point;
  point.x = 1.0;
  point.y = 2.0;
  point.z = 3.0;

  geometry_msgs::Pose pose;
  pose.position.x = 3.0;
  pose.position.y = 4.0;
  pose.position.z = 4.0;

  EXPECT_DOUBLE_EQ(calcDistance3d(point, pose), 3.0);
}
