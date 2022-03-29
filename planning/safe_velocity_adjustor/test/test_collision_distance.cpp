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

#include "safe_velocity_adjustor/collision_distance.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <pcl/impl/point_types.hpp>

#include <autoware_auto_planning_msgs/msg/detail/trajectory_point__struct.hpp>

#include <gtest/gtest.h>

TEST(TestCollisionDistance, forwardSimulatedVector)
{
  using safe_velocity_adjustor::forwardSimulatedVector;
  using safe_velocity_adjustor::linestring_t;
  autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;

  trajectory_point.pose.position.x = 0.0;
  trajectory_point.pose.position.y = 0.0;

  auto duration = 0.0;
  auto extra_dist = 0.0;

  const auto check_vector = [&](const auto vector_length) {
    trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(0.0);
    auto vector = forwardSimulatedVector(trajectory_point, duration, extra_dist);
    EXPECT_DOUBLE_EQ(vector[0].x(), trajectory_point.pose.position.x);
    EXPECT_DOUBLE_EQ(vector[0].y(), trajectory_point.pose.position.y);
    EXPECT_DOUBLE_EQ(vector[1].x(), vector_length);
    EXPECT_DOUBLE_EQ(vector[1].y(), 0.0);
    trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(M_PI_2);
    vector = forwardSimulatedVector(trajectory_point, duration, extra_dist);
    EXPECT_DOUBLE_EQ(vector[0].x(), trajectory_point.pose.position.x);
    EXPECT_DOUBLE_EQ(vector[0].y(), trajectory_point.pose.position.y);
    EXPECT_NEAR(vector[1].x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector[1].y(), vector_length);
    trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(M_PI_4);
    vector = forwardSimulatedVector(trajectory_point, duration, extra_dist);
    EXPECT_DOUBLE_EQ(vector[0].x(), trajectory_point.pose.position.x);
    EXPECT_DOUBLE_EQ(vector[0].y(), trajectory_point.pose.position.y);
    EXPECT_DOUBLE_EQ(vector[1].x(), std::sqrt(0.5) * vector_length);
    EXPECT_DOUBLE_EQ(vector[1].y(), std::sqrt(0.5) * vector_length);
    trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(-M_PI_2);
    vector = forwardSimulatedVector(trajectory_point, duration, extra_dist);
    EXPECT_DOUBLE_EQ(vector[0].x(), trajectory_point.pose.position.x);
    EXPECT_DOUBLE_EQ(vector[0].y(), trajectory_point.pose.position.y);
    EXPECT_NEAR(vector[1].x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector[1].y(), -vector_length);
  };

  // 0 velocity: whatever the duration the vector length is always = to extra_dist
  trajectory_point.longitudinal_velocity_mps = 0.0;

  duration = 0.0;
  extra_dist = 0.0;
  check_vector(extra_dist);

  duration = 5.0;
  extra_dist = 2.0;
  check_vector(extra_dist);

  duration = -5.0;
  extra_dist = 3.5;
  check_vector(extra_dist);

  // set non-zero velocities
  trajectory_point.longitudinal_velocity_mps = 1.0;

  duration = 1.0;
  extra_dist = 0.0;
  check_vector(1.0 + extra_dist);

  duration = 5.0;
  extra_dist = 2.0;
  check_vector(5.0 + extra_dist);

  duration = -5.0;
  extra_dist = 3.5;
  check_vector(-5.0 + extra_dist);
}
TEST(TestCollisionDistance, distanceToClosestCollision)
{
  using safe_velocity_adjustor::distanceToClosestCollision;
  using safe_velocity_adjustor::point_t;
  using safe_velocity_adjustor::polygon_t;
  autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
  trajectory_point.pose.position.x = 0;
  trajectory_point.pose.position.y = 0;
  trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(0.0);
  polygon_t footprint;
  footprint.outer() = {point_t{0, 1}, point_t{5, 1}, point_t{5, -1}, point_t{0, -1}, point_t{0, 1}};
  pcl::PointCloud<pcl::PointXYZ> obstacle_points;
  obstacle_points.points.emplace_back(6, 2, 0);  // outside of the footprint
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_FALSE(dist.has_value());
  }
  obstacle_points.points.emplace_back(4, 0, 0);  // distance of 4
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 4.0);
  }
  obstacle_points.points.emplace_back(
    4.5, 0, 0);  // distance of 4.5, does not change the minimum distance
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 4.0);
  }
  obstacle_points.points.emplace_back(2.0, 0.5, 0);  // new minumum distance of 2.0
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 2.0);
  }
  obstacle_points.points.emplace_back(1.5, -0.75, 0);  // new minumum distance of 1.5
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 1.5);
  }

  // Change footprint heading
  trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(M_PI_2);
  footprint.outer() = {
    point_t{-1, 0}, point_t{-1, 5}, point_t{1, 5}, point_t{1, 0}, point_t{-1, 0}};
  obstacle_points.clear();
  obstacle_points.emplace_back(-2, 3, 0);  // outside of the footprint
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_FALSE(dist.has_value());
  }
  obstacle_points.points.emplace_back(-0.5, 4, 0);  // new minumum distance of 4
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 4);
  }
  obstacle_points.points.emplace_back(0, 4, 0);  // no change in the minimum distance
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 4);
  }
  obstacle_points.points.emplace_back(0.5, 0.5, 0);  // change the minimum distance
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 0.5);
  }

  // Change footprint heading
  trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(M_PI_4);
  footprint.outer() = {
    point_t{-0.5, 0.5}, point_t{2, 3}, point_t{3, 2}, point_t{0.5, -0.5}, point_t{-0.5, 0.5}};
  obstacle_points.clear();
  obstacle_points.emplace_back(3, 3, 0);  // outside of the footprint
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_FALSE(dist.has_value());
  }
  obstacle_points.points.emplace_back(2.25, 1.75, 0);  // new minumum distance of 4
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, std::sqrt(8));
  }
  obstacle_points.points.emplace_back(2, 2, 0);  // no change in the minimum distance
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, std::sqrt(8));
  }
  obstacle_points.points.emplace_back(0.25, 0.75, 0);  // change the minimum distance
  {
    const auto dist = distanceToClosestCollision(trajectory_point, footprint, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, std::sqrt(0.5));
  }
}
