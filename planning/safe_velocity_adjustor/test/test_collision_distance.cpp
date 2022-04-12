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
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <gtest/gtest.h>
#include <pcl/common/generate.h>
#include <pcl/common/random.h>
#include <pcl/point_cloud.h>

TEST(TestCollisionDistance, forwardSimulatedVector)
{
  using safe_velocity_adjustor::forwardSimulatedVector;
  using safe_velocity_adjustor::segment_t;
  autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;

  trajectory_point.pose.position.x = 0.0;
  trajectory_point.pose.position.y = 0.0;

  auto duration = 0.0;
  auto extra_dist = 0.0;

  const auto check_vector = [&](const auto vector_length) {
    trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(0.0);
    auto vector = forwardSimulatedVector(trajectory_point, duration, extra_dist);
    EXPECT_DOUBLE_EQ(vector.first.x(), trajectory_point.pose.position.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), trajectory_point.pose.position.y);
    EXPECT_DOUBLE_EQ(vector.second.x(), vector_length);
    EXPECT_DOUBLE_EQ(vector.second.y(), 0.0);
    trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(M_PI_2);
    vector = forwardSimulatedVector(trajectory_point, duration, extra_dist);
    EXPECT_DOUBLE_EQ(vector.first.x(), trajectory_point.pose.position.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), trajectory_point.pose.position.y);
    EXPECT_NEAR(vector.second.x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector.second.y(), vector_length);
    trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(M_PI_4);
    vector = forwardSimulatedVector(trajectory_point, duration, extra_dist);
    EXPECT_DOUBLE_EQ(vector.first.x(), trajectory_point.pose.position.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), trajectory_point.pose.position.y);
    EXPECT_DOUBLE_EQ(vector.second.x(), std::sqrt(0.5) * vector_length);
    EXPECT_DOUBLE_EQ(vector.second.y(), std::sqrt(0.5) * vector_length);
    trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(-M_PI_2);
    vector = forwardSimulatedVector(trajectory_point, duration, extra_dist);
    EXPECT_DOUBLE_EQ(vector.first.x(), trajectory_point.pose.position.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), trajectory_point.pose.position.y);
    EXPECT_NEAR(vector.second.x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector.second.y(), -vector_length);
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

  safe_velocity_adjustor::segment_t vector = {{0.0, 0.0}, {5.0, 0.0}};
  safe_velocity_adjustor::polygon_t footprint;
  footprint.outer() = {{0.0, 1.0}, {5.0, 1.0}, {5.0, -1.0}, {0.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  safe_velocity_adjustor::multilinestring_t obstacles;

  std::optional<double> result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_FALSE(result.has_value());

  obstacles.push_back({{-1.0, 0.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_FALSE(result.has_value());

  obstacles.push_back({{1.0, 2.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_FALSE(result.has_value());

  obstacles.push_back({{4.0, 0.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 4.0);

  obstacles.push_back({{3.0, 0.5}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 3.0);

  obstacles.push_back({{2.5, -0.75}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 2.5);

  // Change vector and footprint
  vector = {{0.0, 0.0}, {5.0, 5.0}};
  footprint.outer() = {{-1.0, 1.0}, {4.0, 6.0}, {6.0, 4.0}, {1.0, -1.0}};
  boost::geometry::correct(footprint);  // avoid bugs with malformed polygon
  obstacles.clear();

  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_FALSE(result.has_value());

  obstacles.push_back({{4.0, 4.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, std::sqrt(2 * 4.0 * 4.0));

  obstacles.push_back({{1.0, 2.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.121, 1e-3);

  obstacles.push_back({{-2.0, 2.0}, {3.0, -1.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.354, 1e-3);

  obstacles.push_back({{-1.5, 1.5}, {0.0, 0.5}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.141, 1e-3);

  obstacles.push_back({{0.5, 1.0}, {0.5, -0.5}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.0, 1e-3);

  obstacles.clear();
  obstacles.push_back({{0.5, 1.0}, {0.5, 0.0}, {1.5, 0.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 0.353, 1e-3);

  // Change vector (opposite direction)
  vector = {{5.0, 5.0}, {0.0, 0.0}};
  obstacles.clear();

  obstacles.push_back({{1.0, 1.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, std::sqrt(2 * 4.0 * 4.0));

  obstacles.push_back({{4.0, 3.0}});
  result = distanceToClosestCollision(vector, footprint, obstacles);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 2.121, 1e-3);
}
