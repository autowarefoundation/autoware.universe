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
  using safe_velocity_adjustor::point_t;
  using safe_velocity_adjustor::segment_t;
  segment_t vector = {point_t{0, 0}, point_t{5, 0}};
  const auto vehicle_width = 2.0;
  pcl::PointCloud<pcl::PointXYZ> obstacle_points;
  obstacle_points.points.emplace_back(6, 2, 0);  // outside of the footprint
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_FALSE(dist.has_value());
  }
  obstacle_points.points.emplace_back(4, 0, 0);  // distance of 4
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 4.0);
  }
  obstacle_points.points.emplace_back(
    4.5, 0, 0);  // distance of 4.5, does not change the minimum distance
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 4.0);
  }
  obstacle_points.points.emplace_back(2.0, 0.5, 0);  // new minumum distance of 2.0
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 2.0);
  }
  obstacle_points.points.emplace_back(1.5, -0.75, 0);  // new minumum distance of 1.5
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 1.5);
  }

  // Change vector heading
  vector = {point_t{0, 0}, point_t{0, 5}};
  obstacle_points.clear();
  obstacle_points.emplace_back(-2, 3, 0);  // outside of the footprint
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_FALSE(dist.has_value());
  }
  obstacle_points.points.emplace_back(-0.5, 4, 0);  // new minumum distance of 4
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 4);
  }
  obstacle_points.points.emplace_back(0, 4, 0);  // no change in the minimum distance
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 4);
  }
  obstacle_points.points.emplace_back(0.5, 0.5, 0);  // change the minimum distance
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, 0.5);
  }

  // Change vector
  vector = {point_t{0, 0}, point_t{2.5, 2.5}};
  obstacle_points.clear();
  obstacle_points.emplace_back(3, 3, 0);  // outside of the footprint
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_FALSE(dist.has_value());
  }
  obstacle_points.points.emplace_back(2.25, 1.75, 0);  // new minumum distance
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, std::sqrt(8));
  }
  obstacle_points.points.emplace_back(2, 2, 0);  // no change in the minimum distance
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, std::sqrt(8));
  }
  obstacle_points.points.emplace_back(0.25, 0.75, 0);  // change the minimum distance
  {
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    ASSERT_TRUE(dist.has_value());
    EXPECT_DOUBLE_EQ(*dist, std::sqrt(0.5));
  }
}

TEST(TestCollisionDistance, distanceToClosestCollisionBench)
{
  using pcl::common::CloudGenerator;
  using pcl::common::UniformGenerator;
  using safe_velocity_adjustor::distanceToClosestCollision;
  using safe_velocity_adjustor::distanceToClosestCollision_Eigen;
  safe_velocity_adjustor::segment_t vector = {
    safe_velocity_adjustor::point_t{0, 0}, safe_velocity_adjustor::point_t{5, 0}};
  const auto vehicle_width = 2.0;
  tier4_autoware_utils::StopWatch watch;

  pcl::PointCloud<pcl::PointXYZ> obstacle_points;
  constexpr auto size = 10000;
  constexpr auto min = -5.0;
  constexpr auto max = 15.0;
  CloudGenerator<pcl::PointXYZ, UniformGenerator<float> > generator;
  double gen{};
  double baseline{};
  double eigen{};
  for (auto i = 0; i < 100; ++i) {
    watch.tic("gen");
    obstacle_points.clear();
    UniformGenerator<float>::Parameters x_params(min, max, i);
    generator.setParametersForX(x_params);
    UniformGenerator<float>::Parameters y_params(min, max, i);
    generator.setParametersForY(y_params);
    UniformGenerator<float>::Parameters z_params(min, max, i);
    generator.setParametersForZ(z_params);
    generator.fill(size, 1, obstacle_points);
    gen += watch.toc("gen");
    watch.tic("baseline");
    const auto dist = distanceToClosestCollision(vector, vehicle_width, obstacle_points);
    baseline += watch.toc("baseline");
    watch.tic("eigen");
    const auto dist_eigen =
      distanceToClosestCollision_Eigen(vector, vehicle_width, obstacle_points);
    eigen += watch.toc("eigen");
    if (dist && dist_eigen)
      EXPECT_EQ(*dist, *dist_eigen);
    else
      EXPECT_EQ(dist.has_value(), dist_eigen.has_value());
  }
  std::cerr << "Cloud gen: " << gen << std::endl;
  std::cerr << "Baseline : " << baseline << std::endl;
  std::cerr << "Eigen: " << eigen << std::endl;

  std::cerr << "\na: " << safe_velocity_adjustor::a << std::endl;
  std::cerr << "b: " << safe_velocity_adjustor::b << std::endl;
  std::cerr << "c: " << safe_velocity_adjustor::c << std::endl;
  std::cerr << "d: " << safe_velocity_adjustor::d << std::endl;
  std::cerr << "e: " << safe_velocity_adjustor::e << std::endl;
  std::cerr << "f: " << safe_velocity_adjustor::f << std::endl;
}
