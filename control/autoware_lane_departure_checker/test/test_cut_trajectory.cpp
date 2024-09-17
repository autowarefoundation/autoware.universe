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

#include "autoware/lane_departure_checker/utils.hpp"

#include <Eigen/Core>

#include <gtest/gtest.h>

using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

TrajectoryPoints create_trajectory(const std::vector<Eigen::Vector3d> & points)
{
  TrajectoryPoints trajectory;
  for (const auto & point : points) {
    TrajectoryPoint p;
    p.pose.position.x = point.x();
    p.pose.position.y = point.y();
    p.pose.position.z = point.z();
    trajectory.push_back(p);
  }
  return trajectory;
}

class CutTrajectoryTest
: public ::testing::TestWithParam<
    std::tuple<std::vector<Eigen::Vector3d>, double, std::vector<Eigen::Vector3d>>>
{
};

TEST_P(CutTrajectoryTest, test_cut_trajectory)
{
  const auto [trajectory_points, length, expected_points] = GetParam();
  const auto trajectory = create_trajectory(trajectory_points);
  const auto cut = autoware::lane_departure_checker::utils::cutTrajectory(trajectory, length);

  ASSERT_EQ(cut.size(), expected_points.size());

  for (size_t i = 0; i < expected_points.size(); ++i) {
    EXPECT_DOUBLE_EQ(cut[i].pose.position.x, expected_points[i].x());
    EXPECT_DOUBLE_EQ(cut[i].pose.position.y, expected_points[i].y());
    EXPECT_DOUBLE_EQ(cut[i].pose.position.z, expected_points[i].z());
  }
}

INSTANTIATE_TEST_SUITE_P(
  CutTrajectoryTests, CutTrajectoryTest,
  ::testing::Values(
    // Empty trajectory
    std::make_tuple(std::vector<Eigen::Vector3d>{}, 1.0, std::vector<Eigen::Vector3d>{}),

    // Single point trajectory
    std::make_tuple(
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}}, 1.0,
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}}),

    // Interpolation at the viapoint of the trajectory
    std::make_tuple(
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, 1.0,
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}),

    // Interpolation in the middle of the trajectory
    std::make_tuple(
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, 1.5,
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.5, 0.0, 0.0}}),

    // No interpolation
    std::make_tuple(
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, 3.0,
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}})));
