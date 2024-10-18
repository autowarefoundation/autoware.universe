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

#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

using tier4_planning_msgs::msg::PathWithLaneId;

using autoware::test_utils::simple_path_with_lane_id;

TEST(BehaviorPathPlanningPathUtilTest, calcPathArcLengthArray)
{
  using autoware::behavior_path_planner::utils::calcPathArcLengthArray;

  auto path = simple_path_with_lane_id(10, 1.0);
  auto arc_length_array = calcPathArcLengthArray(path);

  ASSERT_EQ(arc_length_array.size(), path.points.size());

  size_t i = 0;
  for (const auto & arc_length : arc_length_array) {
    EXPECT_DOUBLE_EQ(arc_length, 1.0 * i);
    i++;
  }
}

TEST(BehaviorPathPlanningPathUtilTest, resamplePathWithSpline)
{
  using autoware::behavior_path_planner::utils::resamplePathWithSpline;

  // Condition: path point less than 2
  auto path = simple_path_with_lane_id(1, 1.0);
  EXPECT_EQ(resamplePathWithSpline(path, 1.0).points.size(), path.points.size());

  // Condition: not enough point for spline interpolation
  path = simple_path_with_lane_id(10, 0.01);
  EXPECT_EQ(resamplePathWithSpline(path, 1.0).points.size(), path.points.size());

  // Condition: spline interpolation
  path = simple_path_with_lane_id(10, 0.1);
  auto resampled_path = resamplePathWithSpline(path, 1.0);
  EXPECT_EQ(resampled_path.points.size(), 6);
}

TEST(BehaviorPathPlanningPathUtilTest, getIdxByArclength)
{
  using autoware::behavior_path_planner::utils::getIdxByArclength;

  tier4_planning_msgs::msg::PathWithLaneId path;

  // Condition: empty points
  EXPECT_ANY_THROW(getIdxByArclength(path, 5, 1.0));

  // Condition: negative arc with idx 0
  path = simple_path_with_lane_id(10, 1.0);
  EXPECT_EQ(getIdxByArclength(path, 0, -1.0), 0);

  // Condition: negative arc
  EXPECT_EQ(getIdxByArclength(path, 5, -2.0), 2);

  // Condition: positive arc
  EXPECT_EQ(getIdxByArclength(path, 3, 4.0), 8);
}

TEST(BehaviorPathPlanningPathUtilTest, clipPathLength)
{
  using autoware::behavior_path_planner::utils::clipPathLength;

  // Condition: path point less than 3
  auto path = simple_path_with_lane_id(2, 1.0);
  clipPathLength(path, 5, 10.0, 1.0);
  EXPECT_EQ(path.points.size(), 2);

  // Condition: path to be cropped
  path = simple_path_with_lane_id(10, 1.0);
  clipPathLength(path, 5, 10.0, 1.0);
  EXPECT_EQ(path.points.size(), 7);

  size_t i = 0;
  for (const auto & point : path.points) {
    EXPECT_DOUBLE_EQ(point.point.pose.position.x, 3.0 + i);
    i++;
  }

  // Condition: using parameters
  BehaviorPathPlannerParameters param;
  param.forward_path_length = 20.0;
  param.backward_path_length = 1.0;
  path = simple_path_with_lane_id(10, 1.0);
  clipPathLength(path, 5, param);
  EXPECT_EQ(path.points.size(), 7);

  i = 0;
  for (const auto & point : path.points) {
    EXPECT_DOUBLE_EQ(point.point.pose.position.x, 3.0 + i);
    i++;
  }
}

TEST(BehaviorPathPlanningPathUtilTest, getReversingIndices)
{
  using autoware::behavior_path_planner::utils::getReversingIndices;

  size_t target_index = 7;
  auto path = simple_path_with_lane_id(10, 1.0, 1.0);
  path.points.at(target_index).point.longitudinal_velocity_mps = -1.0;
  auto reversing_indices = getReversingIndices(path);
  EXPECT_EQ(reversing_indices.size(), 2);

  size_t i = 0;
  for (const auto & index : reversing_indices) {
    EXPECT_EQ(index, target_index - 1 + i);
    i++;
  }
}

TEST(BehaviorPathPlanningPathUtilTest, dividePath)
{
  using autoware::behavior_path_planner::utils::dividePath;
  auto path = simple_path_with_lane_id(10, 1.0);

  // Condition: empty indices
  std::vector<size_t> indices;
  auto divided_path = dividePath(path, indices);
  EXPECT_EQ(divided_path.size(), 1);

  // Condition: divide path
  indices = {3, 5, 8};
  divided_path = dividePath(path, indices);
  ASSERT_EQ(divided_path.size(), 4);
  EXPECT_EQ(divided_path.at(0).points.size(), 4);
  EXPECT_EQ(divided_path.at(1).points.size(), 3);
  EXPECT_EQ(divided_path.at(2).points.size(), 4);
  EXPECT_EQ(divided_path.at(3).points.size(), 2);
}

TEST(BehaviorPathPlanningPathUtilTest, correctDividedPathVelocity)
{
  using autoware::behavior_path_planner::utils::correctDividedPathVelocity;

  double velocity = 1.0;
  std::vector<PathWithLaneId> divided_paths;
  // forward driving
  divided_paths.push_back(simple_path_with_lane_id(10, 1.0, velocity));
  // reverse driving
  divided_paths.push_back(simple_path_with_lane_id(10, -1.0, velocity, M_PI));
  correctDividedPathVelocity(divided_paths);

  for (const auto & point : divided_paths.at(0).points) {
    if (point == divided_paths.at(0).points.back()) {
      EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps, 0.0);
    } else {
      EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps, velocity);
    }
  }

  for (const auto & point : divided_paths.at(1).points) {
    if (point == divided_paths.at(1).points.back()) {
      EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps, 0.0);
    } else {
      EXPECT_DOUBLE_EQ(point.point.longitudinal_velocity_mps, -velocity);
    }
  }
}

TEST(BehaviorPathPlanningPathUtilTest, spline_two_points)
{
  using autoware::behavior_path_planner::utils::spline_two_points;

  std::vector<double> base_s;
  std::vector<double> base_x;
}
