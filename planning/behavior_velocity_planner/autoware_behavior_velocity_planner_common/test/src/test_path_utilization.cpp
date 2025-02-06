// Copyright 2021 Tier IV, Inc.
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

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <vector>

#define DEBUG_PRINT_PATH(path)                                                        \
  {                                                                                   \
    std::stringstream ss;                                                             \
    ss << #path << "(px, vx): ";                                                      \
    for (const auto p : path.points) {                                                \
      ss << "(" << p.pose.position.x << ", " << p.longitudinal_velocity_mps << "), "; \
    }                                                                                 \
    std::cerr << ss.str() << std::endl;                                               \
  }

TEST(is_ahead_of, nominal)
{
  using autoware::behavior_velocity_planner::planning_utils::isAheadOf;
  geometry_msgs::msg::Pose target = test::generatePose(0);
  geometry_msgs::msg::Pose origin = test::generatePose(1);
  bool is_ahead = isAheadOf(target, origin);
  EXPECT_FALSE(is_ahead);
  target = test::generatePose(2);
  is_ahead = isAheadOf(target, origin);
  EXPECT_TRUE(is_ahead);
}

TEST(smoothDeceleration, calculateMaxSlowDownVelocity)
{
  using autoware::behavior_velocity_planner::planning_utils::
    calcDecelerationVelocityFromDistanceToTarget;
  const double current_accel = 1.0;
  const double current_velocity = 5.0;
  const double max_slow_down_jerk = -1.0;
  const double max_slow_down_accel = -2.0;
  const double eps = 1e-3;
  {
    for (int i = -8; i <= 24; i += 8) {
      // arc length in path point
      const double l = i * 1.0;
      const double v = calcDecelerationVelocityFromDistanceToTarget(
        max_slow_down_jerk, max_slow_down_accel, current_accel, current_velocity, l);
      // case 0 : behind ego
      if (i == -8) EXPECT_NEAR(v, 5.0, eps);
      // case 1 : const jerk
      else if (i == 0)
        EXPECT_NEAR(v, 5.0, eps);
      // case 1 : const jerk
      else if (i == 8)
        EXPECT_NEAR(v, 5.380, eps);
      // case 2 : const accel
      else if (i == 16)
        EXPECT_NEAR(v, 2.872, eps);
      // case 3 : after stop
      else if (i == 24)
        EXPECT_NEAR(v, 0.00, eps);
      else
        continue;
      std::cout << "s: " << l << " v: " << v << std::endl;
    }
  }
}

TEST(specialInterpolation, specialInterpolation)
{
  using autoware::behavior_velocity_planner::interpolatePath;
  using autoware::motion_utils::calcSignedArcLength;
  using autoware::motion_utils::searchZeroVelocityIndex;
  using autoware_planning_msgs::msg::Path;
  using autoware_planning_msgs::msg::PathPoint;

  const auto genPath = [](const auto p, const auto v) {
    if (p.size() != v.size()) throw std::invalid_argument("different size is not expected");
    Path path;
    for (size_t i = 0; i < p.size(); ++i) {
      PathPoint pp;
      pp.pose.position.x = p.at(i);
      pp.longitudinal_velocity_mps = v.at(i);
      path.points.push_back(pp);
    }
    return path;
  };

  constexpr auto length = 5.0;
  constexpr auto interval = 1.0;

  const auto calcInterpolatedStopDist = [&](const auto & px, const auto & vx) {
    const auto path = genPath(px, vx);
    const auto res = interpolatePath(path, length, interval);
    // DEBUG_PRINT_PATH(path);
    // DEBUG_PRINT_PATH(res);
    return calcSignedArcLength(res.points, 0, *searchZeroVelocityIndex(res.points));
  };

  // expected stop position: s=2.0
  {
    const std::vector<double> px{0.0, 1.0, 2.0, 3.0};
    const std::vector<double> vx{5.5, 5.5, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), 2.0);
  }

  // expected stop position: s=2.1
  {
    constexpr auto expected = 2.1;
    const std::vector<double> px{0.0, 1.0, 2.1, 3.0};
    const std::vector<double> vx{5.5, 5.5, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=2.001
  {
    constexpr auto expected = 2.001;
    const std::vector<double> px{0.0, 1.0, 2.001, 3.0};
    const std::vector<double> vx{5.5, 5.5, 0.000, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=2.001
  {
    constexpr auto expected = 2.001;
    const std::vector<double> px{0.0, 1.0, 1.999, 2.0, 2.001, 3.0};
    const std::vector<double> vx{5.5, 5.5, 5.555, 5.5, 0.000, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=2.0
  {
    constexpr auto expected = 2.0;
    const std::vector<double> px{0.0, 1.0, 1.999, 2.0, 2.001, 3.0};
    const std::vector<double> vx{5.5, 5.5, 5.555, 0.0, 0.000, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=1.999
  {
    constexpr auto expected = 1.999;
    const std::vector<double> px{0.0, 1.0, 1.999, 3.0};
    const std::vector<double> vx{5.5, 5.5, 0.000, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=0.2
  {
    constexpr auto expected = 0.2;
    const std::vector<double> px{0.0, 0.1, 0.2, 0.3, 0.4};
    const std::vector<double> vx{5.5, 5.5, 0.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=0.4
  {
    constexpr auto expected = 0.4;
    const std::vector<double> px{0.0, 0.1, 0.2, 0.3, 0.4};
    const std::vector<double> vx{5.5, 5.5, 5.5, 5.5, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }
}

TEST(filterLitterPathPoint, nominal)
{
  using autoware::behavior_velocity_planner::filterLitterPathPoint;
  using autoware_planning_msgs::msg::Path;
  using autoware_planning_msgs::msg::PathPoint;

  const auto genPath = [](const std::vector<double> & px, const std::vector<double> & vx) {
    Path path;
    for (size_t i = 0; i < px.size(); ++i) {
      PathPoint point;
      point.pose.position.x = px[i];
      point.pose.position.y = 0.0;
      point.longitudinal_velocity_mps = static_cast<float>(vx[i]);
      path.points.push_back(point);
    }
    return path;
  };

  const std::vector<double> px{0.0, 1.0, 1.001, 2.0, 3.0};
  const std::vector<double> vx{5.0, 3.5, 3.5, 3.0, 2.5};

  const auto path = genPath(px, vx);
  const auto filtered_path = filterLitterPathPoint(path);

  ASSERT_EQ(filtered_path.points.size(), 4U);  // Expected: Points at x = {0.0, 1.0, 2.0, 3.0}
  EXPECT_DOUBLE_EQ(filtered_path.points[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(filtered_path.points[1].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(filtered_path.points[2].pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(filtered_path.points[3].pose.position.x, 3.0);

  EXPECT_DOUBLE_EQ(filtered_path.points[0].longitudinal_velocity_mps, 5.0);
  EXPECT_DOUBLE_EQ(filtered_path.points[1].longitudinal_velocity_mps, 3.5);
  EXPECT_DOUBLE_EQ(filtered_path.points[2].longitudinal_velocity_mps, 3.0);
  EXPECT_DOUBLE_EQ(filtered_path.points[3].longitudinal_velocity_mps, 2.5);
}

TEST(filterStopPathPoint, nominal)
{
  using autoware::behavior_velocity_planner::filterStopPathPoint;
  using autoware_planning_msgs::msg::Path;
  using autoware_planning_msgs::msg::PathPoint;

  const auto genPath = [](const std::vector<double> & px, const std::vector<double> & vx) {
    Path path;
    for (size_t i = 0; i < px.size(); ++i) {
      PathPoint point;
      point.pose.position.x = px[i];
      point.longitudinal_velocity_mps = static_cast<float>(vx[i]);
      path.points.push_back(point);
    }
    return path;
  };

  const std::vector<double> px{0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<double> vx{5.0, 4.0, 0.0, 2.0, 3.0};

  const auto path = genPath(px, vx);
  const auto filtered_path = filterStopPathPoint(path);

  ASSERT_EQ(filtered_path.points.size(), 5U);
  EXPECT_DOUBLE_EQ(filtered_path.points[0].longitudinal_velocity_mps, 5.0);
  EXPECT_DOUBLE_EQ(filtered_path.points[1].longitudinal_velocity_mps, 4.0);
  EXPECT_DOUBLE_EQ(filtered_path.points[2].longitudinal_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(filtered_path.points[3].longitudinal_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(filtered_path.points[4].longitudinal_velocity_mps, 0.0);
}
