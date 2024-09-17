// Copyright 2023 TIER IV, Inc.
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

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"

#include <gtest/gtest.h>

#include <vector>

constexpr double epsilon = 1e-6;

TEST(spline_interpolation, splineYawFromPoints)
{
  using autoware::universe_utils::createPoint;

  {  // straight
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(createPoint(0.0, 0.0, 0.0));
    points.push_back(createPoint(1.0, 1.5, 0.0));
    points.push_back(createPoint(2.0, 3.0, 0.0));
    points.push_back(createPoint(3.0, 4.5, 0.0));
    points.push_back(createPoint(4.0, 6.0, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937, 0.9827937, 0.9827937, 0.9827937};

    const auto yaws = interpolation::spline_yaw_from_points(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(createPoint(-2.0, -10.0, 0.0));
    points.push_back(createPoint(2.0, 1.5, 0.0));
    points.push_back(createPoint(3.0, 3.0, 0.0));
    points.push_back(createPoint(5.0, 10.0, 0.0));
    points.push_back(createPoint(10.0, 12.5, 0.0));

    const std::vector<double> ans{
      1.3681735780957573, 0.96131783759219847, 1.0860982939072894, 0.93835700249513077,
      0.27859394562688311};

    const auto yaws = interpolation::spline_yaw_from_points(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // size of base_keys is 1 (infeasible to interpolate)
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(createPoint(1.0, 0.0, 0.0));

    EXPECT_THROW(interpolation::spline_yaw_from_points(points), std::logic_error);
  }

  {  // straight: size of base_keys is 2 (edge case in the implementation)
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(createPoint(1.0, 0.0, 0.0));
    points.push_back(createPoint(2.0, 1.5, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937};

    const auto yaws = interpolation::spline_yaw_from_points(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: size of base_keys is 3 (edge case in the implementation)
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(createPoint(1.0, 0.0, 0.0));
    points.push_back(createPoint(2.0, 1.5, 0.0));
    points.push_back(createPoint(3.0, 3.0, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937, 0.9827937};

    const auto yaws = interpolation::spline_yaw_from_points(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }
}

TEST(spline_interpolation, SplineInterpolationPoints2d)
{
  using autoware::universe_utils::createPoint;

  // curve
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(createPoint(-2.0, -10.0, 0.0));
  points.push_back(createPoint(2.0, 1.5, 0.0));
  points.push_back(createPoint(3.0, 3.0, 0.0));
  points.push_back(createPoint(5.0, 10.0, 0.0));
  points.push_back(createPoint(10.0, 12.5, 0.0));

  SplineInterpolationPoints2d s(points);

  {  // point
    // front
    const auto front_point = s.compute_point(0, 0.0);
    EXPECT_NEAR(front_point.x, -2.0, epsilon);
    EXPECT_NEAR(front_point.y, -10.0, epsilon);

    // back
    const auto back_point = s.compute_point(4, 0.0);
    EXPECT_NEAR(back_point.x, 10.0, epsilon);
    EXPECT_NEAR(back_point.y, 12.5, epsilon);

    // random
    const auto random_point = s.compute_point(3, 0.5);
    EXPECT_NEAR(random_point.x, 5.28974013, epsilon);
    EXPECT_NEAR(random_point.y, 10.345031918, epsilon);

    // out of range of total length
    const auto front_out_point = s.compute_point(0.0, -0.1);
    EXPECT_NEAR(front_out_point.x, -2.0, epsilon);
    EXPECT_NEAR(front_out_point.y, -10.0, epsilon);

    const auto back_out_point = s.compute_point(4.0, 0.1);
    EXPECT_NEAR(back_out_point.x, 10.0, epsilon);
    EXPECT_NEAR(back_out_point.y, 12.5, epsilon);

    // out of range of index
    EXPECT_THROW((void)s.compute_point(-1, 0.0), std::out_of_range);
    EXPECT_THROW((void)s.compute_point(5, 0.0), std::out_of_range);
  }

  {  // yaw
    // front
    EXPECT_NEAR(s.compute_yaw(0, 0.0), 1.368173578, epsilon);

    // back
    EXPECT_NEAR(s.compute_yaw(4, 0.0), 0.2785939456, epsilon);

    // random
    EXPECT_NEAR(s.compute_yaw(3, 0.5), 0.80858047003, epsilon);

    // out of range of total length
    EXPECT_NEAR(s.compute_yaw(0.0, -0.1), 1.368173578, epsilon);
    EXPECT_NEAR(s.compute_yaw(4, 0.1), 0.2785939456, epsilon);

    // out of range of index
    EXPECT_THROW((void)s.compute_yaw(-1, 0.0), std::out_of_range);
    EXPECT_THROW((void)s.compute_yaw(5, 0.0), std::out_of_range);
  }

  {  // curvature
    // front
    EXPECT_NEAR(s.compute_curvature(0, 0.0), 0.0, epsilon);

    // back
    EXPECT_NEAR(s.compute_curvature(4, 0.0), 0.0, epsilon);

    // random
    EXPECT_NEAR(s.compute_curvature(3, 0.5), -0.271072931448, epsilon);

    // out of range of total length
    EXPECT_NEAR(s.compute_curvature(0.0, -0.1), 0.0, epsilon);
    EXPECT_NEAR(s.compute_curvature(4, 0.1), 0.0, epsilon);

    // out of range of index
    EXPECT_THROW((void)s.compute_curvature(-1, 0.0), std::out_of_range);
    EXPECT_THROW((void)s.compute_curvature(5, 0.0), std::out_of_range);
  }

  {  // accumulated distance
    // front
    EXPECT_NEAR(s.get_accumulated_length(0), 0.0, epsilon);

    // back
    EXPECT_NEAR(s.get_accumulated_length(4), 26.8488511, epsilon);

    // random
    EXPECT_NEAR(s.get_accumulated_length(3), 21.2586811, epsilon);

    // out of range of index
    EXPECT_THROW((void)s.get_accumulated_length(-1), std::out_of_range);
    EXPECT_THROW((void)s.get_accumulated_length(5), std::out_of_range);
  }

  // size of base_keys is 1 (infeasible to interpolate)
  std::vector<geometry_msgs::msg::Point> single_points;
  single_points.push_back(createPoint(1.0, 0.0, 0.0));
  EXPECT_THROW(SplineInterpolationPoints2d{single_points}, std::logic_error);
}

TEST(spline_interpolation, SplineInterpolationPoints2dPolymorphism)
{
  using autoware::universe_utils::createPoint;
  using autoware_planning_msgs::msg::TrajectoryPoint;

  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(createPoint(-2.0, -10.0, 0.0));
  points.push_back(createPoint(2.0, 1.5, 0.0));
  points.push_back(createPoint(3.0, 3.0, 0.0));

  std::vector<TrajectoryPoint> trajectory_points;
  for (const auto & p : points) {
    TrajectoryPoint tp;
    tp.pose.position = p;
    trajectory_points.push_back(tp);
  }

  SplineInterpolationPoints2d s_point(points);
  (void)s_point.compute_point(0, 0.);

  SplineInterpolationPoints2d s_traj_point(trajectory_points);
  (void)s_traj_point.compute_point(0, 0.);
}
