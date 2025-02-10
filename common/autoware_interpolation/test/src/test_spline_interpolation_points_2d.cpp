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

#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

constexpr double epsilon = 1e-6;

using autoware::interpolation::SplineInterpolationPoints2d;

TEST(spline_interpolation, splineYawFromPoints)
{
  using autoware_utils::create_point;

  {  // straight
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(0.0, 0.0, 0.0));
    points.push_back(create_point(1.0, 1.5, 0.0));
    points.push_back(create_point(2.0, 3.0, 0.0));
    points.push_back(create_point(3.0, 4.5, 0.0));
    points.push_back(create_point(4.0, 6.0, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937, 0.9827937, 0.9827937, 0.9827937};

    const auto yaws = autoware::interpolation::splineYawFromPoints(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(-2.0, -10.0, 0.0));
    points.push_back(create_point(2.0, 1.5, 0.0));
    points.push_back(create_point(3.0, 3.0, 0.0));
    points.push_back(create_point(5.0, 10.0, 0.0));
    points.push_back(create_point(10.0, 12.5, 0.0));

    const std::vector<double> ans{1.368174, 0.961318, 1.086098, 0.938357, 0.278594};
    const auto yaws = autoware::interpolation::splineYawFromPoints(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // size of base_keys is 1 (infeasible to interpolate)
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(1.0, 0.0, 0.0));

    EXPECT_THROW(autoware::interpolation::splineYawFromPoints(points), std::logic_error);
  }

  {  // straight: size of base_keys is 2 (edge case in the implementation)
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(1.0, 0.0, 0.0));
    points.push_back(create_point(2.0, 1.5, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937};

    const auto yaws = autoware::interpolation::splineYawFromPoints(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: size of base_keys is 3 (edge case in the implementation)
    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(create_point(1.0, 0.0, 0.0));
    points.push_back(create_point(2.0, 1.5, 0.0));
    points.push_back(create_point(3.0, 3.0, 0.0));

    const std::vector<double> ans{0.9827937, 0.9827937, 0.9827937};

    const auto yaws = autoware::interpolation::splineYawFromPoints(points);
    for (size_t i = 0; i < yaws.size(); ++i) {
      EXPECT_NEAR(yaws.at(i), ans.at(i), epsilon);
    }
  }
}

TEST(spline_interpolation, SplineInterpolationPoints2d)
{
  using autoware_utils::create_point;

  // curve
  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(-2.0, -10.0, 0.0));
  points.push_back(create_point(2.0, 1.5, 0.0));
  points.push_back(create_point(3.0, 3.0, 0.0));
  points.push_back(create_point(5.0, 10.0, 0.0));
  points.push_back(create_point(10.0, 12.5, 0.0));

  SplineInterpolationPoints2d s(points);

  {  // point
    // front
    const auto front_point = s.getSplineInterpolatedPoint(0, 0.0);
    EXPECT_NEAR(front_point.x, -2.0, epsilon);
    EXPECT_NEAR(front_point.y, -10.0, epsilon);

    // back
    const auto back_point = s.getSplineInterpolatedPoint(4, 0.0);
    EXPECT_NEAR(back_point.x, 10.0, epsilon);
    EXPECT_NEAR(back_point.y, 12.5, epsilon);

    // random
    const auto random_point = s.getSplineInterpolatedPoint(3, 0.5);
    EXPECT_NEAR(random_point.x, 5.28974, epsilon);
    EXPECT_NEAR(random_point.y, 10.3450319, epsilon);

    // out of range of total length
    const auto front_out_point = s.getSplineInterpolatedPoint(0.0, -0.1);
    EXPECT_NEAR(front_out_point.x, -2.0, epsilon);
    EXPECT_NEAR(front_out_point.y, -10.0, epsilon);

    const auto back_out_point = s.getSplineInterpolatedPoint(4.0, 0.1);
    EXPECT_NEAR(back_out_point.x, 10.0, epsilon);
    EXPECT_NEAR(back_out_point.y, 12.5, epsilon);

    // out of range of index
    EXPECT_THROW(s.getSplineInterpolatedPoint(-1, 0.0), std::out_of_range);
    EXPECT_THROW(s.getSplineInterpolatedPoint(5, 0.0), std::out_of_range);
  }

  {  // yaw
    // front
    EXPECT_NEAR(s.getSplineInterpolatedYaw(0, 0.0), 1.368174, epsilon);

    // back
    EXPECT_NEAR(s.getSplineInterpolatedYaw(4, 0.0), 0.278594, epsilon);

    // random
    EXPECT_NEAR(s.getSplineInterpolatedYaw(3, 0.5), 0.808580, epsilon);

    // out of range of total length
    EXPECT_NEAR(s.getSplineInterpolatedYaw(0.0, -0.1), 1.368174, epsilon);
    EXPECT_NEAR(s.getSplineInterpolatedYaw(4, 0.1), 0.278594, epsilon);

    // out of range of index
    EXPECT_THROW(s.getSplineInterpolatedYaw(-1, 0.0), std::out_of_range);
    EXPECT_THROW(s.getSplineInterpolatedYaw(5, 0.0), std::out_of_range);
  }

  {  // curvature
    // front
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(0, 0.0), 0.0, epsilon);

    // back
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(4, 0.0), 0.0, epsilon);

    // random
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(3, 0.5), -0.271073, epsilon);

    // out of range of total length
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(0.0, -0.1), 0.0, epsilon);
    EXPECT_NEAR(s.getSplineInterpolatedCurvature(4, 0.1), 0.0, epsilon);

    // out of range of index
    EXPECT_THROW(s.getSplineInterpolatedCurvature(-1, 0.0), std::out_of_range);
    EXPECT_THROW(s.getSplineInterpolatedCurvature(5, 0.0), std::out_of_range);
  }

  {  // accumulated distance
    // front
    EXPECT_NEAR(s.getAccumulatedLength(0), 0.0, epsilon);

    // back
    EXPECT_NEAR(s.getAccumulatedLength(4), 26.8488511, epsilon);

    // random
    EXPECT_NEAR(s.getAccumulatedLength(3), 21.2586811, epsilon);

    // out of range of index
    EXPECT_THROW(s.getAccumulatedLength(-1), std::out_of_range);
    EXPECT_THROW(s.getAccumulatedLength(5), std::out_of_range);
  }

  // size of base_keys is 1 (infeasible to interpolate)
  std::vector<geometry_msgs::msg::Point> single_points;
  single_points.push_back(create_point(1.0, 0.0, 0.0));
  EXPECT_THROW(SplineInterpolationPoints2d{single_points}, std::logic_error);
}

TEST(spline_interpolation, SplineInterpolationPoints2dPolymorphism)
{
  using autoware_planning_msgs::msg::TrajectoryPoint;
  using autoware_utils::create_point;

  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(create_point(-2.0, -10.0, 0.0));
  points.push_back(create_point(2.0, 1.5, 0.0));
  points.push_back(create_point(3.0, 3.0, 0.0));

  std::vector<TrajectoryPoint> trajectory_points;
  for (const auto & p : points) {
    TrajectoryPoint tp;
    tp.pose.position = p;
    trajectory_points.push_back(tp);
  }

  SplineInterpolationPoints2d s_point(points);
  s_point.getSplineInterpolatedPoint(0, 0.);

  SplineInterpolationPoints2d s_traj_point(trajectory_points);
  s_traj_point.getSplineInterpolatedPoint(0, 0.);
}
