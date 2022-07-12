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

#include "apparent_safe_velocity_limiter/forward_projection.hpp"
#include "apparent_safe_velocity_limiter/types.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/io/wkt/write.hpp>

#include <geometry_msgs/msg/point.h>
#include <gtest/gtest.h>

#include <algorithm>

constexpr auto EPS = 1e-15;
constexpr auto EPS_APPROX = 1e-3;

TEST(TestForwardProjection, forwardSimulatedSegment)
{
  using apparent_safe_velocity_limiter::forwardSimulatedSegment;
  using apparent_safe_velocity_limiter::ProjectionParameters;
  using apparent_safe_velocity_limiter::segment_t;

  geometry_msgs::msg::Point point;
  point.x = 0.0;
  point.y = 0.0;
  ProjectionParameters params;
  params.model = ProjectionParameters::PARTICLE;

  const auto check_vector = [&](const auto expected_vector_length) {
    params.heading = 0.0;
    auto vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_DOUBLE_EQ(vector.second.x(), expected_vector_length);
    EXPECT_DOUBLE_EQ(vector.second.y(), 0.0);
    params.heading = M_PI_2;
    vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_NEAR(vector.second.x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector.second.y(), expected_vector_length);
    params.heading = M_PI_4;
    vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_DOUBLE_EQ(vector.second.x(), std::sqrt(0.5) * expected_vector_length);
    EXPECT_DOUBLE_EQ(vector.second.y(), std::sqrt(0.5) * expected_vector_length);
    params.heading = -M_PI_2;
    vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_NEAR(vector.second.x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector.second.y(), -expected_vector_length);
  };

  // 0 velocity: whatever the duration the vector length is always = to extra_dist
  params.velocity = 0.0;

  params.duration = 0.0;
  params.extra_length = 0.0;
  check_vector(params.extra_length);

  params.duration = 5.0;
  params.extra_length = 2.0;
  check_vector(params.extra_length);

  params.duration = -5.0;
  params.extra_length = 3.5;
  check_vector(params.extra_length);

  // set non-zero velocities
  params.velocity = 1.0;

  params.duration = 1.0;
  params.extra_length = 0.0;
  check_vector(1.0 + params.extra_length);

  params.duration = 5.0;
  params.extra_length = 2.0;
  check_vector(5.0 + params.extra_length);

  params.duration = -5.0;
  params.extra_length = 3.5;
  check_vector(-5.0 + params.extra_length);
}

const auto point_in_polygon = [](const auto x, const auto y, const auto & polygon) {
  return std::find_if(polygon.outer().begin(), polygon.outer().end(), [=](const auto & pt) {
           return pt.x() == x && pt.y() == y;
         }) != polygon.outer().end();
};

TEST(TestForwardProjection, forwardSimulatedFootprint)
{
  using apparent_safe_velocity_limiter::generateFootprint;
  using apparent_safe_velocity_limiter::linestring_t;
  using apparent_safe_velocity_limiter::segment_t;

  auto footprint = generateFootprint(linestring_t{{0.0, 0.0}, {1.0, 0.0}}, 1.0);
  EXPECT_TRUE(point_in_polygon(0.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.0, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));
  footprint = generateFootprint(segment_t{{0.0, 0.0}, {1.0, 0.0}}, 1.0);
  EXPECT_TRUE(point_in_polygon(0.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.0, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));

  footprint = generateFootprint(linestring_t{{0.0, 0.0}, {0.0, -1.0}}, 0.5);
  EXPECT_TRUE(point_in_polygon(0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, -1.0, footprint));
  footprint = generateFootprint(segment_t{{0.0, 0.0}, {0.0, -1.0}}, 0.5);
  EXPECT_TRUE(point_in_polygon(0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, -1.0, footprint));

  footprint = generateFootprint(linestring_t{{-2.5, 5.0}, {2.5, 0.0}}, std::sqrt(2));
  EXPECT_TRUE(point_in_polygon(3.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-3.5, 4.0, footprint));
  EXPECT_TRUE(point_in_polygon(-1.5, 6.0, footprint));
  footprint = generateFootprint(segment_t{{-2.5, 5.0}, {2.5, 0.0}}, std::sqrt(2));
  EXPECT_TRUE(point_in_polygon(3.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-3.5, 4.0, footprint));
  EXPECT_TRUE(point_in_polygon(-1.5, 6.0, footprint));
}

TEST(TestForwardProjection, bicycleProjectionLines)
{
  using apparent_safe_velocity_limiter::bicycleProjectionLines;
  using apparent_safe_velocity_limiter::linestring_t;
  using apparent_safe_velocity_limiter::point_t;
  using apparent_safe_velocity_limiter::ProjectionParameters;

  std::vector<linestring_t> lines;

  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;

  ProjectionParameters params;
  params.model = ProjectionParameters::BICYCLE;
  params.heading = 0.0;
  params.extra_length = 0.0;
  params.points_per_projection = 2;
  params.wheel_base = 1.0;
  params.steering_angle = 0.0;
  params.steering_angle_offsets = {0.0};

  const auto assert_zero_projection = [&]() {
    lines = bicycleProjectionLines(origin, params);
    ASSERT_EQ(lines.size(), 1ul);
    ASSERT_EQ(lines[0].size(), static_cast<size_t>(params.points_per_projection));
    EXPECT_DOUBLE_EQ(lines[0][0].x(), origin.x);
    EXPECT_DOUBLE_EQ(lines[0][0].y(), origin.y);
    for (size_t i = 0; i + 1 < lines[0].size(); ++i) {
      EXPECT_DOUBLE_EQ(lines[0][i].x(), lines[0][i + 1].x());
      EXPECT_DOUBLE_EQ(lines[0][i].y(), lines[0][i + 1].y());
    }
  };

  // Test 0 duration or 0 velocity -> no projection
  for (params.points_per_projection = 2; params.points_per_projection < 10;
       ++params.points_per_projection) {
    params.duration = 0.0;
    for (params.velocity = 0.0; params.velocity < 10.0; ++params.velocity) assert_zero_projection();
    params.velocity = 0.0;
    for (params.duration = 0.0; params.duration < 10.0; ++params.duration) assert_zero_projection();
  }

  // Projections with no heading and no steering
  params.points_per_projection = 2;
  params.duration = 1.0;
  params.velocity = 1.0;
  params.heading = 0.0;
  params.steering_angle = 0.0;
  params.steering_angle_offsets = {0.0};
  lines = bicycleProjectionLines(origin, params);
  ASSERT_EQ(lines.size(), 1ul);
  ASSERT_EQ(lines[0].size(), 2ul);
  EXPECT_DOUBLE_EQ(lines[0][0].x(), origin.x);
  EXPECT_DOUBLE_EQ(lines[0][0].y(), origin.y);
  EXPECT_DOUBLE_EQ(lines[0][1].x(), 1.0);
  EXPECT_DOUBLE_EQ(lines[0][1].y(), 0.0);

  params.heading = M_PI_2;
  lines = bicycleProjectionLines(origin, params);
  ASSERT_EQ(lines.size(), 1ul);
  ASSERT_EQ(lines[0].size(), 2ul);
  EXPECT_NEAR(lines[0][0].x(), origin.x, EPS);
  EXPECT_NEAR(lines[0][0].y(), origin.y, EPS);
  EXPECT_NEAR(lines[0][1].x(), 0.0, EPS);
  EXPECT_NEAR(lines[0][1].y(), 1.0, EPS);

  params.heading = M_PI;
  lines = bicycleProjectionLines(origin, params);
  ASSERT_EQ(lines.size(), 1ul);
  ASSERT_EQ(lines[0].size(), 2ul);
  EXPECT_NEAR(lines[0][0].x(), origin.x, EPS);
  EXPECT_NEAR(lines[0][0].y(), origin.y, EPS);
  EXPECT_NEAR(lines[0][1].x(), -1.0, EPS);
  EXPECT_NEAR(lines[0][1].y(), 0.0, EPS);

  params.heading = 0.0;
  params.steering_angle = 0.1;
  lines = bicycleProjectionLines(origin, params);
  ASSERT_EQ(lines.size(), 1ul);
  ASSERT_EQ(lines[0].size(), 2ul);
  EXPECT_NEAR(lines[0][0].x(), origin.x, EPS);
  EXPECT_NEAR(lines[0][0].y(), origin.y, EPS);
  EXPECT_NEAR(lines[0][1].x(), 0.995, EPS_APPROX);
  EXPECT_NEAR(lines[0][1].y(), 0.1001, EPS_APPROX);

  // set an offset
  params.steering_angle_offsets = {0.0, -0.1, 0.1};
  params.steering_angle = 0.1;
  lines = bicycleProjectionLines(origin, params);
  ASSERT_EQ(lines.size(), 3ul);
  for (const auto & line : lines) {
    ASSERT_EQ(line.size(), 2ul);
    EXPECT_NEAR(line[0].x(), origin.x, EPS);
    EXPECT_NEAR(line[0].y(), origin.y, EPS);
  }
  EXPECT_NEAR(lines[0][1].x(), 0.995, EPS_APPROX);
  EXPECT_NEAR(lines[0][1].y(), 0.1001, EPS_APPROX);
  EXPECT_NEAR(lines[1][1].x(), 1.0, EPS_APPROX);
  EXPECT_NEAR(lines[1][1].y(), 0.0, EPS_APPROX);
  EXPECT_NEAR(lines[2][1].x(), 0.9797, EPS_APPROX);
  EXPECT_NEAR(lines[2][1].y(), 0.2005, EPS_APPROX);
}

TEST(TestForwardProjection, bicycleProjectionFootprint)
{
  using apparent_safe_velocity_limiter::forwardSimulatedPolygon;
  using apparent_safe_velocity_limiter::point_t;
  using apparent_safe_velocity_limiter::polygon_t;
  using apparent_safe_velocity_limiter::ProjectionParameters;
  using apparent_safe_velocity_limiter::segment_t;

  polygon_t footprint;
  segment_t segment;

  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;

  double lateral_offset = 1.0;
  ProjectionParameters params;
  params.model = ProjectionParameters::BICYCLE;
  params.heading = 0.0;
  params.extra_length = 0.0;
  params.points_per_projection = 2;
  params.wheel_base = 1.0;
  params.steering_angle = 0.0;

  params.velocity = 0.0;
  params.duration = 0.0;
  params.steering_angle_offsets = {0.0};

  // no projection (0vel) -> square polygon
  footprint = forwardSimulatedPolygon(origin, params, lateral_offset, segment);
  EXPECT_EQ(footprint.outer().size(), 5ul);  // first == last
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-1.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-1.0, -1.0, footprint));

  params.velocity = 1.0;
  params.duration = 1.0;
  params.steering_angle_offsets = {0.0};
  footprint = forwardSimulatedPolygon(origin, params, lateral_offset, segment);
  EXPECT_EQ(footprint.outer().size(), 6ul);  // for some reason bg::buffer puts a point twice
  EXPECT_TRUE(point_in_polygon(0.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.0, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));

  params.points_per_projection = 10;
  params.steering_angle_offsets = {1.0};
  footprint = forwardSimulatedPolygon(origin, params, lateral_offset, segment);
  std::cout << boost::geometry::wkt(footprint) << std::endl;

  params.steering_angle_offsets = {-1.0};
  footprint = forwardSimulatedPolygon(origin, params, lateral_offset, segment);
  std::cout << boost::geometry::wkt(footprint) << std::endl;

  params.steering_angle_offsets = {-1.0, 0.0, 1.0};
  footprint = forwardSimulatedPolygon(origin, params, lateral_offset, segment);
  std::cout << boost::geometry::wkt(footprint) << std::endl;
}
