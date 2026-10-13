// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/map_velocity_limits.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <rclcpp/duration.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>

namespace
{
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_modifier::plugin::detail::apply_velocity_limits;
using autoware::trajectory_modifier::plugin::detail::VelocityLimitConstraints;
using autoware::trajectory_modifier::plugin::detail::VelocityLimitOptions;
constexpr double dt = 0.1;
constexpr double tolerance = 2e-5;
constexpr VelocityLimitConstraints constraints{1.0, 1.0, 1.0};

TrajectoryPoints make_trajectory(const double speed = 10.0, const double acceleration = 0.0)
{
  TrajectoryPoints points(80);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double t = (i + 1) * dt;
    auto & point = points[i];
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    point.pose.position.x = speed * t + 0.5 * acceleration * t * t;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(speed + acceleration * t);
    point.acceleration_mps2 = static_cast<float>(acceleration);
  }
  return points;
}

auto constant_limit(const double speed)
{
  return [speed](const geometry_msgs::msg::Point &) { return std::optional<double>{speed}; };
}

void expect_format(const TrajectoryPoints & input, const TrajectoryPoints & output)
{
  ASSERT_EQ(output.size(), 80U);
  EXPECT_EQ(output.front().pose, input.front().pose);
  for (std::size_t i = 0; i < output.size(); ++i) {
    EXPECT_EQ(output[i].time_from_start, input[i].time_from_start);
  }
}

void expect_straight_motion(const TrajectoryPoints & points)
{
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double v0 = points[i].longitudinal_velocity_mps;
    const double v1 = points[i + 1].longitudinal_velocity_mps;
    EXPECT_NEAR(
      points[i + 1].pose.position.x - points[i].pose.position.x, 0.5 * (v0 + v1) * dt, 1e-8)
      << i;
    EXPECT_NEAR(points[i].acceleration_mps2, (v1 - v0) / dt, tolerance) << i;
    if (points[i + 1].pose.position.x == points[i].pose.position.x) {
      EXPECT_FLOAT_EQ(v0, 0.0F) << i;
      EXPECT_FLOAT_EQ(v1, 0.0F) << i;
    }
  }
  EXPECT_FLOAT_EQ(points.back().acceleration_mps2, 0.0F);
}

TEST(MapVelocityLimitsProfile, RecomputesAccelerationAndRetimes)
{
  auto points = make_trajectory(10.0, 1.0);
  const auto original = points;
  // The ego already drives at the limit, so the whole profile settles on it.
  VelocityLimitOptions options;
  options.current_ego_velocity = 5.0;
  options.current_ego_acceleration = 0.0;
  const auto result = apply_velocity_limits(points, constraints, constant_limit(5.0), options);
  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  for (const auto & point : points) {
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 5.0F);
    EXPECT_FLOAT_EQ(point.acceleration_mps2, 0.0F);
  }
  expect_format(original, points);
  expect_straight_motion(points);
}

TEST(MapVelocityLimitsProfile, KeepsResampledPointsOnOriginalCurvedPolyline)
{
  auto points = make_trajectory();
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].pose.position.y = 2.0 * std::sin(i * 0.1);
    points[i].pose.position.z = 0.2 * i;
  }
  const auto original = points;
  const auto result = apply_velocity_limits(points, constraints, constant_limit(5.0));
  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  expect_format(original, points);
  for (const auto & point : points) {
    auto next = std::upper_bound(
      original.begin(), original.end(), point.pose.position.x,
      [](const double x, const auto & p) { return x < p.pose.position.x; });
    ASSERT_NE(next, original.begin());
    if (next == original.end()) {
      EXPECT_EQ(point.pose, original.back().pose);
      continue;
    }
    const auto & p0 = (next - 1)->pose.position;
    const auto & p1 = next->pose.position;
    const double ratio = (point.pose.position.x - p0.x) / (p1.x - p0.x);
    EXPECT_NEAR(point.pose.position.y, p0.y + ratio * (p1.y - p0.y), 1e-9);
    EXPECT_NEAR(point.pose.position.z, p0.z + ratio * (p1.z - p0.z), 1e-9);
  }
}

TEST(MapVelocityLimitsProfile, PreservesValidConstantSpeedTrajectory)
{
  auto points = make_trajectory(5.0);
  const auto original = points;
  const auto result = apply_velocity_limits(points, constraints, constant_limit(10.0));
  EXPECT_EQ(result.status, ProcessingResult::Unchanged);
  EXPECT_EQ(points, original);
}

}  // namespace
