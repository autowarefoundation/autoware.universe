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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <rclcpp/duration.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <vector>

namespace
{
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_modifier::plugin::detail::apply_velocity_limits;
using autoware::trajectory_modifier::plugin::detail::jerk_limited_braking_distance;
using autoware::trajectory_modifier::plugin::detail::VelocityLimitConstraints;
using autoware::trajectory_modifier::plugin::detail::VelocityLimitOptions;

constexpr double dt = 0.1;
constexpr VelocityLimitConstraints constraints{1.0, 1.0, 3.0};

/// @brief Time-optimal braking integrated numerically: jerk toward the deceleration limit, hold,
/// and release when releasing ends at the target.
double numeric_braking_distance(
  double velocity, double acceleration, const double target, const double deceleration,
  const double jerk)
{
  constexpr double step = 1e-5;
  double distance = 0.0;
  const auto release_velocity = [&] {
    return velocity + acceleration * std::abs(acceleration) / (2.0 * jerk);
  };
  if (release_velocity() <= target) {
    return 0.0;
  }
  while (release_velocity() > target) {
    const double next = std::max(-deceleration, acceleration - jerk * step);
    const double applied = acceleration < -deceleration ? acceleration + jerk * step : next;
    distance += velocity * step;
    velocity += 0.5 * (acceleration + applied) * step;
    acceleration = applied;
  }
  while (acceleration < 0.0) {
    distance += velocity * step;
    velocity += acceleration * step;
    acceleration += jerk * step;
  }
  return distance;
}

/// @brief Straight trajectory sampled every dt from t=0 with positions integrated from velocities.
TrajectoryPoints make_trajectory(
  const std::size_t count, const std::function<double(double)> & velocity_at)
{
  TrajectoryPoints points(count);
  double x = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const double t = dt * static_cast<double>(i);
    if (i > 0) {
      x += 0.5 * (points[i - 1].longitudinal_velocity_mps + velocity_at(t)) * dt;
    }
    points[i].time_from_start = rclcpp::Duration::from_seconds(t);
    points[i].pose.position.x = x;
    points[i].pose.orientation.w = 1.0;
    points[i].longitudinal_velocity_mps = static_cast<float>(velocity_at(t));
  }
  return points;
}

VelocityLimitOptions make_options(const double velocity, const double acceleration)
{
  VelocityLimitOptions options;
  options.current_ego_velocity = velocity;
  options.current_ego_acceleration = acceleration;
  return options;
}

void expect_within_constraints(const TrajectoryPoints & points, const double initial_acceleration)
{
  constexpr double tolerance = 1e-4;
  double previous = initial_acceleration;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double acceleration = points[i].acceleration_mps2;
    EXPECT_GE(acceleration, -constraints.max_deceleration - tolerance) << i;
    EXPECT_LE(acceleration, constraints.max_acceleration + tolerance) << i;
    EXPECT_LE(std::abs(acceleration - previous), constraints.max_jerk * dt + tolerance) << i;
    EXPECT_GE(points[i].longitudinal_velocity_mps, 0.0F) << i;
    previous = acceleration;
  }
}

double velocity_at_x(const TrajectoryPoints & points, const double x)
{
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto & p0 = points[i - 1];
    const auto & p1 = points[i];
    if (p1.pose.position.x >= x && p1.pose.position.x > p0.pose.position.x) {
      const double ratio = (x - p0.pose.position.x) / (p1.pose.position.x - p0.pose.position.x);
      return p0.longitudinal_velocity_mps +
             ratio * (p1.longitudinal_velocity_mps - p0.longitudinal_velocity_mps);
    }
  }
  return points.back().longitudinal_velocity_mps;
}

TEST(VelocityLimits, BrakingDistanceMatchesNumericIntegration)
{
  struct Case
  {
    double velocity;
    double acceleration;
    double target;
    double deceleration;
    double jerk;
  };
  const std::vector<Case> cases{{16.67, 0.0, 8.33, 1.0, 3.0}, {10.0, 1.0, 4.0, 2.0, 0.5},
                                {10.0, -2.0, 4.0, 1.0, 1.0},  {5.0, -0.5, 0.0, 1.0, 3.0},
                                {9.0, 0.0, 8.8, 1.0, 3.0},    {12.0, -0.3, 11.0, 1.0, 3.0},
                                {10.0, -1.0, 9.9, 1.0, 3.0}};
  for (const auto & c : cases) {
    EXPECT_NEAR(
      jerk_limited_braking_distance(c.velocity, c.acceleration, c.target, c.deceleration, c.jerk),
      numeric_braking_distance(c.velocity, c.acceleration, c.target, c.deceleration, c.jerk), 2e-2)
      << c.velocity << " " << c.acceleration << " " << c.target;
  }
  EXPECT_DOUBLE_EQ(jerk_limited_braking_distance(10.0, -1.0, 9.9, 1.0, 3.0), 0.0);
}

TEST(VelocityLimits, StartsAtTheEgoState)
{
  auto points = make_trajectory(80, [](double) { return 16.0; });
  const auto result = apply_velocity_limits(
    points, constraints,
    [](const geometry_msgs::msg::Point &) { return std::optional<double>{10.0}; },
    make_options(12.0, 0.4));

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  EXPECT_FLOAT_EQ(points.front().longitudinal_velocity_mps, 12.0F);
  EXPECT_FLOAT_EQ(points.front().acceleration_mps2, 0.4F);
  expect_within_constraints(points, 0.4);
  EXPECT_NEAR(points.back().longitudinal_velocity_mps, 10.0, 0.05);
}

TEST(VelocityLimits, ReachesZoneLimitWhereItStartsWithoutBrakingEarly)
{
  constexpr double cruise = 16.6667;
  constexpr double limit = 8.3333;
  constexpr double zone_start = 125.0;
  // Long enough for the braking profile to reach the zone.
  auto points = make_trajectory(120, [](double) { return cruise; });
  const auto result = apply_velocity_limits(
    points, constraints,
    [=](const geometry_msgs::msg::Point & position) {
      return std::optional<double>{position.x >= zone_start ? limit : cruise};
    },
    make_options(cruise, 0.0));

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  expect_within_constraints(points, 0.0);
  // Braking starts when 95% of the deceleration limit is needed.
  const double braking_distance = jerk_limited_braking_distance(
    cruise, 0.0, limit, 0.95 * constraints.max_deceleration, constraints.max_jerk);
  EXPECT_NEAR(velocity_at_x(points, zone_start - braking_distance - 2.0), cruise, 1e-3);
  EXPECT_LE(velocity_at_x(points, zone_start), limit + 0.1);
}

TEST(VelocityLimits, StopsSmoothlyAtZeroLimit)
{
  auto points = make_trajectory(100, [](double) { return 5.0; });
  const auto result = apply_velocity_limits(
    points, constraints,
    [](const geometry_msgs::msg::Point &) { return std::optional<double>{0.0}; },
    make_options(5.0, 0.0));

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  expect_within_constraints(points, 0.0);
  EXPECT_NEAR(points.back().longitudinal_velocity_mps, 0.0, 1e-2);
  EXPECT_NEAR(points.back().acceleration_mps2, 0.0, 1e-2);
}

TEST(VelocityLimits, AnticipatesSteeperUpstreamStop)
{
  // The upstream stops at 2 m/s^2 at x = 60 m; braking at 1 m/s^2 must start earlier.
  constexpr double stop_x = 60.0;
  constexpr double cruise = 8.0;
  TrajectoryPoints points(150);
  double x = 0.0;
  double velocity = cruise;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      const double next = std::min(cruise, std::sqrt(4.0 * std::max(0.0, stop_x - x)));
      x += 0.5 * (velocity + next) * dt;
      velocity = std::min(cruise, std::sqrt(4.0 * std::max(0.0, stop_x - x)));
    }
    points[i].time_from_start = rclcpp::Duration::from_seconds(dt * static_cast<double>(i));
    points[i].pose.position.x = x;
    points[i].pose.orientation.w = 1.0;
    points[i].longitudinal_velocity_mps = static_cast<float>(velocity);
  }
  const auto input = points;
  const auto result = apply_velocity_limits(
    points, constraints,
    [](const geometry_msgs::msg::Point &) { return std::optional<double>{6.0}; },
    make_options(6.0, 0.0));

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  expect_within_constraints(points, 0.0);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    EXPECT_LE(
      points[i].longitudinal_velocity_mps, velocity_at_x(input, points[i].pose.position.x) + 0.05)
      << i;
  }
  EXPECT_LE(points.back().pose.position.x, stop_x + 0.1);
  EXPECT_NEAR(points.back().longitudinal_velocity_mps, 0.0, 0.05);
}

}  // namespace
