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
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_modifier::plugin::detail::apply_velocity_limits;
using autoware::trajectory_modifier::plugin::detail::jerk_limited_braking_distance;
using autoware::trajectory_modifier::plugin::detail::VelocityLimitConstraints;
using autoware::trajectory_modifier::plugin::detail::VelocityLimitOptions;

TrajectoryPoints load_recorded_input(const std::string & filename)
{
  const auto path = std::filesystem::path(__FILE__).parent_path() / "data" / filename;
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Cannot open recorded trajectory fixture: " + path.string());
  }
  TrajectoryPoints points;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::stringstream values(line);
    std::string cell;
    std::vector<double> columns;
    while (std::getline(values, cell, ',')) {
      columns.push_back(std::stod(cell));
    }
    if (columns.size() != 6) {
      throw std::runtime_error("Invalid trajectory fixture row: " + line);
    }
    auto & point = points.emplace_back();
    point.time_from_start = rclcpp::Duration::from_seconds(columns[0]);
    point.pose.position.x = columns[1];
    point.pose.position.y = columns[2];
    point.pose.position.z = columns[3];
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(columns[4]);
    point.acceleration_mps2 = static_cast<float>(columns[5]);
  }
  return points;
}

double speed_at_time(const TrajectoryPoints & points, const double time)
{
  const auto point =
    std::min_element(points.begin(), points.end(), [time](const auto & left, const auto & right) {
      return std::abs(rclcpp::Duration(left.time_from_start).seconds() - time) <
             std::abs(rclcpp::Duration(right.time_from_start).seconds() - time);
    });
  return point->longitudinal_velocity_mps;
}

double arc_length(const TrajectoryPoints & points, const std::size_t index)
{
  double s = 0.0;
  for (std::size_t i = 1; i <= index; ++i) {
    const auto & p0 = points[i - 1].pose.position;
    const auto & p1 = points[i].pose.position;
    s += std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
  }
  return s;
}

/// @brief The limit is reached where the zone starts: directly when the output gets there,
/// otherwise braking from the last point must still reach it in time.
void expect_reaches_limit_at(
  const TrajectoryPoints & output, const double zone_s, const double limit,
  const VelocityLimitConstraints & constraints)
{
  for (std::size_t i = 1; i < output.size(); ++i) {
    const double s0 = arc_length(output, i - 1);
    const double s1 = arc_length(output, i);
    if (s1 >= zone_s && s1 > s0) {
      const double v0 = output[i - 1].longitudinal_velocity_mps;
      const double v1 = output[i].longitudinal_velocity_mps;
      EXPECT_LE(v0 + (zone_s - s0) / (s1 - s0) * (v1 - v0), limit + 0.2);
      return;
    }
  }
  const auto & last = output.back();
  EXPECT_LE(
    jerk_limited_braking_distance(
      last.longitudinal_velocity_mps, last.acceleration_mps2, limit, constraints.max_deceleration,
      constraints.max_jerk),
    zone_s - arc_length(output, output.size() - 1) + 0.5);
}

void expect_consistent_motion(const TrajectoryPoints & input, const TrajectoryPoints & output)
{
  ASSERT_EQ(input.size(), output.size());
  EXPECT_EQ(input.front().pose.position, output.front().pose.position);
  for (std::size_t index = 1; index < output.size(); ++index) {
    EXPECT_EQ(output[index].time_from_start, input[index].time_from_start);
    const auto & before = output[index - 1];
    const auto & after = output[index];
    const double dt = rclcpp::Duration(after.time_from_start).seconds() -
                      rclcpp::Duration(before.time_from_start).seconds();
    const double distance = std::hypot(
      after.pose.position.x - before.pose.position.x,
      after.pose.position.y - before.pose.position.y);
    const double predicted =
      0.5 * (before.longitudinal_velocity_mps + after.longitudinal_velocity_mps) * dt;
    EXPECT_NEAR(distance, predicted, 0.15) << index;
    if (input[index].longitudinal_velocity_mps > 0.0F) {
      EXPECT_LE(after.longitudinal_velocity_mps, input[index].longitudinal_velocity_mps + 0.1)
        << index;
    }
  }
}

TEST(VelocityLimitsRecordedProfiles, ExternalThirtyDoesNotInstantlyClipPositiveAcceleration)
{
  auto points = load_recorded_input("external_30kph_positive_acceleration.csv");
  const auto input = points;
  VelocityLimitOptions options;
  options.current_ego_velocity = 9.2015;
  options.current_ego_acceleration = 0.1975;
  const auto result = apply_velocity_limits(
    points, VelocityLimitConstraints{1.0, 2.0, 0.6},
    [](const geometry_msgs::msg::Point &) { return std::optional<double>{8.3333333}; }, options);

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  EXPECT_GT(points.front().longitudinal_velocity_mps, 8.3333333);
  EXPECT_NEAR(points.front().longitudinal_velocity_mps, 9.2015, 0.1);
  EXPECT_LE(speed_at_time(points, 3.0), 8.5333333);
  expect_consistent_motion(input, points);
}

TEST(VelocityLimitsRecordedProfiles, MapThirtyBrakesBeforeLimitedLanelet)
{
  auto points = load_recorded_input("map_30kph_turn_positive_acceleration.csv");
  const auto input = points;
  const double first_limited_y = input[66].pose.position.y;
  VelocityLimitOptions options;
  options.current_ego_velocity = 12.4737;
  options.current_ego_acceleration = 0.2214;
  const VelocityLimitConstraints constraints{1.0, 1.0, 3.0};
  const auto result = apply_velocity_limits(
    points, constraints,
    [first_limited_y](const geometry_msgs::msg::Point & position) {
      return std::optional<double>{position.y <= first_limited_y ? 8.3333333 : 16.6666667};
    },
    options);

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  // No acceleration toward the lower limit ahead; braking starts as late as possible.
  EXPECT_LE(speed_at_time(points, 1.5), options.current_ego_velocity.value() + 0.05);
  expect_reaches_limit_at(points, arc_length(input, 66), 8.3333333, constraints);
  for (const auto & point : points) {
    EXPECT_LE(point.longitudinal_velocity_mps, 16.7666667);
  }
  expect_consistent_motion(input, points);
}

TEST(VelocityLimitsRecordedProfiles, MapFiftyStartsBrakingBeforeTargetLanelet)
{
  auto points = load_recorded_input("map_50kph_turn_positive_acceleration.csv");
  const auto input = points;
  const double first_limited_y = input[37].pose.position.y;
  VelocityLimitOptions options;
  options.current_ego_velocity = 15.4009;
  options.current_ego_acceleration = 0.2152;
  const VelocityLimitConstraints constraints{1.0, 1.0, 3.0};
  const auto result = apply_velocity_limits(
    points, constraints,
    [first_limited_y](const geometry_msgs::msg::Point & position) {
      return std::optional<double>{position.y <= first_limited_y ? 13.8888889 : 16.6666667};
    },
    options);

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  EXPECT_NEAR(points.front().longitudinal_velocity_mps, 15.4009, 0.1);
  EXPECT_LE(speed_at_time(points, 1.0), options.current_ego_velocity.value() + 0.05);
  expect_reaches_limit_at(points, arc_length(input, 37), 13.8888889, constraints);
  expect_consistent_motion(input, points);
}
}  // namespace
