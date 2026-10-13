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

#include <cstddef>
#include <optional>

namespace
{
namespace detail = autoware::trajectory_modifier::plugin::detail;
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using detail::FixedProfileMemory;
using detail::FixedProfileParameters;
using detail::ProfileStart;
using detail::VelocityLimitConstraints;
using detail::VelocityLimitOptions;

constexpr double dt = 0.1;
constexpr double cruise = 16.6667;
constexpr double zone_start = 125.0;
constexpr double zone_limit = 8.3333;
constexpr VelocityLimitConstraints constraints{1.0, 1.0, 3.0};

/// @brief Constant-speed input sampled every dt along +x from `x0`.
TrajectoryPoints make_input(const double x0, const std::size_t count = 120)
{
  TrajectoryPoints points(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double t = dt * static_cast<double>(i);
    points[i].time_from_start = rclcpp::Duration::from_seconds(t);
    points[i].pose.position.x = x0 + cruise * t;
    points[i].pose.orientation.w = 1.0;
    points[i].longitudinal_velocity_mps = static_cast<float>(cruise);
  }
  return points;
}

std::optional<double> zone(const geometry_msgs::msg::Point & position)
{
  return position.x >= zone_start ? zone_limit : cruise;
}

VelocityLimitOptions make_options(const double velocity, const double acceleration)
{
  VelocityLimitOptions options;
  options.current_ego_velocity = velocity;
  options.current_ego_acceleration = acceleration;
  return options;
}

/// @brief Plan that brakes for the zone ahead, from a cruising ego at x = 40 m.
TrajectoryPoints make_plan()
{
  auto plan = make_input(40.0);
  const auto result =
    detail::apply_velocity_limits(plan, constraints, zone, make_options(cruise, 0.0));
  EXPECT_EQ(result.status, ProcessingResult::Modified) << result.error;
  return plan;
}

geometry_msgs::msg::Point point_at(const double x)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  return point;
}

class FixedProfile : public ::testing::Test
{
protected:
  void SetUp() override
  {
    plan_ = make_plan();
    memory_.set_parameters(FixedProfileParameters{});
    memory_.store(0, 1, 10.0, plan_);
  }

  /// @brief Start at t = 10.1 s from a measurement offset from the plan's second point.
  [[nodiscard]] detail::ProfileStartResult start_with_offsets(
    const double velocity, const double acceleration, const double distance,
    const double time = 10.1) const
  {
    const auto & planned = plan_[1];
    return memory_.start(
      0, time, point_at(planned.pose.position.x + distance),
      make_options(
        planned.longitudinal_velocity_mps + velocity, planned.acceleration_mps2 + acceleration));
  }

  TrajectoryPoints plan_;
  FixedProfileMemory memory_;
};

TEST_F(FixedProfile, StartsFromTheMeasuredStateWhenDisabled)
{
  FixedProfileParameters parameters;
  parameters.enable = false;
  memory_.set_parameters(parameters);
  memory_.store(0, 1, 10.0, plan_);

  const auto start = start_with_offsets(0.2, 0.0, 0.0);
  EXPECT_EQ(start.start, ProfileStart::Measured);
  EXPECT_DOUBLE_EQ(*start.options.current_ego_velocity, plan_[1].longitudinal_velocity_mps + 0.2);
}

TEST_F(FixedProfile, StartsFromTheMeasuredStateWithoutPreviousPlan)
{
  EXPECT_EQ(
    memory_.start(1, 10.1, point_at(0.0), make_options(10.0, 0.0)).start,
    ProfileStart::NoPreviousPlan);
  EXPECT_EQ(
    memory_.start(0, std::nullopt, point_at(0.0), make_options(10.0, 0.0)).start,
    ProfileStart::NoPreviousPlan);
}

TEST_F(FixedProfile, ContinuesThePreviousPlanWhileTheEgoTracksIt)
{
  const auto start = start_with_offsets(0.4, -0.4, 0.8);
  ASSERT_EQ(start.start, ProfileStart::PreviousPlan);
  EXPECT_NEAR(*start.options.current_ego_velocity, plan_[1].longitudinal_velocity_mps, 1e-4);
  EXPECT_NEAR(*start.options.current_ego_acceleration, plan_[1].acceleration_mps2, 1e-4);
}

TEST_F(FixedProfile, RestartsFromTheMeasuredStateWhenTheEgoDeviates)
{
  EXPECT_EQ(start_with_offsets(0.6, 0.0, 0.0).start, ProfileStart::ResetVelocity);
  EXPECT_EQ(start_with_offsets(-0.6, 0.0, 0.0).start, ProfileStart::ResetVelocity);
  EXPECT_EQ(start_with_offsets(0.0, 0.6, 0.0).start, ProfileStart::ResetAcceleration);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 1.5).start, ProfileStart::ResetDistance);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0, 10.6).start, ProfileStart::ResetTimeGap);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0, 9.9).start, ProfileStart::ResetTimeGap);

  const auto start = start_with_offsets(0.6, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(*start.options.current_ego_velocity, plan_[1].longitudinal_velocity_mps + 0.6);
}

TEST_F(FixedProfile, KeepsCandidatesApart)
{
  auto other = make_input(40.0);
  const auto result = detail::apply_velocity_limits(
    other, constraints, [](const auto &) { return std::optional<double>{5.0}; },
    make_options(cruise, 0.0));
  ASSERT_EQ(result.status, ProcessingResult::Modified);
  memory_.store(0, 2, 10.0, plan_);
  memory_.store(1, 2, 10.0, other);

  const auto first = memory_.start(
    0, 10.1, plan_[1].pose.position,
    make_options(plan_[1].longitudinal_velocity_mps, plan_[1].acceleration_mps2));
  const auto second = memory_.start(
    1, 10.1, other[1].pose.position,
    make_options(other[1].longitudinal_velocity_mps, other[1].acceleration_mps2));
  ASSERT_EQ(first.start, ProfileStart::PreviousPlan);
  ASSERT_EQ(second.start, ProfileStart::PreviousPlan);
  EXPECT_NEAR(*first.options.current_ego_acceleration, plan_[1].acceleration_mps2, 1e-4);
  EXPECT_NEAR(*second.options.current_ego_acceleration, other[1].acceleration_mps2, 1e-4);

  // A batch with a single candidate forgets the second one.
  memory_.store(0, 1, 10.1, plan_);
  EXPECT_EQ(
    memory_.start(1, 10.2, other[1].pose.position, make_options(cruise, 0.0)).start,
    ProfileStart::NoPreviousPlan);
}

TEST_F(FixedProfile, ParameterChangeForgetsPreviousPlans)
{
  memory_.set_parameters(FixedProfileParameters{});
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0).start, ProfileStart::PreviousPlan);

  FixedProfileParameters parameters;
  parameters.reset_velocity_deviation = 0.3;
  memory_.set_parameters(parameters);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0).start, ProfileStart::NoPreviousPlan);
}

TEST(FixedProfileReplanning, StartingFromThePreviousPlanReproducesIt)
{
  // Every cycle, a new input starts where the previous plan says the ego is, and the measured
  // state is off the plan by a small error. Starting from the previous plan, the successive plans
  // continue the first one, including its braking for the zone.
  const auto first = make_plan();
  FixedProfileMemory memory;
  memory.set_parameters(FixedProfileParameters{});
  memory.store(0, 1, 0.0, first);
  auto previous = first;
  for (std::size_t cycle = 1; cycle <= 40; ++cycle) {
    const double time = dt * static_cast<double>(cycle);
    const auto & planned = previous[1];
    const double error = cycle % 2 == 0 ? 0.2 : -0.2;
    const auto start = memory.start(
      0, time, planned.pose.position,
      make_options(planned.longitudinal_velocity_mps + error, planned.acceleration_mps2 - error));
    ASSERT_EQ(start.start, ProfileStart::PreviousPlan) << cycle;

    auto plan = make_input(planned.pose.position.x);
    detail::apply_velocity_limits(plan, constraints, zone, start.options);
    memory.store(0, 1, time, plan);
    for (std::size_t i = 0; i + cycle < first.size(); ++i) {
      EXPECT_NEAR(
        plan[i].longitudinal_velocity_mps, first[i + cycle].longitudinal_velocity_mps, 2e-2)
        << "cycle " << cycle << " point " << i;
    }
    previous = plan;
  }
}

}  // namespace
