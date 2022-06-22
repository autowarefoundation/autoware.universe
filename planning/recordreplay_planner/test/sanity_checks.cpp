// Copyright 2020-2021 Embotech AG, Zurich, Switzerland, Arm Ltd. Inspired by Christopher Ho
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

#include <gtest/gtest.h>
#include <recordreplay_planner/recordreplay_planner.hpp>
#include <motion_testing/motion_testing.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <motion_common/config.hpp>
#include <motion_common/motion_common.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <common/types.hpp>

#include <chrono>
#include <set>
#include <algorithm>
#include <string>
#include <cstdio>

using motion::planning::recordreplay_planner::RecordReplayPlanner;
using std::chrono::system_clock;
using motion::motion_testing::make_state;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point32;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

class sanity_checks_base : public ::testing::Test
{
protected:
  RecordReplayPlanner planner_{};
};


//------------------ Test basic properties of a recorded, then replayed trajectory
struct PropertyTestParameters
{
  std::chrono::milliseconds time_spacing_ms;
  system_clock::time_point starting_time;
};

class SanityChecksTrajectoryProperties
  : public sanity_checks_base, public testing::WithParamInterface<PropertyTestParameters>
{};

TEST_P(SanityChecksTrajectoryProperties, Basicproperties)
{
  const auto p = GetParam();
  auto t0 = p.starting_time;

  // Build a trajectory
  constexpr auto N = 10;
  constexpr auto dx = 1.0F;
  const auto time_increment = p.time_spacing_ms;
  const auto v = dx / (1.0e-3F * p.time_spacing_ms.count());
  for (uint32_t k = {}; k < N; ++k) {
    const auto next_state = make_state(
      dx * k, 0.0F, 0.0F, v, 0.0F, 0.0F,
      t0 + k * time_increment);
    planner_.record_state(next_state);
  }

  // Test: Check that the length is equal to the number of states we fed in
  EXPECT_EQ(planner_.get_record_length(), static_cast<std::size_t>(N));

  // Test: Check that the plan returned has the expected time length
  auto trajectory = planner_.plan(make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
  float64_t trajectory_time_length = trajectory.points[N - 1].time_from_start.sec + 1e-9F *
    trajectory.points[N - 1].time_from_start.nanosec;
  float64_t endpoint_sec = (1.0F * (N - 1) * time_increment).count() * 1.0e-3;
  float64_t ep = 1.0e-5;
  EXPECT_NEAR(trajectory_time_length, endpoint_sec, ep);
}

INSTANTIATE_TEST_CASE_P(
  TrajectoryProperties,
  SanityChecksTrajectoryProperties,
  testing::Values(
    PropertyTestParameters{std::chrono::milliseconds(100), system_clock::from_time_t({})},
    PropertyTestParameters{std::chrono::milliseconds(200), system_clock::from_time_t({})},
    PropertyTestParameters{std::chrono::milliseconds(100), system_clock::from_time_t(10)},
    PropertyTestParameters{std::chrono::milliseconds(200), system_clock::from_time_t(10)}
    // cppcheck-suppress syntaxError
  ), );


//------------------ Test that length cropping properly works
struct LengthTestParameters
{
  // The number of points to be recorded
  uint32_t number_of_points;
};


class SanityChecksTrajectoryLength
  : public sanity_checks_base, public testing::WithParamInterface<LengthTestParameters>
{};

TEST_P(SanityChecksTrajectoryLength, Length)
{
  const auto p = GetParam();
  const auto N = p.number_of_points;
  const auto dummy_state = make_state(
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    system_clock::from_time_t({}));

  for (uint32_t k = {}; k < N; ++k) {
    planner_.record_state(dummy_state);
  }

  // Test: Check that the length is equal to the number of states we fed in
  EXPECT_EQ(planner_.get_record_length(), N);
  auto trajectory = planner_.plan(dummy_state);

  EXPECT_EQ(
    trajectory.points.size(),
    std::min(N, static_cast<uint32_t>(trajectory.points.max_size())));
}

INSTANTIATE_TEST_CASE_P(
  TrajectoryLength,
  SanityChecksTrajectoryLength,
  testing::Values(
    LengthTestParameters{80},
    LengthTestParameters{200}
  ), );


// Test setup helper function. This creates a planner and records a trajectory
// that goes along the points (0,0), (1,0), .... (N-1,0) with the heading set to
// 0 throughout - for testing purposes
RecordReplayPlanner helper_create_and_record_example(uint32_t N)
{
  auto planner = RecordReplayPlanner();
  auto t0 = system_clock::from_time_t({});

  // Record some states going from
  for (uint32_t k = {}; k < N; ++k) {
    planner.record_state(
      make_state(
        1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        t0 + k * std::chrono::milliseconds{100LL}));
  }

  return planner;
}


//------------------ Test that "receding horizon" planning properly works: happy case
TEST(RecordreplaySanityChecks, RecedingHorizonHappycase)
{
  const auto N = 3;
  auto planner = helper_create_and_record_example(N);

  // Call "plan" multiple times in sequence, expecting the states to come back out in order
  const auto t0 = system_clock::from_time_t({});
  for (uint32_t k = {}; k < N; ++k) {
    auto trajectory = planner.plan(make_state(1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    // normally don't check float equality but we _just_ pushed this float so it ought not
    // to have changed
    EXPECT_EQ(1.0F * k, trajectory.points[0].pose.position.x);
    EXPECT_EQ(N - k, trajectory.points.size());
  }
}

//------------------ Test that "receding horizon" planning properly works:
TEST(RecordreplaySanityChecks, RecedingHorizonCornercases)
{
  const auto N = 3;
  auto planner = helper_create_and_record_example(N);

  const auto t0 = system_clock::from_time_t({});

  // Check: State we have not recorded, but is closest to the (0,0) state
  {
    auto trajectory = planner.plan(make_state(-1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ(0.0F, trajectory.points[0].pose.position.x);
  }

  // Check: State we have not recorded, but is closest to the (0,0) state
  {
    auto trajectory = planner.plan(make_state(0.1F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ(0.0F, trajectory.points[0].pose.position.x);
    EXPECT_EQ(0.0F, trajectory.points[0].pose.position.y);
  }

  // Check: State we have not recorded, but is closest to the (N,0) state
  {
    auto trajectory = planner.plan(make_state(1.0F * N + 5.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ((N - 1) * 1.0F, trajectory.points[0].pose.position.x);
    EXPECT_EQ(0.0F, trajectory.points[0].pose.position.y);
  }
}

TEST(RecordreplaySanityChecks, StateSettingMechanism)
{
  auto planner = RecordReplayPlanner{};

  // Make sure setting and reading the recording state works
  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  planner.start_recording();

  EXPECT_TRUE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  planner.stop_recording();

  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  // Make sure setting and reading the replaying state works
  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  planner.start_replaying();

  EXPECT_FALSE(planner.is_recording() );
  EXPECT_TRUE(planner.is_replaying() );

  planner.stop_replaying();

  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );
}

TEST(RecordreplaySanityChecks, HeadingWeightSetting)
{
  auto planner = RecordReplayPlanner{};

  planner.set_heading_weight(0.5);
  EXPECT_EQ(planner.get_heading_weight(), 0.5);
  EXPECT_THROW(planner.set_heading_weight(-1.0), std::domain_error);
}

// Test write/read trajectory to/from file
TEST(RecordreplayWriteReadTrajectory, WriteReadTrajectory)
{
  std::string file_name("write_test.trajectory");

  const auto N = 5;
  auto planner = helper_create_and_record_example(N);

  // Write, clear buffer and read the written data again
  planner.writeTrajectoryBufferToFile(file_name);

  planner.clear_record();
  EXPECT_EQ(planner.get_record_length(), static_cast<std::size_t>(0));

  planner.readTrajectoryBufferFromFile(file_name);

  EXPECT_EQ(std::remove(file_name.c_str()), 0);
  EXPECT_EQ(planner.get_record_length(), static_cast<std::size_t>(5));

  const auto t0 = system_clock::from_time_t({});
  auto trajectory = planner.plan(make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));

  for (uint32_t k = {}; k < N; ++k) {
    EXPECT_EQ(1.0F * k, trajectory.points[k].pose.position.x);
  }
}

TEST(RecordreplayWriteReadTrajectory, writeTrajectoryEmptyPath)
{
  const auto N = 5;
  auto planner = helper_create_and_record_example(N);

  ASSERT_THROW(planner.writeTrajectoryBufferToFile(""), std::runtime_error);
}

TEST(RecordreplayWriteReadTrajectory, readTrajectoryEmptyPath)
{
  const auto N = 5;
  auto planner = helper_create_and_record_example(N);

  ASSERT_THROW(planner.readTrajectoryBufferFromFile(""), std::runtime_error);
}

TEST(RecordreplayReachGoal, checkReachGoalCondition)
{
  const auto N = 6;
  auto planner = helper_create_and_record_example(N);

  const float64_t distance_thresh = 1.0;
  const float64_t angle_thresh = autoware::common::types::PI_2;

  // vehicle 1.5 meters away from the last point in the trajectory
  {
    const float32_t x = 3.5F;
    const float32_t heading = 0.0F;
    const auto vehicle_state = make_state(
      x, 0.0F, heading, 0.0F, 0.0F, 0.0F,
      system_clock::from_time_t({}));
    planner.plan(vehicle_state);
    EXPECT_FALSE(planner.reached_goal(vehicle_state, distance_thresh, angle_thresh));
  }

  // vehicle facing in opposite direction from the last point in the trajectory
  {
    const float32_t x = 5.0F;
    const float32_t heading = -autoware::common::types::PI;
    const auto vehicle_state = make_state(
      x, 0.0F, heading, 0.0F, 0.0F, 0.0F,
      system_clock::from_time_t({}));
    planner.plan(vehicle_state);
    EXPECT_FALSE(planner.reached_goal(vehicle_state, distance_thresh, angle_thresh));
  }

  // vehicle state is the same as the last point in the trajectory
  {
    const float32_t x = 5.0F;
    const float32_t heading = 0.0F;
    const auto vehicle_state = make_state(
      x, 0.0F, heading, 0.0F, 0.0F, 0.0F,
      system_clock::from_time_t({}));
    planner.plan(vehicle_state);
    EXPECT_TRUE(planner.reached_goal(vehicle_state, distance_thresh, angle_thresh));
  }
}

TEST(RecordreplayLoopingTrajectories, maintainTrajectoryLength) {
  // This test assumes that `trajectory.points.max_size() > 5
  // As of 2021-08-12 it is 100
  uint32_t N = 5;

  // It doesn't matter that this trajectory isn't a real loop, we will treat it like one
  auto planner = helper_create_and_record_example(N);
  planner.set_loop(true);

  // We will start in the middle of this, and we expect
  // that the return trajectory is of the full length
  auto vehicle_state = make_state(
    N / 2, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, system_clock::from_time_t({}));

  auto traj = planner.plan(vehicle_state);
  EXPECT_EQ(traj.points.size(), static_cast<std::size_t>(N));
}

RecordReplayPlanner helper_create_and_record_pseudo_loop(uint32_t N)
{
  if (N < 2) {
    return RecordReplayPlanner();
  } else {
    // Assume that helper_create_and_record_example(N)
    // still creates a straight line trajectory from (0,0) to (N-1,0)
    auto planner = helper_create_and_record_example(N - 1);

    // The last argument could be made better by pulling the
    // time associated with the last point in the trajectory
    // but this isn't that important to testing loop functionality
    planner.record_state(
      make_state(
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        system_clock::from_time_t({})));

    return planner;
  }
}

TEST(RecordreplayLoopingTrajectories, correctLoopHandling) {
  auto planner = helper_create_and_record_pseudo_loop(500);
  planner.set_loop(planner.is_loop(5));

  // We will start in the middle of this, and we
  // expect that the return trajectory is of the full length
  auto record_buf = planner.get_record_buffer();

  auto vehicle_trajectory_point = record_buf[record_buf.size() - 10].state;

  autoware_auto_vehicle_msgs::msg::VehicleKinematicState vehicle_state
  {rosidl_runtime_cpp::MessageInitialization::ALL};

  vehicle_state.state.pose.position.x = vehicle_trajectory_point.pose.position.x;
  vehicle_state.state.pose.position.y = vehicle_trajectory_point.pose.position.y;
  vehicle_state.state.pose.orientation = vehicle_trajectory_point.pose.orientation;
  vehicle_state.state.longitudinal_velocity_mps =
    vehicle_trajectory_point.longitudinal_velocity_mps;
  vehicle_state.state.acceleration_mps2 = vehicle_trajectory_point.acceleration_mps2;
  vehicle_state.state.heading_rate_rps = vehicle_trajectory_point.heading_rate_rps;
  vehicle_state.state.lateral_velocity_mps = vehicle_trajectory_point.lateral_velocity_mps;
  vehicle_state.header.stamp = time_utils::to_message(system_clock::from_time_t({}));

  auto traj = planner.plan(vehicle_state);

  EXPECT_EQ(traj.points.size(), static_cast<std::size_t>(100));
}
