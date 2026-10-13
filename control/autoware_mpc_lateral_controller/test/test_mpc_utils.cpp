// Copyright 2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace
{
namespace MPCUtils = autoware::motion::control::mpc_lateral_controller::MPCUtils;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint makePoint(const double x, const double y, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.longitudinal_velocity_mps = vx;
  return p;
}

autoware::motion::control::mpc_lateral_controller::MPCTrajectory makeCircularArcTrajectory(
  const double radius, const int num_points, const double arc_length)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;

  MPCTrajectory traj;
  const double half_angle = arc_length / (2.0 * radius);
  for (int i = 0; i < num_points; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(num_points - 1);
    const double theta = -half_angle + t * 2.0 * half_angle;
    traj.push_back(
      radius * std::cos(theta), radius * std::sin(theta), 0.0, 0.0, 1.0, 0.0, 0.0,
      static_cast<double>(i) * 0.1);
  }
  return traj;
}

autoware::motion::control::mpc_lateral_controller::MPCTrajectory makeStraightTrajectory(
  const int num_points, const double length)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;

  MPCTrajectory traj;
  for (int i = 0; i < num_points; ++i) {
    const double s = static_cast<double>(i) / static_cast<double>(num_points - 1) * length;
    traj.push_back(s, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, static_cast<double>(i) * 0.1);
  }
  return traj;
}

// Builds a trajectory whose points all sit on the heading direction, spaced by point_interval, with
// a constant velocity. A zero point_interval reproduces a temporal trajectory at standstill, where
// every point collapses onto the same position.
autoware::motion::control::mpc_lateral_controller::MPCTrajectory makeDirectionalTrajectory(
  const int num_points, const double heading_yaw, const double point_interval,
  const double velocity)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;

  MPCTrajectory traj;
  for (int i = 0; i < num_points; ++i) {
    const double distance_from_start = static_cast<double>(i) * point_interval;
    traj.push_back(
      distance_from_start * std::cos(heading_yaw), distance_from_start * std::sin(heading_yaw), 0.0,
      heading_yaw, velocity, 0.0, 0.0, static_cast<double>(i) * 0.1);
  }
  return traj;
}

/* cppcheck-suppress syntaxError */
TEST(TestMPCUtils, InferForwardDrivingFromVelocitySign)
{
  // Moving trajectories are decided by the velocity sign, independently of point spacing.
  const auto forward_traj = makeDirectionalTrajectory(10, M_PI, 0.5, 3.0);
  const auto backward_traj = makeDirectionalTrajectory(10, M_PI, 0.5, -3.0);

  EXPECT_EQ(MPCUtils::infer_forward_driving(forward_traj), std::optional<bool>(true));
  EXPECT_EQ(MPCUtils::infer_forward_driving(backward_traj), std::optional<bool>(false));
}

TEST(TestMPCUtils, IssuedSteeringUsesPostStepTimesAsIntervalEnds)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;
  MPCTrajectory trajectory;
  for (int i = 0; i < 4; ++i) {
    trajectory.push_back(
      static_cast<double>(i), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, static_cast<double>(i + 1),
      0.1 * static_cast<double>(i + 1));
  }

  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.04), 0U);
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.099), 0U);
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.1), 1U);
  // The recorded follower cycle was 102 ms after its trajectory stamp. It must issue u[1],
  // rather than looking another 180 ms ahead and interpolating u[2..3].
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.102), 1U);
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.2), 2U);
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.4), std::nullopt);
}

TEST(TestMPCUtils, IssuedSteeringRejectsInvalidTimes)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;
  MPCTrajectory trajectory;
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.0), std::nullopt);
  trajectory.push_back(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.1);
  trajectory.push_back(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.2, 0.2);
  EXPECT_EQ(
    MPCUtils::findIssuedSteeringCommandIndex(trajectory, std::numeric_limits<double>::quiet_NaN()),
    std::nullopt);
  trajectory.relative_time[1] = 0.0;
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.05), std::nullopt);
}

TEST(TestMPCUtils, IssuedSteeringWithZeroTimePointKeepsFirstCommandUntilNextSample)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;
  MPCTrajectory trajectory;
  trajectory.push_back(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0);
  trajectory.push_back(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.2, 0.1);
  trajectory.push_back(2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.3, 0.2);
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.0), 0U);
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.05), 0U);
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.1), 1U);
  EXPECT_EQ(MPCUtils::findIssuedSteeringCommandIndex(trajectory, 0.2), std::nullopt);
}

TEST(TestMPCUtils, InferForwardDrivingIgnoresLeadingStopPoints)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;

  // A trajectory that starts from standstill and accelerates: the first non-negligible velocity
  // decides the direction.
  MPCTrajectory traj;
  traj.push_back(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  traj.push_back(0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.1);
  traj.push_back(0.1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.2);
  traj.push_back(0.5, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.3);

  EXPECT_EQ(MPCUtils::infer_forward_driving(traj), std::optional<bool>(true));
}

TEST(TestMPCUtils, InferForwardDrivingOnCollapsedStopTrajectory)
{
  // Regression: a stopped temporal trajectory has zero point spacing, so the azimuth between the
  // first two points is meaningless. The direction must be reported as unknown instead of being
  // derived from atan2(0, 0), which used to flip the direction for headings away from +x and
  // rotate every reference yaw by pi.
  for (const double heading_yaw : {0.0, M_PI_2, M_PI, -M_PI_2, 2.5, -2.5}) {
    const auto stopped_traj = makeDirectionalTrajectory(50, heading_yaw, 0.0, 0.0);
    EXPECT_EQ(MPCUtils::infer_forward_driving(stopped_traj), std::nullopt)
      << "heading_yaw = " << heading_yaw;
  }
}

TEST(TestMPCUtils, InferForwardDrivingOnCreepingTrajectory)
{
  // Below the velocity threshold the point spacing is still too short to measure a direction from,
  // so the direction stays unknown and the caller keeps its previous value.
  const auto creeping_traj = makeDirectionalTrajectory(10, M_PI, 0.005, 0.05);
  EXPECT_EQ(MPCUtils::infer_forward_driving(creeping_traj), std::nullopt);
}

TEST(TestMPCUtils, InferForwardDrivingFromGeometryWhenStopped)
{
  // Zero velocity but a long enough geometric baseline: the direction comes from the trajectory
  // shape relative to the trajectory heading.
  const auto forward_traj = makeDirectionalTrajectory(20, M_PI, 0.1, 0.0);
  EXPECT_EQ(MPCUtils::infer_forward_driving(forward_traj), std::optional<bool>(true));

  // Same points, but the stored heading is reversed, i.e. the trajectory runs behind the vehicle.
  auto backward_traj = forward_traj;
  for (auto & yaw : backward_traj.yaw) {
    yaw = 0.0;
  }
  EXPECT_EQ(MPCUtils::infer_forward_driving(backward_traj), std::optional<bool>(false));
}

TEST(TestMPCUtils, InferForwardDrivingWithTooFewPoints)
{
  const auto empty_traj = makeDirectionalTrajectory(0, 0.0, 0.5, 3.0);
  const auto single_point_traj = makeDirectionalTrajectory(1, 0.0, 0.5, 3.0);

  EXPECT_EQ(MPCUtils::infer_forward_driving(empty_traj), std::nullopt);
  EXPECT_EQ(MPCUtils::infer_forward_driving(single_point_traj), std::nullopt);
}

TEST(TestMPC, CalcStopDistance)
{
  constexpr float MOVE = 1.0f;
  constexpr float STOP = 0.0f;

  Trajectory trajectory_msg;
  trajectory_msg.points.push_back(makePoint(0.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(1.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(2.0, 0.0, STOP));  // STOP
  trajectory_msg.points.push_back(makePoint(3.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(4.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(5.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(6.0, 0.0, STOP));  // STOP
  trajectory_msg.points.push_back(makePoint(7.0, 0.0, STOP));  // STOP

  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 0), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 1), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 2), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 3), 3.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 4), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 5), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 6), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 7), -1.0);
}

TEST(TestMPC, ConvertToMPCTrajectoryTemporalUsesTimeFromStart)
{
  Trajectory trajectory_msg;
  auto p0 = makePoint(0.0, 0.0, 2.0f);
  auto p1 = makePoint(1.0, 0.0, 2.0f);
  auto p2 = makePoint(2.0, 0.0, 2.0f);
  p0.time_from_start = rclcpp::Duration::from_seconds(0.0);
  p1.time_from_start = rclcpp::Duration::from_seconds(0.3);
  p2.time_from_start = rclcpp::Duration::from_seconds(0.8);
  p0.front_wheel_angle_rad = 0.1F;
  p1.front_wheel_angle_rad = 0.2F;
  p2.front_wheel_angle_rad = 0.4F;
  trajectory_msg.points = {p0, p1, p2};

  const auto mpc_traj = MPCUtils::convertToMPCTrajectory(trajectory_msg, true);
  ASSERT_EQ(mpc_traj.relative_time.size(), 3UL);
  ASSERT_EQ(mpc_traj.steer.size(), 3UL);
  EXPECT_DOUBLE_EQ(mpc_traj.relative_time.at(0), 0.0);
  EXPECT_DOUBLE_EQ(mpc_traj.relative_time.at(1), 0.3);
  EXPECT_DOUBLE_EQ(mpc_traj.relative_time.at(2), 0.8);
  EXPECT_FLOAT_EQ(static_cast<float>(mpc_traj.steer.at(0)), 0.1F);
  EXPECT_FLOAT_EQ(static_cast<float>(mpc_traj.steer.at(1)), 0.2F);
  EXPECT_FLOAT_EQ(static_cast<float>(mpc_traj.steer.at(2)), 0.4F);
}

TEST(TestMPC, LinearInterpMPCTrajectoryInterpolatesSteering)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;

  MPCTrajectory input;
  input.push_back(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.0);
  input.push_back(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.3, 1.0);

  MPCTrajectory output;
  ASSERT_TRUE(MPCUtils::linearInterpMPCTrajectory({0.0, 1.0}, input, {0.0, 0.5, 1.0}, output));
  ASSERT_EQ(output.steer.size(), 3UL);
  EXPECT_DOUBLE_EQ(output.steer.at(0), 0.1);
  EXPECT_DOUBLE_EQ(output.steer.at(1), 0.2);
  EXPECT_DOUBLE_EQ(output.steer.at(2), 0.3);
}

TEST(TestMPC, CalcNearestPoseInterpUsesTimeWindowCandidates)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;
  using geometry_msgs::msg::Pose;

  MPCTrajectory traj;
  traj.push_back(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  traj.push_back(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
  traj.push_back(2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);
  traj.push_back(3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 3.0);

  Pose self_pose{};
  self_pose.position.x = 2.05;
  self_pose.position.y = 0.0;
  self_pose.orientation.w = 1.0;

  Pose nearest_pose{};
  size_t nearest_index = 0;
  double nearest_time = 0.0;
  constexpr double max_dist = 10.0;
  constexpr double max_yaw = M_PI;
  const bool ok = MPCUtils::calcNearestPoseInterp(
    traj, self_pose, &nearest_pose, &nearest_index, &nearest_time, max_dist, max_yaw, true, 1.8,
    2.2);

  ASSERT_TRUE(ok);
  EXPECT_GE(nearest_time, 1.8);
  EXPECT_LE(nearest_time, 2.2);
  EXPECT_NEAR(nearest_pose.position.x, 2.05, 1e-6);
}

TEST(TestMPC, CalcNearestPoseInterpFallsBackWhenTimeWindowHasNoCandidates)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;
  using geometry_msgs::msg::Pose;

  MPCTrajectory traj;
  traj.push_back(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  traj.push_back(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
  traj.push_back(2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);

  Pose self_pose{};
  self_pose.position.x = 0.2;
  self_pose.position.y = 0.0;
  self_pose.orientation.w = 1.0;

  Pose nearest_pose{};
  size_t nearest_index = 0;
  double nearest_time = 0.0;
  constexpr double max_dist = 10.0;
  constexpr double max_yaw = M_PI;
  const bool ok = MPCUtils::calcNearestPoseInterp(
    traj, self_pose, &nearest_pose, &nearest_index, &nearest_time, max_dist, max_yaw, true, 10.0,
    11.0);

  ASSERT_TRUE(ok);
  EXPECT_NEAR(nearest_pose.position.x, 0.2, 1e-6);
  EXPECT_NEAR(nearest_time, 0.2, 1e-6);
}

TEST(TestMPC, ShortTrajectoryCurvatureUsesGlobalTaubinOnArc)
{
  constexpr int num_points = 20;
  constexpr double radius = 10.0;
  constexpr double arc_length = 1.5;
  constexpr int smoothing = 15;
  constexpr double true_kappa = 1.0 / radius;

  const auto traj = makeCircularArcTrajectory(radius, num_points, arc_length);
  const auto curvature = MPCUtils::calcTrajectoryCurvature(smoothing, traj, false);

  ASSERT_EQ(curvature.size(), static_cast<size_t>(num_points));
  for (size_t i = 0; i < curvature.size(); ++i) {
    EXPECT_NEAR(curvature.at(i), curvature.at(0), 1e-9)
      << "all points should share the same global Taubin curvature";
    EXPECT_NEAR(std::abs(curvature.at(i)), true_kappa, 0.02) << "curvature at index " << i;
  }
}

TEST(TestMPC, ShortTrajectoryCurvatureIsZeroForStraightLine)
{
  constexpr int num_points = 20;
  constexpr int smoothing = 15;

  const auto traj = makeStraightTrajectory(num_points, 1.5);
  const auto curvature = MPCUtils::calcTrajectoryCurvature(smoothing, traj, false);

  ASSERT_EQ(curvature.size(), static_cast<size_t>(num_points));
  for (const double k : curvature) {
    EXPECT_NEAR(k, 0.0, 1e-6);
  }
}

TEST(TestMPC, LongTrajectoryCurvatureUsesPerPointThreePointFit)
{
  constexpr int num_points = 40;
  constexpr double radius = 10.0;
  constexpr double arc_length = 3.0;
  constexpr int smoothing = 15;

  // n=40 with smoothing=15 uses the standard 3-point path (max_smoothing=19 >= 15).
  const auto traj = makeCircularArcTrajectory(radius, num_points, arc_length);
  const auto curvature = MPCUtils::calcTrajectoryCurvature(smoothing, traj, false);

  ASSERT_EQ(curvature.size(), static_cast<size_t>(num_points));

  const size_t interior_idx = 20;
  EXPECT_NEAR(std::abs(curvature.at(interior_idx)), 1.0 / radius, 0.05);

  // Short trajectory on the same arc triggers Taubin fallback and should agree roughly.
  const auto short_traj = makeCircularArcTrajectory(radius, 20, arc_length);
  const auto short_curvature = MPCUtils::calcTrajectoryCurvature(smoothing, short_traj, false);
  EXPECT_NEAR(std::abs(short_curvature.at(0)), std::abs(curvature.at(interior_idx)), 0.05);
}

TEST(TestMPC, TemporalYawAndCurvatureStayStableForShortSegments)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;

  MPCTrajectory traj;
  traj.push_back(0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0);
  traj.push_back(0.01, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.1);
  traj.push_back(0.02, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.2);
  traj.push_back(1.0, 0.0, 0.0, 0.3, 1.0, 0.0, 0.0, 0.3);

  MPCUtils::calcTrajectoryYawFromXY(traj, true, true);
  MPCUtils::calcTrajectoryCurvature(1, 1, traj, true);

  EXPECT_NEAR(traj.yaw.at(0), 0.3, 1e-6);
  EXPECT_NEAR(traj.yaw.at(1), 0.3, 1e-6);
  EXPECT_NEAR(traj.yaw.at(2), 0.3, 1e-6);
  EXPECT_NEAR(traj.k.at(0), 0.0, 1e-6);
  EXPECT_NEAR(traj.k.at(1), 0.0, 1e-6);
  EXPECT_NEAR(traj.k.at(2), 0.0, 1e-6);
}

}  // namespace
