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

#include <string>

#include "gtest/gtest.h"

#include "planning_error_monitor/planning_error_monitor_node.hpp"
#include "test_planning_error_monitor_helper.hpp"

constexpr double NOMINAL_INTERVAL = 1.0;
constexpr double ERROR_INTERVAL = 1000.0;
constexpr double ERROR_CURVATURE = 2.0;

TEST(PlanningErrorMonitor, ValidValueChecker)
{
  using autoware_planning_msgs::msg::Trajectory;
  using planning_diagnostics::PlanningErrorMonitorNode;

  // Valid Trajectory
  Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);

  std::string valid_error_msg;
  ASSERT_TRUE(PlanningErrorMonitorNode::checkTrajectoryPointValue(valid_traj, valid_error_msg));
  ASSERT_EQ(valid_error_msg, "This Trajectory doesn't have any invalid values");

  // Nan Trajectory
  Trajectory nan_traj = generateNanTrajectory();

  std::string nan_error_msg;
  ASSERT_FALSE(PlanningErrorMonitorNode::checkTrajectoryPointValue(nan_traj, nan_error_msg));
  ASSERT_EQ(nan_error_msg, "This trajectory has an infinite value");

  // Inf Trajectory
  Trajectory inf_traj = generateInfTrajectory();

  std::string inf_error_msg;
  ASSERT_FALSE(PlanningErrorMonitorNode::checkTrajectoryPointValue(inf_traj, inf_error_msg));
  ASSERT_EQ(nan_error_msg, "This trajectory has an infinite value");
}

TEST(PlanningErrorMonitor, TrajectoryIntervalChecker)
{
  using autoware_planning_msgs::msg::Trajectory;
  using planning_diagnostics::PlanningErrorMonitorNode;
  // Normal Trajectory
  {
    Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);

    std::string valid_msg;
    ASSERT_TRUE(
      PlanningErrorMonitorNode::checkTrajectoryInterval(valid_traj, ERROR_INTERVAL, valid_msg));
    ASSERT_EQ(valid_msg, "Trajectory Interval Length is within the expected range");

    std::string boundary_msg;
    ASSERT_TRUE(
      PlanningErrorMonitorNode::checkTrajectoryInterval(
        valid_traj, NOMINAL_INTERVAL,
        boundary_msg));
    ASSERT_EQ(boundary_msg, "Trajectory Interval Length is within the expected range");
  }

  // Long Interval Trajectory
  {
    Trajectory long_interval_traj = generateTrajectory(ERROR_INTERVAL);
    std::string long_interval_error_msg;
    ASSERT_FALSE(
      PlanningErrorMonitorNode::checkTrajectoryInterval(
        long_interval_traj, NOMINAL_INTERVAL, long_interval_error_msg));
    ASSERT_EQ(
      long_interval_error_msg, "Trajectory Interval Length is longer than the expected range");
  }
}

TEST(PlanningErrorMonitor, TrajectoryCurvatureChecker)
{
  using autoware_planning_msgs::msg::Trajectory;
  using planning_diagnostics::PlanningErrorMonitorNode;
  // Normal Trajectory
  {
    Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);
    std::string valid_error_msg;
    ASSERT_TRUE(
      PlanningErrorMonitorNode::checkTrajectoryCurvature(
        valid_traj, ERROR_CURVATURE,
        valid_error_msg));
    ASSERT_EQ(valid_error_msg, "This trajectory's curvature is within the expected range");
  }

  // TODO(Horibe) write test for large curvature trajectory
  {
    // Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);
    // std::string valid_error_msg;
    // ASSERT_TRUE(
    //   PlanningErrorMonitorNode::checkTrajectoryCurvature(
    //     valid_traj, ERROR_CURVATURE, valid_error_msg));
    // ASSERT_EQ(valid_error_msg, "This trajectory's curvature is within the expected range");
  }
}
