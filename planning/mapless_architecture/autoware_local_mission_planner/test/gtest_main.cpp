// Copyright 2024 driveblocks GmbH, authors: Simon Eisenmann, Thomas Herrmann
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "include/test_mission_planner_core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autoware::mapless_architecture
{

TEST(MissionPlannerCore, CalculateDistanceBetweenPointAndLineString)
{
  EXPECT_EQ(TestCalculateDistanceBetweenPointAndLineString(), 0);
}

TEST(MissionPlannerCore, GetPointOnLane)
{
  EXPECT_EQ(TestGetPointOnLane(), 0);
}

TEST(MissionPlannerCore, RecenterGoalPoint)
{
  EXPECT_EQ(TestRecenterGoalpoint(), 0);
}

TEST(MissionPlanner, IsOnGoalLane)
{
  EXPECT_EQ(TestIsOnGoalLane(), 0);
}

TEST(MissionPlanner, CheckIfGoalPointShouldBeReset)
{
  EXPECT_EQ(TestCheckIfGoalPointShouldBeReset(), 0);
}

TEST(MissionPlanner, CalculateLanes)
{
  EXPECT_EQ(TestCalculateLanes(), 0);
}

TEST(MissionPlanner, CreateMarkerArray)
{
  EXPECT_EQ(TestCreateMarkerArray(), 0);
}

TEST(MissionPlanner, CreateDrivingCorridor)
{
  EXPECT_EQ(TestCreateDrivingCorridor(), 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}
}  // namespace autoware::mapless_architecture
