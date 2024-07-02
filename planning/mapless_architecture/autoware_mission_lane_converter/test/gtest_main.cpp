// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#include "gtest/gtest.h"
#include "include/test_mission_planner_converter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autoware::mapless_architecture
{

TEST(MissionConverter, MissionToTrajectory)
{
  EXPECT_EQ(TestMissionToTrajectory(), 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}

}  // namespace autoware::mapless_architecture
