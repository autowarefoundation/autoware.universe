// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license

#include "autoware/mission_lane_converter/mission_lane_converter_node.hpp"
#include "gtest/gtest.h"

#include "geometry_msgs/msg/point.hpp"

namespace autoware::mapless_architecture
{

int TestMissionToTrajectory()
{
  MissionLaneConverterNode mission_converter = MissionLaneConverterNode();

  autoware_planning_msgs::msg::MissionLanesStamped mission_msg;

  // Set target lane to ego lane
  mission_msg.target_lane = 0;

  // Add a driving corridor to the ego lane
  mission_msg.ego_lane = autoware_planning_msgs::msg::DrivingCorridor();

  // Add points to the ego lane centerline
  mission_msg.ego_lane.centerline.push_back(geometry_msgs::msg::Point());
  mission_msg.ego_lane.centerline.back().x = 0.0;
  mission_msg.ego_lane.centerline.back().y = 0.0;

  // Get converted trajectory
  std::tuple<
    autoware_auto_planning_msgs::msg::Trajectory, visualization_msgs::msg::Marker,
    autoware_auto_planning_msgs::msg::Path, visualization_msgs::msg::MarkerArray>
    mission_to_trj = mission_converter.ConvertMissionToTrajectory(mission_msg);

  // Extract trajectory
  autoware_auto_planning_msgs::msg::Trajectory trj_msg = std::get<0>(mission_to_trj);

  EXPECT_EQ(trj_msg.points.back().pose.position.x, mission_msg.ego_lane.centerline.back().x);
  EXPECT_EQ(trj_msg.points.back().pose.position.y, mission_msg.ego_lane.centerline.back().y);
  EXPECT_EQ(trj_msg.points.back().pose.orientation.x, 0.0);
  EXPECT_EQ(trj_msg.points.back().pose.orientation.y, 0.0);
  EXPECT_EQ(trj_msg.points.back().pose.orientation.z, 0.0);
  EXPECT_EQ(trj_msg.points.back().pose.orientation.w, 1.0);

  // TEST 2: some more points on the ego lane
  // add points to the ego lane centerline
  mission_msg.ego_lane.centerline.push_back(geometry_msgs::msg::Point());
  mission_msg.ego_lane.centerline.back().x = 1.0;
  mission_msg.ego_lane.centerline.back().y = 1.0;
  mission_msg.ego_lane.centerline.push_back(geometry_msgs::msg::Point());
  mission_msg.ego_lane.centerline.back().x = 2.0;
  mission_msg.ego_lane.centerline.back().y = 2.0;

  // Convert
  mission_to_trj = mission_converter.ConvertMissionToTrajectory(mission_msg);

  // Extract trajectory
  trj_msg = std::get<0>(mission_to_trj);

  EXPECT_EQ(trj_msg.points.back().pose.position.x, mission_msg.ego_lane.centerline.back().x);
  EXPECT_EQ(trj_msg.points.back().pose.position.y, mission_msg.ego_lane.centerline.back().y);

  // TEST 3: neighbor lane left
  mission_msg.drivable_lanes_left.push_back(autoware_planning_msgs::msg::DrivingCorridor());
  mission_msg.drivable_lanes_left.back().centerline.push_back(geometry_msgs::msg::Point());
  mission_msg.drivable_lanes_left.back().centerline.back().x = 0.0;
  mission_msg.drivable_lanes_left.back().centerline.back().y = 0.0;
  mission_msg.drivable_lanes_left.back().centerline.push_back(geometry_msgs::msg::Point());
  mission_msg.drivable_lanes_left.back().centerline.back().x = 5.0;
  mission_msg.drivable_lanes_left.back().centerline.back().y = 3.0;

  // Set target lane to neighbor left
  mission_msg.target_lane = -1;

  // Convert
  mission_to_trj = mission_converter.ConvertMissionToTrajectory(mission_msg);

  // Extract trajectory
  trj_msg = std::get<0>(mission_to_trj);

  EXPECT_EQ(
    trj_msg.points.back().pose.position.x,
    mission_msg.drivable_lanes_left.back().centerline.back().x);
  EXPECT_EQ(
    trj_msg.points.back().pose.position.y,
    mission_msg.drivable_lanes_left.back().centerline.back().y);

  // TEST 4: neighbor lane right
  mission_msg.drivable_lanes_right.push_back(autoware_planning_msgs::msg::DrivingCorridor());
  mission_msg.drivable_lanes_right.back().centerline.push_back(geometry_msgs::msg::Point());
  mission_msg.drivable_lanes_right.back().centerline.back().x = 1.0;
  mission_msg.drivable_lanes_right.back().centerline.back().y = 1.2;
  mission_msg.drivable_lanes_right.back().centerline.push_back(geometry_msgs::msg::Point());
  mission_msg.drivable_lanes_right.back().centerline.back().x = 3.0;
  mission_msg.drivable_lanes_right.back().centerline.back().y = 3.6;

  // Set target lane to neighbor right
  mission_msg.target_lane = 1;

  // Convert
  mission_to_trj = mission_converter.ConvertMissionToTrajectory(mission_msg);

  // Extract trajectory
  trj_msg = std::get<0>(mission_to_trj);

  EXPECT_EQ(
    trj_msg.points.back().pose.position.x,
    mission_msg.drivable_lanes_right.back().centerline.back().x);
  EXPECT_EQ(
    trj_msg.points.back().pose.position.y,
    mission_msg.drivable_lanes_right.back().centerline.back().y);

  return 0;
}
}  // namespace autoware::mapless_architecture
