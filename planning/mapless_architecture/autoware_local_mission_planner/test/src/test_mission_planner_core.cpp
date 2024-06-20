// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license

#include "gtest/gtest.h"
#include "autoware/local_mission_planner_common/helper_functions.hpp"
#include "autoware/local_mission_planner/mission_planner_node.hpp"

#include "db_msgs/msg/lanelets_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <iostream>
#include <tuple>

db_msgs::msg::LaneletsStamped CreateLanelets()
{
  // Local variables
  const int n_lanelets = 3;
  const int n_attribute_vecs = 1;
  const int n_points = 2;

  // Fill lanelet2 message
  db_msgs::msg::LaneletsStamped message;
  std::vector<db_msgs::msg::Lanelet> lanelet_vec(n_lanelets);
  message.lanelets = lanelet_vec;

  // Global position
  geometry_msgs::msg::PoseStamped msg_global_pose;
  msg_global_pose.header.frame_id = "base_link";

  std::array<db_msgs::msg::Linestring, 2> linestring_vec;
  std::vector<db_msgs::msg::Attribute> attribute_vec(n_attribute_vecs);
  message.lanelets[0].id = 0;
  message.lanelets[0].linestrings = linestring_vec;
  message.lanelets[0].attributes = attribute_vec;
  message.lanelets[1].id = 1;
  message.lanelets[1].linestrings = linestring_vec;
  message.lanelets[1].attributes = attribute_vec;
  message.lanelets[2].id = 2;
  message.lanelets[2].linestrings = linestring_vec;
  message.lanelets[2].attributes = attribute_vec;

  uint16_t id = 0;
  std::vector<db_msgs::msg::DBPoint> points_vec(n_points);
  message.lanelets[0].linestrings[0].id = id;
  message.lanelets[0].linestrings[1].id = ++id;
  message.lanelets[0].linestrings[0].points = points_vec;
  message.lanelets[0].linestrings[1].points = points_vec;
  message.lanelets[1].linestrings[0].id = ++id;
  message.lanelets[1].linestrings[1].id = ++id;
  message.lanelets[1].linestrings[0].points = points_vec;
  message.lanelets[1].linestrings[1].points = points_vec;
  message.lanelets[2].linestrings[0].id = ++id;
  message.lanelets[2].linestrings[1].id = ++id;
  message.lanelets[2].linestrings[0].points = points_vec;
  message.lanelets[2].linestrings[1].points = points_vec;

  tf2::Quaternion quaternion;

  // Vehicle position in global cosy
  message.pose.position.x = 0.0;
  message.pose.position.y = 0.0;
  quaternion.setRPY(0, 0, 0);
  message.pose.orientation.x = quaternion.getX();
  message.pose.orientation.y = quaternion.getY();
  message.pose.orientation.z = quaternion.getZ();
  message.pose.orientation.w = quaternion.getW();

  // Street layout in current local cosy = global cosy since vehicle position is
  // at the origin in the global cosy
  message.lanelets[0].linestrings[0].points[0].pose.position.x = -2.0;
  message.lanelets[0].linestrings[0].points[0].pose.position.y = -0.5;
  message.lanelets[0].linestrings[0].points[0].pose.position.z = 0.0;
  message.lanelets[0].linestrings[0].points[1].pose.position.x = 10.0;
  message.lanelets[0].linestrings[0].points[1].pose.position.y = -0.5;
  message.lanelets[0].linestrings[0].points[1].pose.position.z = 0.0;

  message.lanelets[0].linestrings[1].points[0].pose.position.x = -2.0;
  message.lanelets[0].linestrings[1].points[0].pose.position.y = 0.5;
  message.lanelets[0].linestrings[1].points[0].pose.position.z = 0.0;
  message.lanelets[0].linestrings[1].points[1].pose.position.x = 10.0;
  message.lanelets[0].linestrings[1].points[1].pose.position.y = 0.5;
  message.lanelets[0].linestrings[1].points[1].pose.position.z = 0.0;

  message.lanelets[1].linestrings[0].points[0].pose.position.x = 0.0;
  message.lanelets[1].linestrings[0].points[0].pose.position.y = 1.0;
  message.lanelets[1].linestrings[0].points[0].pose.position.z = 0.0;
  message.lanelets[1].linestrings[0].points[1].pose.position.x = 10.0;
  message.lanelets[1].linestrings[0].points[1].pose.position.y = 1.0;
  message.lanelets[1].linestrings[0].points[1].pose.position.z = 0.0;

  message.lanelets[1].linestrings[1].points[0].pose.position.x = 0.0;
  message.lanelets[1].linestrings[1].points[0].pose.position.y = 2.0;
  message.lanelets[1].linestrings[1].points[0].pose.position.z = 0.0;
  message.lanelets[1].linestrings[1].points[1].pose.position.x = 10.0;
  message.lanelets[1].linestrings[1].points[1].pose.position.y = 2.0;
  message.lanelets[1].linestrings[1].points[1].pose.position.z = 0.0;

  message.lanelets[2].linestrings[0].points[0].pose.position.x = 10.0;
  message.lanelets[2].linestrings[0].points[0].pose.position.y = -0.5;
  message.lanelets[2].linestrings[0].points[0].pose.position.z = 0.0;
  message.lanelets[2].linestrings[0].points[1].pose.position.x = 20.0;
  message.lanelets[2].linestrings[0].points[1].pose.position.y = -0.5;
  message.lanelets[2].linestrings[0].points[1].pose.position.z = 0.0;

  message.lanelets[2].linestrings[1].points[0].pose.position.x = 10.0;
  message.lanelets[2].linestrings[1].points[0].pose.position.y = 0.5;
  message.lanelets[2].linestrings[1].points[0].pose.position.z = 0.0;
  message.lanelets[2].linestrings[1].points[1].pose.position.x = 20.0;
  message.lanelets[2].linestrings[1].points[1].pose.position.y = 0.5;
  message.lanelets[2].linestrings[1].points[1].pose.position.z = 0.0;

  // Define connections
  message.lanelets[0].neighboring_lanelet_id = {0, 1};
  message.lanelets[1].neighboring_lanelet_id = {0, 1};
  message.lanelets[2].neighboring_lanelet_id = {-1, -1};
  message.lanelets[0].successor_lanelet_id = {2};
  message.lanelets[1].successor_lanelet_id = {-1};
  message.lanelets[2].successor_lanelet_id = {-1};

  return message;
}

int TestCalculateDistanceBetweenPointAndLineString()
{
  // Create an example Linestring
  lanelet::LineString2d linestring;
  lanelet::Point2d p1(0, 1.0,
                      0.0);  // First argument is ID (set it to 0 for now)
  linestring.push_back(p1);
  lanelet::Point2d p2(0, 2.0,
                      0.0);  // First argument is ID (set it to 0 for now)
  linestring.push_back(p2);
  lanelet::Point2d p3(0, 3.0,
                      0.0);  // First argument is ID (set it to 0 for now)
  linestring.push_back(p3);

  // Create an example point
  lanelet::BasicPoint2d point(1.0, 1.0);

  // Initialize MissionPlannerNode
  MissionPlannerNode MissionPlanner = MissionPlannerNode();

  // Run function
  double distance = MissionPlanner.CalculateDistanceBetweenPointAndLineString_(linestring, point);

  // Check if distance is near to 1.0 (with allowed error of 0.001)
  EXPECT_NEAR(distance, 1.0, 0.001);

  return 0;
}

int TestGetPointOnLane()
{
  // Create some example lanelets
  auto msg_lanelets = CreateLanelets();

  // Initialize MissionPlannerNode
  MissionPlannerNode MissionPlanner = MissionPlannerNode();

  // Convert road model
  std::vector<LaneletConnection> lanelet_connections;
  std::vector<lanelet::Lanelet> lanelets;

  // Convert message
  autoware_planning_msgs::msg::RoadSegments road_segments =
    lib_mission_planner::ConvertLaneletsStamped2RoadSegments(msg_lanelets);

  MissionPlanner.ConvertInput2LaneletFormat(road_segments, lanelets, lanelet_connections);

  // Get a point from the tested function which has the x value 3.0 and lies on
  // the centerline of the lanelets
  lanelet::BasicPoint2d point = MissionPlanner.GetPointOnLane_({0, 1}, 3.0, lanelets);

  // Check if x value of the point is near to 3.0 (with an allowed error of
  // 0.001)
  EXPECT_NEAR(point.x(), 3.0, 0.001);

  // Get a point from the tested function which has the x value 100.0 and lies
  // on the centerline of the lanelets
  point = MissionPlanner.GetPointOnLane_({0, 1}, 100.0,
                                         lanelets);  // Far away (100m)

  // Check if x value of the point is near to 10.0 (with an allowed error of
  // 0.001)
  EXPECT_NEAR(point.x(), 10.0, 0.001);

  return 0;
}

int TestIsOnGoalLane()
{
  // Create some example lanelets
  auto msg_road_model = CreateLanelets();

  // Initialize MissionPlannerNode
  MissionPlannerNode MissionPlanner = MissionPlannerNode();

  // Convert road model
  std::vector<LaneletConnection> lanelet_connections;
  std::vector<lanelet::Lanelet> lanelets;

  // Convert message
  autoware_planning_msgs::msg::RoadSegments road_segments =
    lib_mission_planner::ConvertLaneletsStamped2RoadSegments(msg_road_model);

  MissionPlanner.ConvertInput2LaneletFormat(road_segments, lanelets, lanelet_connections);

  // Define a point with x = 1.0 and y = 0.0
  lanelet::BasicPoint2d point(1.0, 0.0);

  // Check if the point is on the lane: should be true
  EXPECT_EQ(MissionPlanner.IsOnGoalLane_(0, point, lanelets, lanelet_connections), true);

  // Define a point with x = 100.0 and y = 100.0
  lanelet::BasicPoint2d point2(100.0, 100.0);

  // Check if the point is on the lane: should be false
  EXPECT_EQ(MissionPlanner.IsOnGoalLane_(0, point2, lanelets, lanelet_connections), false);

  // Define a point with x = 15.0 and y = 0.0
  lanelet::BasicPoint2d point3(15.0, 0.0);

  // Check if the point is on the lane: should be true
  EXPECT_EQ(MissionPlanner.IsOnGoalLane_(0, point3, lanelets, lanelet_connections), true);

  return 0;
}

db_msgs::msg::LaneletsStamped GetTestRoadModelForRecenterTests()
{
  // local variables
  const int n_lanelets = 2;
  const int n_attribute_vecs = 1;
  const int n_points = 2;

  // Fill lanelet2 message
  db_msgs::msg::LaneletsStamped message;
  std::vector<db_msgs::msg::Lanelet> lanelet_vec(n_lanelets);
  message.lanelets = lanelet_vec;

  // global position
  geometry_msgs::msg::PoseStamped msg_global_pose;
  msg_global_pose.header.frame_id = "base_link";

  std::array<db_msgs::msg::Linestring, 2> linestring_vec;
  std::vector<db_msgs::msg::Attribute> attribute_vec(n_attribute_vecs);
  message.lanelets[0].id = 0;
  message.lanelets[0].linestrings = linestring_vec;
  message.lanelets[0].attributes = attribute_vec;
  message.lanelets[1].id = 1;
  message.lanelets[1].linestrings = linestring_vec;
  message.lanelets[1].attributes = attribute_vec;

  uint16_t id = 0;
  std::vector<db_msgs::msg::DBPoint> points_vec(n_points);
  message.lanelets[0].linestrings[0].id = id;
  message.lanelets[0].linestrings[1].id = ++id;
  message.lanelets[0].linestrings[0].points = points_vec;
  message.lanelets[0].linestrings[1].points = points_vec;
  message.lanelets[1].linestrings[0].id = ++id;
  message.lanelets[1].linestrings[1].id = ++id;
  message.lanelets[1].linestrings[0].points = points_vec;
  message.lanelets[1].linestrings[1].points = points_vec;

  tf2::Quaternion quaternion;

  // vehicle position in global cosy
  message.pose.position.x = 0.0;
  message.pose.position.y = 0.0;
  quaternion.setRPY(0, 0, 0);
  message.pose.orientation.x = quaternion.getX();
  message.pose.orientation.y = quaternion.getY();
  message.pose.orientation.z = quaternion.getZ();
  message.pose.orientation.w = quaternion.getW();

  // street layout in current local cosy = global cosy since vehicle position is
  // at the origin in the global cosy
  message.lanelets[0].linestrings[0].points[0].pose.position.x = 0.0;
  message.lanelets[0].linestrings[0].points[0].pose.position.y = -0.5;
  message.lanelets[0].linestrings[0].points[0].pose.position.z = 0.0;
  message.lanelets[0].linestrings[0].points[1].pose.position.x = 10.0;
  message.lanelets[0].linestrings[0].points[1].pose.position.y = -0.5;
  message.lanelets[0].linestrings[0].points[1].pose.position.z = 0.0;

  message.lanelets[0].linestrings[1].points[0].pose.position.x = 0.0;
  message.lanelets[0].linestrings[1].points[0].pose.position.y = 0.5;
  message.lanelets[0].linestrings[1].points[0].pose.position.z = 0.0;
  message.lanelets[0].linestrings[1].points[1].pose.position.x = 10.0;
  message.lanelets[0].linestrings[1].points[1].pose.position.y = 0.5;
  message.lanelets[0].linestrings[1].points[1].pose.position.z = 0.0;

  message.lanelets[1].linestrings[0].points[0].pose.position.x = 0.0;
  message.lanelets[1].linestrings[0].points[0].pose.position.y = 1.0;
  message.lanelets[1].linestrings[0].points[0].pose.position.z = 0.0;
  message.lanelets[1].linestrings[0].points[1].pose.position.x = 10.0;
  message.lanelets[1].linestrings[0].points[1].pose.position.y = 1.0;
  message.lanelets[1].linestrings[0].points[1].pose.position.z = 0.0;

  message.lanelets[1].linestrings[1].points[0].pose.position.x = 0.0;
  message.lanelets[1].linestrings[1].points[0].pose.position.y = 2.0;
  message.lanelets[1].linestrings[1].points[0].pose.position.z = 0.0;
  message.lanelets[1].linestrings[1].points[1].pose.position.x = 10.0;
  message.lanelets[1].linestrings[1].points[1].pose.position.y = 2.0;
  message.lanelets[1].linestrings[1].points[1].pose.position.z = 0.0;

  int32_t neigh_1 = 1;
  std::vector<int32_t> neighboring_ids = {neigh_1};
  message.lanelets[0].neighboring_lanelet_id = neighboring_ids;
  std::vector<int32_t> neighboring_ids_2 = {};
  message.lanelets[1].neighboring_lanelet_id = neighboring_ids_2;

  return message;
}

int TestRecenterGoalpoint()
{
  // Create a mission planner
  MissionPlannerNode mission_planner = MissionPlannerNode();

  // Get a local road model for testing
  db_msgs::msg::LaneletsStamped local_road_model = GetTestRoadModelForRecenterTests();

  // Used for the output
  std::vector<LaneletConnection> lanelet_connections;
  std::vector<lanelet::Lanelet> converted_lanelets;

  // Convert message
  autoware_planning_msgs::msg::RoadSegments road_segments =
    lib_mission_planner::ConvertLaneletsStamped2RoadSegments(local_road_model);

  mission_planner.ConvertInput2LaneletFormat(
    road_segments, converted_lanelets, lanelet_connections);

  // TEST 1: point on centerline of origin lanelet
  // define a test point and re-center onto its centerline
  lanelet::BasicPoint2d goal_point(0.0, 0.0);
  lanelet::BasicPoint2d goal_point_recentered =
    mission_planner.RecenterGoalPoint(goal_point, converted_lanelets);

  EXPECT_NEAR(goal_point_recentered.x(), 0.0, 1e-5);
  EXPECT_NEAR(goal_point_recentered.y(), 0.0, 1e-5);

  // TEST 2: point left to centerline of origin lanelet
  // define a test point which is not on the centerline
  goal_point.x() = 1.0;
  goal_point.y() = 0.25;
  goal_point_recentered = mission_planner.RecenterGoalPoint(goal_point, converted_lanelets);

  // expect re-centered point to lie on y coordinate 0
  EXPECT_NEAR(goal_point_recentered.x(), goal_point.x(), 1e-5);
  EXPECT_NEAR(goal_point_recentered.y(), 0.0, 1e-5);

  // TEST 3: point right to centerline of origin lanelet
  // define a test point which is not on the centerline
  goal_point.x() = 8.0;
  goal_point.y() = -0.2;
  goal_point_recentered = mission_planner.RecenterGoalPoint(goal_point, converted_lanelets);

  // expect re-centered point to lie on y coordinate 0 but with x coord equal to
  // goal_point (removes lateral error/noise)
  EXPECT_NEAR(goal_point_recentered.x(), goal_point.x(), 1e-5);
  EXPECT_NEAR(goal_point_recentered.y(), 0.0, 1e-5);

  // TEST 4: point left of centerline of neighboring lanelet -> tests if correct
  // lanelet is selected for re-centering define a test point which is not on
  // the centerline
  goal_point.x() = 2.0;
  goal_point.y() = 1.75;
  goal_point_recentered = mission_planner.RecenterGoalPoint(goal_point, converted_lanelets);

  // expect re-centered point to lie on y coordinate 0 but with x coord equal to
  // goal_point (removes lateral error/noise)
  EXPECT_NEAR(goal_point_recentered.x(), goal_point.x(), 1e-5);
  EXPECT_NEAR(goal_point_recentered.y(), 1.5, 1e-5);

  // TEST 5: point outside of local road model, goal point should remain
  // untouched
  goal_point.x() = 11.0;
  goal_point.y() = -2.0;
  goal_point_recentered = mission_planner.RecenterGoalPoint(goal_point, converted_lanelets);

  EXPECT_EQ(goal_point_recentered.x(), goal_point.x());
  EXPECT_EQ(goal_point_recentered.y(), goal_point.y());

  return 0;
}

int TestCheckIfGoalPointShouldBeReset()
{
  // Create some example lanelets
  auto msg = CreateLanelets();

  // Convert message
  autoware_planning_msgs::msg::RoadSegments road_segments =
    lib_mission_planner::ConvertLaneletsStamped2RoadSegments(msg);
  autoware_planning_msgs::msg::LocalMap local_map;
  local_map.road_segments = road_segments;

  // Initialize MissionPlannerNode
  MissionPlannerNode MissionPlanner = MissionPlannerNode();

  // Convert road model
  std::vector<LaneletConnection> lanelet_connections;
  std::vector<lanelet::Lanelet> lanelets;

  MissionPlanner.ConvertInput2LaneletFormat(road_segments, lanelets, lanelet_connections);

  // Compute available lanes
  MissionPlanner.CallbackLocalMapMessages_(local_map);

  // TEST 1: check if goal point is reset in non-default mission
  // Define a goal point with negative x value
  lanelet::BasicPoint2d point(-1.0, 0.0);
  MissionPlanner.goal_point(point);

  // set a non-default mission to make the goal point reset work
  autoware_planning_msgs::msg::Mission mission_msg;
  mission_msg.mission_type = autoware_planning_msgs::msg::Mission::LANE_CHANGE_LEFT;
  MissionPlanner.CallbackMissionMessages_(mission_msg);

  // Call function which is tested
  MissionPlanner.CheckIfGoalPointShouldBeReset_(lanelets, lanelet_connections);

  // Check if the goal point is reset
  EXPECT_EQ(
    MissionPlanner.goal_point().x(),
    10);  // Projection is to x = 10, since this is the highest x value
          // on the right neighbor lane!

  // TEST 2: check if goal point reset is skipped in default mission
  MissionPlanner.goal_point(point);

  // set a non-default mission to make the goal point reset work
  mission_msg.mission_type = autoware_planning_msgs::msg::Mission::LANE_KEEP;
  MissionPlanner.CallbackMissionMessages_(mission_msg);

  // Call function which is tested
  MissionPlanner.CheckIfGoalPointShouldBeReset_(lanelets, lanelet_connections);

  // Check if the goal point is reset
  EXPECT_EQ(
    MissionPlanner.goal_point().x(),
    point.x());  // goal point should equal the input point since goal point
                 // is not reset in the default mission (ego lane following)

  return 0;
}

std::tuple<std::vector<lanelet::Lanelet>, std::vector<LaneletConnection>> CreateLane()
{
  // Local variables
  const int n_lanelets = 2;
  const int n_attribute_vecs = 1;
  const int n_points = 2;

  // Fill lanelet2 message
  db_msgs::msg::LaneletsStamped message;
  std::vector<db_msgs::msg::Lanelet> lanelet_vec(n_lanelets);
  message.lanelets = lanelet_vec;

  // Global position
  geometry_msgs::msg::PoseStamped msg_global_pose;
  msg_global_pose.header.frame_id = "base_link";

  std::array<db_msgs::msg::Linestring, 2> linestring_vec;
  std::vector<db_msgs::msg::Attribute> attribute_vec(n_attribute_vecs);
  message.lanelets[0].id = 0;
  message.lanelets[0].linestrings = linestring_vec;
  message.lanelets[0].attributes = attribute_vec;
  message.lanelets[1].id = 1;
  message.lanelets[1].linestrings = linestring_vec;
  message.lanelets[1].attributes = attribute_vec;

  uint16_t id = 0;
  std::vector<db_msgs::msg::DBPoint> points_vec(n_points);
  message.lanelets[0].linestrings[0].id = id;
  message.lanelets[0].linestrings[1].id = ++id;
  message.lanelets[0].linestrings[0].points = points_vec;
  message.lanelets[0].linestrings[1].points = points_vec;
  message.lanelets[1].linestrings[0].id = ++id;
  message.lanelets[1].linestrings[1].id = ++id;
  message.lanelets[1].linestrings[0].points = points_vec;
  message.lanelets[1].linestrings[1].points = points_vec;

  tf2::Quaternion quaternion;

  // Vehicle position in global cosy
  message.pose.position.x = 0.0;
  message.pose.position.y = 0.0;
  quaternion.setRPY(0, 0, 0);
  message.pose.orientation.x = quaternion.getX();
  message.pose.orientation.y = quaternion.getY();
  message.pose.orientation.z = quaternion.getZ();
  message.pose.orientation.w = quaternion.getW();

  // Street layout in current local cosy = global cosy since vehicle position is
  // at the origin in the global cosy
  message.lanelets[0].linestrings[0].points[0].pose.position.x = -2.0;
  message.lanelets[0].linestrings[0].points[0].pose.position.y = -0.5;
  message.lanelets[0].linestrings[0].points[0].pose.position.z = 0.0;
  message.lanelets[0].linestrings[0].points[1].pose.position.x = 10.0;
  message.lanelets[0].linestrings[0].points[1].pose.position.y = -0.5;
  message.lanelets[0].linestrings[0].points[1].pose.position.z = 0.0;

  message.lanelets[0].linestrings[1].points[0].pose.position.x = -2.0;
  message.lanelets[0].linestrings[1].points[0].pose.position.y = 0.5;
  message.lanelets[0].linestrings[1].points[0].pose.position.z = 0.0;
  message.lanelets[0].linestrings[1].points[1].pose.position.x = 10.0;
  message.lanelets[0].linestrings[1].points[1].pose.position.y = 0.5;
  message.lanelets[0].linestrings[1].points[1].pose.position.z = 0.0;

  message.lanelets[1].linestrings[0].points[0].pose.position.x = 10.0;
  message.lanelets[1].linestrings[0].points[0].pose.position.y = -0.5;
  message.lanelets[1].linestrings[0].points[0].pose.position.z = 0.0;
  message.lanelets[1].linestrings[0].points[1].pose.position.x = 20.0;
  message.lanelets[1].linestrings[0].points[1].pose.position.y = -0.5;
  message.lanelets[1].linestrings[0].points[1].pose.position.z = 0.0;

  message.lanelets[1].linestrings[1].points[0].pose.position.x = 10.0;
  message.lanelets[1].linestrings[1].points[0].pose.position.y = 0.5;
  message.lanelets[1].linestrings[1].points[0].pose.position.z = 0.0;
  message.lanelets[1].linestrings[1].points[1].pose.position.x = 20.0;
  message.lanelets[1].linestrings[1].points[1].pose.position.y = 0.5;
  message.lanelets[1].linestrings[1].points[1].pose.position.z = 0.0;

  // Define connections
  message.lanelets[0].successor_lanelet_id = {1};
  message.lanelets[0].neighboring_lanelet_id = {-1, -1};
  message.lanelets[1].successor_lanelet_id = {-1};
  message.lanelets[1].neighboring_lanelet_id = {-1, -1};

  // Initialize MissionPlannerNode
  MissionPlannerNode MissionPlanner = MissionPlannerNode();

  // Output
  std::vector<LaneletConnection> lanelet_connections;
  std::vector<lanelet::Lanelet> converted_lanelets;

  // Convert message
  autoware_planning_msgs::msg::RoadSegments road_segments =
    lib_mission_planner::ConvertLaneletsStamped2RoadSegments(message);

  MissionPlanner.ConvertInput2LaneletFormat(road_segments, converted_lanelets, lanelet_connections);
  return std::make_tuple(converted_lanelets, lanelet_connections);
}

int TestCalculateLanes()
{
  // Create some example lanelets: Two lanelets 0 and 1, 1 is the successor of
  // 0, the ego lanelet is 0
  auto tuple = CreateLane();
  std::vector<lanelet::Lanelet> lanelets = std::get<0>(tuple);
  std::vector<LaneletConnection> lanelet_connections = std::get<1>(tuple);

  // Initialize MissionPlannerNode
  MissionPlannerNode MissionPlanner = MissionPlannerNode();

  // Call function which is tested
  Lanes result = MissionPlanner.CalculateLanes_(lanelets, lanelet_connections);

  // Get lanes
  std::vector<int> ego_lane_idx = result.ego;
  std::vector<int> left_lane_idx = result.left[0];
  std::vector<int> right_lane_idx = result.right[0];

  // Check if left lane is empty
  EXPECT_EQ(left_lane_idx.empty(), true);  // Expect empty lane

  // Check if right lane is empty
  EXPECT_EQ(right_lane_idx.empty(), true);  // Expect empty lane

  // Check if ego lane is correct
  EXPECT_EQ(
    ego_lane_idx[0],
    0);  // Expect ego lane = {0, 1} (1 is index of successor of ego lanelet)

  EXPECT_EQ(
    ego_lane_idx[1],
    1);  // Expect ego lane = {0, 1} (1 is index of successor of ego lanelet)

  return 0;
}

int TestCreateMarkerArray()
{
  // Create some example lanelets
  auto tuple = CreateLane();
  std::vector<lanelet::Lanelet> lanelets = std::get<0>(tuple);
  std::vector<LaneletConnection> lanelet_connections = std::get<1>(tuple);

  // Initialize MissionPlannerNode
  MissionPlannerNode MissionPlanner = MissionPlannerNode();

  // Create empty message
  autoware_planning_msgs::msg::RoadSegments message;

  // Calculate centerlines, left and right bounds
  std::vector<lanelet::ConstLineString3d> centerlines;
  std::vector<lanelet::ConstLineString3d> left_bounds;
  std::vector<lanelet::ConstLineString3d> right_bounds;

  for (const lanelet::Lanelet & l : lanelets) {
    auto centerline = l.centerline();
    auto bound_left = l.leftBound();
    auto bound_right = l.rightBound();

    centerlines.push_back(centerline);
    left_bounds.push_back(bound_left);
    right_bounds.push_back(bound_right);
  }

  // Call function which is tested
  auto marker_array =
    MissionPlanner.CreateMarkerArray_(centerlines, left_bounds, right_bounds, message);

  // Check if marker_array is empty
  EXPECT_EQ(marker_array.markers.empty(),
            false);  // Expect not empty marker_array

  // Check first point of marker array (should be first point on the centerline)
  EXPECT_EQ(marker_array.markers[0].points[0].x,
            -2.0);  // Expect -2.0 for x

  // Check first point of marker array (should be first point on the centerline)
  EXPECT_EQ(marker_array.markers[0].points[0].y,
            0.0);  // Expect 0.0 for y

  return 0;
}

int TestCreateDrivingCorridor()
{
  // Create some example lanelets
  auto tuple = CreateLane();
  std::vector<lanelet::Lanelet> lanelets = std::get<0>(tuple);
  std::vector<LaneletConnection> lanelet_connections = std::get<1>(tuple);

  // Initialize MissionPlannerNode
  MissionPlannerNode MissionPlanner = MissionPlannerNode();

  // Call function which is tested
  autoware_planning_msgs::msg::DrivingCorridor driving_corridor =
    MissionPlanner.CreateDrivingCorridor_({0, 1}, lanelets);

  // Check if x value of first point in centerline is -2.0
  EXPECT_EQ(driving_corridor.centerline[0].x, -2.0);

  // Check if y value of first point in centerline is 0.0
  EXPECT_EQ(driving_corridor.centerline[0].y, 0.0);

  // Check if x value of first point in left bound is -2.0
  EXPECT_EQ(driving_corridor.bound_left[0].x, -2.0);

  // Check if y value of first point in left bound is -0.5
  EXPECT_EQ(driving_corridor.bound_left[0].y, -0.5);

  // Check if x value of first point in right bound is -2.0
  EXPECT_EQ(driving_corridor.bound_right[0].x, -2.0);

  // Check if y value of first point in right bound is 0.5
  EXPECT_EQ(driving_corridor.bound_right[0].y, 0.5);

  return 0;
}
