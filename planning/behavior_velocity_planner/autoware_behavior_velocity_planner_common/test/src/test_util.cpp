// Copyright 2024 TIER IV, Inc.
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

#include "./utils.hpp"
#include "autoware/behavior_velocity_planner_common/utilization/util.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

using namespace autoware::behavior_velocity_planner;                  // NOLINT
using namespace autoware::behavior_velocity_planner::planning_utils;  // NOLINT
using autoware_planning_msgs::msg::PathPoint;

TEST(PlanningUtilsTest, calc_segment_index_from_point_index)
{
  auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
  geometry_msgs::msg::Point point;
  point.x = 4.5;
  point.y = 0.0;

  size_t result = calc_segment_index_from_point_index(path.points, point, 4);

  EXPECT_EQ(result, 4);
}

TEST(PlanningUtilsTest, calculate_offset_point2d)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));  // No rotation

  double offset_x = 1.0;
  double offset_y = 1.0;

  auto result = calculate_offset_point2d(pose, offset_x, offset_y);

  EXPECT_NEAR(result.x(), 1.0, 0.1);
  EXPECT_NEAR(result.y(), 1.0, 0.1);
}

TEST(PlanningUtilsTest, create_detection_area_polygons)
{
  // using namespace autoware::behavior_velocity_planner::planning_utils;

  // Input parameters
  Polygons2d da_polys;
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  geometry_msgs::msg::Pose target_pose;
  size_t target_seg_idx = 0;
  autoware::behavior_velocity_planner::DetectionRange da_range;

  da_range.min_longitudinal_distance = 1.0;
  da_range.max_longitudinal_distance = 10.0;
  da_range.max_lateral_distance = 2.0;
  da_range.interval = 5.0;
  da_range.wheel_tread = 1.0;
  da_range.left_overhang = 0.5;
  da_range.right_overhang = 0.5;
  da_range.use_left = true;
  da_range.use_right = true;

  double obstacle_vel_mps = 0.5;
  double min_velocity = 1.0;

  // Path with some points
  for (double i = 0.0; i < 3.0; ++i) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose.position.x = i * 5.0;
    point.point.pose.position.y = 0.0;
    point.point.longitudinal_velocity_mps = 1.0;
    path.points.push_back(point);
  }

  // Target pose
  target_pose.position.x = 0.0;
  target_pose.position.y = 0.0;

  // Call the function
  bool success = create_detection_area_polygons(
    da_polys, path, target_pose, target_seg_idx, da_range, obstacle_vel_mps, min_velocity);

  // Assert success
  EXPECT_TRUE(success);

  // Validate results
  ASSERT_FALSE(da_polys.empty());
  EXPECT_EQ(da_polys.size(), 2);  // Expect polygons for left and right bounds

  // Check the first polygon
  auto & polygon = da_polys.front();
  EXPECT_EQ(polygon.outer().size(), 7);  // Each polygon should be a rectangle

  // Check some specific points
  EXPECT_NEAR(polygon.outer()[0].x(), 1.0, 0.1);  // Left inner bound
  EXPECT_NEAR(polygon.outer()[0].y(), 1.0, 0.1);
}

// Test for calc_judge_line_dist_with_acc_limit
TEST(PlanningUtilsTest, calc_judge_line_dist_with_acc_limit)
{
  double velocity = 10.0;               // m/s
  double max_stop_acceleration = -3.0;  // m/s^2
  double delay_response_time = 1.0;     // s

  double result =
    calc_judge_line_dist_with_acc_limit(velocity, max_stop_acceleration, delay_response_time);

  EXPECT_NEAR(result, 26.67, 0.01);  // Updated expected value
}

// Test for calc_judge_line_dist_with_jerk_limit
TEST(PlanningUtilsTest, calc_judge_line_dist_with_jerk_limit)
{
  double velocity = 10.0;               // m/s
  double acceleration = 0.0;            // m/s^2
  double max_stop_acceleration = -3.0;  // m/s^2
  double max_stop_jerk = -1.0;          // m/s^3
  double delay_response_time = 1.0;     // s

  double result = calc_judge_line_dist_with_jerk_limit(
    velocity, acceleration, max_stop_acceleration, max_stop_jerk, delay_response_time);

  EXPECT_GT(result, 0.0);  // The result should be positive
}

// Test for is_ahead_of
TEST(PlanningUtilsTest, is_ahead_of)
{
  geometry_msgs::msg::Pose target;
  geometry_msgs::msg::Pose origin;
  target.position.x = 10.0;
  target.position.y = 0.0;
  origin.position.x = 0.0;
  origin.position.y = 0.0;
  origin.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));  // No rotation

  EXPECT_TRUE(is_ahead_of(target, origin));

  target.position.x = -10.0;
  EXPECT_FALSE(is_ahead_of(target, origin));
}

TEST(PlanningUtilsTest, insert_decel_point)
{
  auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
  geometry_msgs::msg::Point stop_point;
  stop_point.x = 5.0;
  stop_point.y = 0.0;

  auto stop_pose = insert_decel_point(stop_point, path, 5.0);
  ASSERT_TRUE(stop_pose.has_value());
  EXPECT_NEAR(stop_pose->position.x, 5.0, 0.1);
}

// Test for insert_velocity
TEST(PlanningUtilsTest, insert_velocity)
{
  auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
  autoware_internal_planning_msgs::msg::PathPointWithLaneId path_point;
  path_point.point.pose.position.x = 5.0;
  path_point.point.pose.position.y = 0.0;
  path_point.point.longitudinal_velocity_mps = 10.0;

  size_t insert_index = 5;
  insert_velocity(path, path_point, 10.0, insert_index);

  EXPECT_EQ(path.points.size(), 11);
  EXPECT_NEAR(path.points.at(insert_index).point.longitudinal_velocity_mps, 10.0, 0.1);
}

// Test for insert_stop_point
TEST(PlanningUtilsTest, insert_stop_point)
{
  {
    auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
    geometry_msgs::msg::Point stop_point;
    stop_point.x = 5.0;
    stop_point.y = 0.0;

    auto stop_pose = insert_stop_point(stop_point, path);
    ASSERT_TRUE(stop_pose.has_value());
    EXPECT_NEAR(stop_pose->position.x, 5.0, 0.1);
  }
  {
    auto path = test::generatePath(0.0, 0.0, 10.0, 0.0, 10);
    geometry_msgs::msg::Point stop_point;
    stop_point.x = 5.0;
    stop_point.y = 0.0;

    auto stop_pose = insert_stop_point(stop_point, 4, path);
    ASSERT_TRUE(stop_pose.has_value());
    EXPECT_NEAR(stop_pose->position.x, 5.0, 0.1);
  }
}

// Test for get_ahead_pose
TEST(PlanningUtilsTest, get_ahead_pose)
{
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point1;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point2;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point3;
  point1.point.pose.position.x = 0.0;
  point2.point.pose.position.x = 5.0;
  point3.point.pose.position.x = 10.0;

  path.points.emplace_back(point1);
  path.points.emplace_back(point2);
  path.points.emplace_back(point3);

  double ahead_dist = 7.0;
  auto pose = get_ahead_pose(0, ahead_dist, path);

  EXPECT_NEAR(pose.position.x, 7.0, 0.1);
}

TEST(PlanningUtilsTest, calc_deceleration_velocity_from_distance_to_target)
{
  double max_slowdown_jerk = -1.0;    // m/s^3
  double max_slowdown_accel = -3.0;   // m/s^2
  double current_accel = -1.0;        // m/s^2
  double current_velocity = 10.0;     // m/s
  double distance_to_target = 100.0;  // m

  double result = calc_deceleration_velocity_from_distance_to_target(
    max_slowdown_jerk, max_slowdown_accel, current_accel, current_velocity, distance_to_target);

  EXPECT_LT(result, current_velocity);
}

// Test for to_ros_points
TEST(PlanningUtilsTest, ToRosPoints)
{
  using autoware_perception_msgs::msg::PredictedObject;
  PredictedObjects objects;

  // Add a predicted object
  PredictedObject obj1;
  obj1.kinematics.initial_pose_with_covariance.pose.position.x = 1.0;
  obj1.kinematics.initial_pose_with_covariance.pose.position.y = 2.0;
  obj1.kinematics.initial_pose_with_covariance.pose.position.z = 3.0;
  objects.objects.push_back(obj1);

  // Add another predicted object
  PredictedObject obj2;
  obj2.kinematics.initial_pose_with_covariance.pose.position.x = 4.0;
  obj2.kinematics.initial_pose_with_covariance.pose.position.y = 5.0;
  obj2.kinematics.initial_pose_with_covariance.pose.position.z = 6.0;
  objects.objects.push_back(obj2);

  auto points = to_ros_points(objects);

  ASSERT_EQ(points.size(), 2);  // Verify the number of points
  EXPECT_EQ(points[0].x, 1.0);
  EXPECT_EQ(points[0].y, 2.0);
  EXPECT_EQ(points[0].z, 3.0);
  EXPECT_EQ(points[1].x, 4.0);
  EXPECT_EQ(points[1].y, 5.0);
  EXPECT_EQ(points[1].z, 6.0);
}

// Test for extend_line
TEST(PlanningUtilsTest, ExtendLine)
{
  lanelet::ConstPoint3d point1(lanelet::InvalId, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d point2(lanelet::InvalId, 1.0, 1.0, 0.0);
  double length = 1.0;

  auto extended_line = extend_line(point1, point2, length);

  ASSERT_EQ(extended_line.size(), 2);  // Verify the line has two points

  // Check the extended line coordinates
  EXPECT_NEAR(extended_line[0].x(), -0.707, 0.001);  // Extended in the reverse direction
  EXPECT_NEAR(extended_line[0].y(), -0.707, 0.001);
  EXPECT_NEAR(extended_line[1].x(), 1.707, 0.001);  // Extended in the forward direction
  EXPECT_NEAR(extended_line[1].y(), 1.707, 0.001);
}

TEST(PlanningUtilsTest, get_const_lanelets_from_ids)
{
  const auto package_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
  lanelet::LaneletMapPtr map =
    autoware::test_utils::loadMap(package_dir + "/test_map/lanelet2_map.osm");

  auto lanelets = get_const_lanelets_from_ids(map, {10333, 10310, 10291});

  EXPECT_EQ(lanelets.size(), 3);
}
