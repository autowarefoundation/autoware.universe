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

#include "../src/utils.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <tier4_planning_msgs/msg/detail/path_point_with_lane_id__struct.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <stdexcept>

TEST(NoStoppingAreaTest, isTargetStuckVehicleType)
{
  using autoware::behavior_velocity_planner::no_stopping_area::is_vehicle_type;
  autoware_perception_msgs::msg::PredictedObject object;
  EXPECT_NO_FATAL_FAILURE(is_vehicle_type(object));
  autoware_perception_msgs::msg::ObjectClassification classification;
  for (const auto label : {
         autoware_perception_msgs::msg::ObjectClassification::CAR,
         autoware_perception_msgs::msg::ObjectClassification::BUS,
         autoware_perception_msgs::msg::ObjectClassification::TRUCK,
         autoware_perception_msgs::msg::ObjectClassification::TRAILER,
         autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE,
       }) {
    classification.label = label;
    object.classification = {classification};
    EXPECT_TRUE(is_vehicle_type(object));
  }
}

TEST(NoStoppingAreaTest, insertStopPoint)
{
  using autoware::behavior_velocity_planner::no_stopping_area::insert_stop_point;
  tier4_planning_msgs::msg::PathWithLaneId base_path;
  constexpr auto nb_points = 10;
  constexpr auto default_velocity = 5.0;
  for (auto x = 0; x < nb_points; ++x) {
    tier4_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = 1.0 * x;
    p.point.pose.position.y = 0.0;
    p.point.longitudinal_velocity_mps = default_velocity;
    base_path.points.push_back(p);
  }
  autoware::behavior_velocity_planner::no_stopping_area::PathIndexWithPose stop_point;
  // stop exactly at a point, expect no new points but a 0 velocity at the stop point and after
  auto path = base_path;
  stop_point.first = 4UL;
  stop_point.second = path.points[stop_point.first].point.pose;
  insert_stop_point(path, stop_point);
  ASSERT_EQ(path.points.size(), nb_points);
  for (auto i = 0UL; i < stop_point.first; ++i) {
    EXPECT_EQ(path.points[i].point.longitudinal_velocity_mps, default_velocity);
  }
  for (auto i = stop_point.first; i < nb_points; ++i) {
    EXPECT_EQ(path.points[i].point.longitudinal_velocity_mps, 0.0);
  }

  // stop between points
  path = base_path;
  stop_point.first = 3UL;
  stop_point.second.position.x = 3.5;
  insert_stop_point(path, stop_point);
  ASSERT_EQ(path.points.size(), nb_points + 1);
  EXPECT_EQ(path.points[stop_point.first + 1].point.pose.position.x, stop_point.second.position.x);
  for (auto i = 0UL; i <= stop_point.first; ++i) {
    EXPECT_EQ(path.points[i].point.longitudinal_velocity_mps, default_velocity);
  }
  for (auto i = stop_point.first + 1; i < nb_points + 1; ++i) {
    EXPECT_EQ(path.points[i].point.longitudinal_velocity_mps, 0.0);
  }

  // stop at the last point: exception  // TODO(anyone): is this okay ?
  path = base_path;
  stop_point.first = path.points.size() - 1;
  stop_point.second = path.points.back().point.pose;
  EXPECT_THROW(insert_stop_point(path, stop_point), std::out_of_range);
}

TEST(NoStoppingAreaTest, generateStopLine)
{
  using autoware::behavior_velocity_planner::no_stopping_area::generate_stop_line;
  tier4_planning_msgs::msg::PathWithLaneId path;
  constexpr auto nb_points = 10;
  for (auto x = 0; x < nb_points; ++x) {
    tier4_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = 1.0 * x;
    p.point.pose.position.y = 0.0;
    path.points.push_back(p);
  }
  lanelet::ConstPolygons3d no_stopping_areas;
  double ego_width = 0.0;
  double stop_line_margin = 0.0;
  // no areas, expect no stop line
  auto stop_line = generate_stop_line(path, no_stopping_areas, ego_width, stop_line_margin);
  EXPECT_FALSE(stop_line.has_value());

  // area outside of the path
  lanelet::Polygon3d poly;
  poly.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, -2.0));
  poly.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, -0.5));
  poly.push_back(lanelet::Point3d(lanelet::InvalId, 5.0, -0.5));
  poly.push_back(lanelet::Point3d(lanelet::InvalId, 5.0, -2.0));
  no_stopping_areas.push_back(poly);
  stop_line = generate_stop_line(path, no_stopping_areas, ego_width, stop_line_margin);
  EXPECT_FALSE(stop_line.has_value());
  // area on the path, 0 margin and 0 width
  poly.push_back(lanelet::Point3d(lanelet::InvalId, 5.0, -1.0));
  poly.push_back(lanelet::Point3d(lanelet::InvalId, 5.0, 1.0));
  poly.push_back(lanelet::Point3d(lanelet::InvalId, 6.0, 1.0));
  poly.push_back(lanelet::Point3d(lanelet::InvalId, 6.0, -1.0));
  no_stopping_areas.push_back(poly);
  stop_line = generate_stop_line(path, no_stopping_areas, ego_width, stop_line_margin);
  EXPECT_TRUE(stop_line.has_value());
  ASSERT_EQ(stop_line->size(), 2UL);
  EXPECT_EQ(stop_line->front().x(), 5.0);
  EXPECT_EQ(stop_line->front().y(), 0.0);
  EXPECT_EQ(stop_line->back().x(), 5.0);
  EXPECT_EQ(stop_line->back().y(), 0.0);
  // set a margin
  stop_line_margin = 1.0;
  stop_line = generate_stop_line(path, no_stopping_areas, ego_width, stop_line_margin);
  EXPECT_TRUE(stop_line.has_value());
  ASSERT_EQ(stop_line->size(), 2UL);
  EXPECT_EQ(stop_line->front().x(), 4.0);
  EXPECT_EQ(stop_line->front().y(), 0.0);
  EXPECT_EQ(stop_line->back().x(), 4.0);
  EXPECT_EQ(stop_line->back().y(), 0.0);
  // set a width
  ego_width = 2.0;
  stop_line = generate_stop_line(path, no_stopping_areas, ego_width, stop_line_margin);
  EXPECT_TRUE(stop_line.has_value());
  ASSERT_EQ(stop_line->size(), 2UL);
  // TODO(anyone): if we stop at this stop line ego overlaps the 1st area, is this okay ?
  EXPECT_EQ(stop_line->front().x(), 4.0);
  EXPECT_EQ(stop_line->front().y(), 2.0);
  EXPECT_EQ(stop_line->back().x(), 4.0);
  EXPECT_EQ(stop_line->back().y(), -2.0);
}

TEST(NoStoppingAreaTest, isStoppable)
{
  using autoware::behavior_velocity_planner::no_stopping_area::is_stoppable;
  autoware::behavior_velocity_planner::no_stopping_area::PassJudge pass_judge;
  geometry_msgs::msg::Pose self_pose;
  geometry_msgs::msg::Pose line_pose;
  rclcpp::Clock clock;
  auto logger = rclcpp::get_logger("test_logger");
  logger.set_level(rclcpp::Logger::Level::Debug);
  autoware::behavior_velocity_planner::no_stopping_area::EgoData ego_data;
  ego_data.current_velocity = 1.0;
  ego_data.current_acceleration = 0.0;
  ego_data.delay_response_time = 0.0;
  ego_data.max_stop_acc = -1.0;
  ego_data.max_stop_jerk = -1.0;
  // try to stop 1m ahead
  self_pose.position.x = 0.0;
  self_pose.position.y = 0.0;
  line_pose.position.x = 1.0;
  line_pose.position.y = 0.0;

  // enough distance to stop and slow velocity
  EXPECT_TRUE(is_stoppable(pass_judge, self_pose, line_pose, ego_data, logger, clock));
  EXPECT_TRUE(pass_judge.is_stoppable);
  EXPECT_FALSE(pass_judge.pass_judged);

  // unstoppable and fast velocity
  ego_data.current_velocity = 10.0;
  EXPECT_FALSE(is_stoppable(pass_judge, self_pose, line_pose, ego_data, logger, clock));
  EXPECT_FALSE(pass_judge.is_stoppable);
  EXPECT_TRUE(pass_judge.pass_judged);

  // unstoppable and slow velocity, but since we already judged, it is not updated
  ego_data.current_velocity = 1.9;
  EXPECT_FALSE(is_stoppable(pass_judge, self_pose, line_pose, ego_data, logger, clock));
  EXPECT_FALSE(pass_judge.is_stoppable);
  EXPECT_TRUE(pass_judge.pass_judged);

  // reset and result is updated
  // TODO(someone): is this the correct behavior ? distance is unstoppable but result is stoppable
  pass_judge.pass_judged = false;
  EXPECT_TRUE(is_stoppable(pass_judge, self_pose, line_pose, ego_data, logger, clock));
  EXPECT_TRUE(pass_judge.is_stoppable);
  EXPECT_TRUE(pass_judge.pass_judged);
}
