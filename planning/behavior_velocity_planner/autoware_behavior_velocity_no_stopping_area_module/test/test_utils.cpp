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

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <tier4_planning_msgs/msg/detail/path_point_with_lane_id__struct.hpp>

#include <gtest/gtest.h>

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
