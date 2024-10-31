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

#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <gtest/gtest.h>

StopObstacle generate_labeled_null_stop_obstacle(uint8_t label)
{
  const std::string uuid{};
  const rclcpp::Time time{};
  ObjectClassification object_classification{};
  object_classification.label = label;
  const geometry_msgs::msg::Pose pose{};
  const Shape shape{};
  const double lon_velocity{};
  const double lat_velocity{};
  const geometry_msgs::msg::Point collision_point{};
  const double dist{10.0};

  return StopObstacle{uuid,         time,         object_classification, pose, shape,
                      lon_velocity, lat_velocity, collision_point,       dist};
}

TEST(ObstacleCruisePlannerUtilsTest, getClosestStopObstacles)
{
  using autoware_perception_msgs::msg::ObjectClassification;

  std::vector<StopObstacle> stop_obstacles;
  EXPECT_EQ(0, obstacle_cruise_utils::getClosestStopObstacles(stop_obstacles).size());

  stop_obstacles.emplace_back(generate_labeled_null_stop_obstacle(ObjectClassification::UNKNOWN));
  EXPECT_EQ(1, obstacle_cruise_utils::getClosestStopObstacles(stop_obstacles).size());

  stop_obstacles.emplace_back(generate_labeled_null_stop_obstacle(ObjectClassification::UNKNOWN));
  EXPECT_EQ(1, obstacle_cruise_utils::getClosestStopObstacles(stop_obstacles).size());

  stop_obstacles.emplace_back(generate_labeled_null_stop_obstacle(ObjectClassification::CAR));
  EXPECT_EQ(2, obstacle_cruise_utils::getClosestStopObstacles(stop_obstacles).size());

  stop_obstacles.emplace_back(generate_labeled_null_stop_obstacle(ObjectClassification::BUS));
  EXPECT_EQ(3, obstacle_cruise_utils::getClosestStopObstacles(stop_obstacles).size());
}
