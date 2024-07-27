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

#include "../src/object_filtering.hpp"

#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/LineString.h>

TEST(TestObjectFiltering, isVehicle)
{
  using autoware::motion_velocity_planner::dynamic_obstacle_stop::is_vehicle;
  using autoware_perception_msgs::msg::ObjectClassification;
  autoware_perception_msgs::msg::PredictedObject object;
  ObjectClassification classification;
  object.classification = {};
  EXPECT_FALSE(is_vehicle(object));
  classification.label = ObjectClassification::PEDESTRIAN;
  object.classification.push_back(classification);
  EXPECT_FALSE(is_vehicle(object));
  classification.label = ObjectClassification::UNKNOWN;
  object.classification.push_back(classification);
  EXPECT_FALSE(is_vehicle(object));
  classification.label = ObjectClassification::TRAILER;
  object.classification.push_back(classification);
  EXPECT_TRUE(is_vehicle(object));
  object.classification.clear();
  for (const auto label :
       {ObjectClassification::CAR, ObjectClassification::BUS, ObjectClassification::BICYCLE,
        ObjectClassification::TRUCK, ObjectClassification::TRAILER,
        ObjectClassification::MOTORCYCLE}) {
    classification.label = label;
    object.classification = {classification};
    EXPECT_TRUE(is_vehicle(object));
  }
}

TEST(TestObjectFiltering, isInRange)
{
  using autoware::motion_velocity_planner::dynamic_obstacle_stop::is_in_range;
  autoware_perception_msgs::msg::PredictedObject object;
  autoware::motion_velocity_planner::dynamic_obstacle_stop::TrajectoryPoints ego_trajectory;
  autoware_planning_msgs::msg::TrajectoryPoint trajectory_p;
  autoware::motion_velocity_planner::dynamic_obstacle_stop::PlannerParam params;
  trajectory_p.pose.position.y = 0.0;
  for (auto x = -10.0; x <= 10.0; x += 1.0) {
    trajectory_p.pose.position.x = x;
    ego_trajectory.push_back(trajectory_p);
  }
  // object 4m from the ego trajectory
  object.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 4.0;
  object.shape.dimensions.y = 2.0;
  params.minimum_object_distance_from_ego_trajectory = 1.0;
  params.ego_lateral_offset = 1.0;
  double hysteresis = 0.0;
  EXPECT_FALSE(is_in_range(object, ego_trajectory, params, hysteresis));
  // object 3m from the ego trajectory
  object.kinematics.initial_pose_with_covariance.pose.position.y = 3.0;
  EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
  hysteresis = 1.0;
  EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
  // object 2m from the ego trajectory
  object.kinematics.initial_pose_with_covariance.pose.position.y = 2.0;
  hysteresis = 0.0;
  EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
  hysteresis = 1.0;
  EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
  // object exactly on the trajectory
  object.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  for (auto ego_lat_offset = 0.1; ego_lat_offset < 2.0; ego_lat_offset += 0.1) {
    for (auto object_width = 0.1; object_width < 2.0; object_width += 0.1) {
      for (auto min_dist = 0.1; min_dist < 2.0; min_dist += 0.1) {
        for (hysteresis = 0.1; hysteresis < 2.0; hysteresis += 0.1) {
          params.ego_lateral_offset = ego_lat_offset;
          object.shape.dimensions.y = object_width;
          params.minimum_object_distance_from_ego_trajectory = min_dist;
          EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
        }
      }
    }
  }
}

TEST(TestObjectFiltering, isNotTooClose)
{
  using autoware::motion_velocity_planner::dynamic_obstacle_stop::is_not_too_close;

  double ego_longitudinal_offset = 1.0;
  autoware::motion_velocity_planner::dynamic_obstacle_stop::EgoData ego_data;
  ego_data.longitudinal_offset_to_first_trajectory_idx = 0.0;
  ego_data.pose.position.x = 0.0;
  ego_data.pose.position.y = 0.0;
  autoware_planning_msgs::msg::TrajectoryPoint trajectory_p;
  trajectory_p.pose.position.y = 0.0;
  for (auto x = -10.0; x <= 10.0; x += 1.0) {
    trajectory_p.pose.position.x = x;
    ego_data.trajectory.push_back(trajectory_p);
  }
  autoware_perception_msgs::msg::PredictedObject object;
  object.shape.dimensions.x = 2.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 2.0;
  // object ego with 1m offset = too close if poses are within 2m of arc length
  for (auto obj_x = -2.0; obj_x <= 2.0; obj_x += 0.1) {
    object.kinematics.initial_pose_with_covariance.pose.position.x = obj_x;
    EXPECT_FALSE(is_not_too_close(object, ego_data, ego_longitudinal_offset));
  }
  for (auto obj_x = -2.1; obj_x >= -10.0; obj_x -= 0.1) {
    object.kinematics.initial_pose_with_covariance.pose.position.x = obj_x;
    EXPECT_TRUE(is_not_too_close(object, ego_data, ego_longitudinal_offset));
  }
  for (auto obj_x = 2.1; obj_x <= 10.0; obj_x += 0.1) {
    object.kinematics.initial_pose_with_covariance.pose.position.x = obj_x;
    EXPECT_TRUE(is_not_too_close(object, ego_data, ego_longitudinal_offset));
  }
}

TEST(TestObjectFiltering, isUnavoidable)
{
}
