// Copyright 2024 Tier IV, Inc. All rights reserved.
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

#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

using autoware::universe_utils::Point2d;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;
using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;

using autoware::test_utils::createPose;
using autoware::test_utils::generateTrajectory;

TEST(BehaviorPathPlanningUtilTest, l2Norm)
{
  using autoware::behavior_path_planner::utils::l2Norm;

  geometry_msgs::msg::Vector3 vector = autoware::universe_utils::createVector3(0.0, 0.0, 0.0);
  auto norm = l2Norm(vector);
  EXPECT_DOUBLE_EQ(norm, 0.0);

  vector = autoware::universe_utils::createVector3(1.0, 2.0, 2.0);
  norm = l2Norm(vector);
  EXPECT_DOUBLE_EQ(norm, 3.0);
}

TEST(BehaviorPathPlanningUtilTest, checkCollisionBetweenPathFootprintsAndObjects)
{
  using autoware::behavior_path_planner::utils::checkCollisionBetweenPathFootprintsAndObjects;

  autoware::universe_utils::LinearRing2d base_footprint = {
    Point2d{1.0, 1.0}, Point2d{1.0, -1.0}, Point2d{-1.0, -1.0}, Point2d{-1.0, 1.0},
    Point2d{1.0, -1.0}};
  double margin = 0.2;
  PredictedObjects objs;
  PredictedObject obj;
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = 2.0;
  obj.kinematics.initial_pose_with_covariance.pose = createPose(9.0, 1.0, 0.0, 0.0, 0.0, 0.0);
  objs.objects.push_back(obj);

  PathWithLaneId ego_path;

  // Condition: no path
  EXPECT_FALSE(
    checkCollisionBetweenPathFootprintsAndObjects(base_footprint, ego_path, objs, margin));

  // Condition: object in front of path
  ego_path = generateTrajectory<PathWithLaneId>(5, 1.0);
  EXPECT_FALSE(
    checkCollisionBetweenPathFootprintsAndObjects(base_footprint, ego_path, objs, margin));

  // Condition: object overlapping path
  ego_path = generateTrajectory<PathWithLaneId>(10, 1.0);
  EXPECT_TRUE(
    checkCollisionBetweenPathFootprintsAndObjects(base_footprint, ego_path, objs, margin));
}

TEST(BehaviorPathPlanningUtilTest, checkCollisionBetweenFootprintAndObjects)
{
  using autoware::behavior_path_planner::utils::checkCollisionBetweenFootprintAndObjects;

  auto ego_pose = createPose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0);
  autoware::universe_utils::LinearRing2d base_footprint = {
    Point2d{1.0, 1.0}, Point2d{1.0, -1.0}, Point2d{-1.0, -1.0}, Point2d{-1.0, 1.0},
    Point2d{1.0, -1.0}};
  double margin = 0.2;
  PredictedObjects objs;

  // Condition: no object
  EXPECT_FALSE(checkCollisionBetweenFootprintAndObjects(base_footprint, ego_pose, objs, margin));

  // Condition: no collision
  PredictedObject obj;
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = 2.0;
  obj.kinematics.initial_pose_with_covariance.pose = createPose(9.0, 9.0, 0.0, 0.0, 0.0, 0.0);
  objs.objects.push_back(obj);
  EXPECT_FALSE(checkCollisionBetweenFootprintAndObjects(base_footprint, ego_pose, objs, margin));

  // Condition: collision
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 1.0;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 1.0;
  objs.objects.push_back(obj);
  EXPECT_TRUE(checkCollisionBetweenFootprintAndObjects(base_footprint, ego_pose, objs, margin));
}

TEST(BehaviorPathPlanningUtilTest, calcLateralDistanceFromEgoToObject)
{
  using autoware::behavior_path_planner::utils::calcLateralDistanceFromEgoToObject;

  auto ego_pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  double vehicle_width = 2.0;

  PredictedObject obj;
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = 2.0;

  // Condition: overlapping
  obj.kinematics.initial_pose_with_covariance.pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(calcLateralDistanceFromEgoToObject(ego_pose, vehicle_width, obj), 0.0);

  // Condition: object on left
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 5.0;
  EXPECT_DOUBLE_EQ(calcLateralDistanceFromEgoToObject(ego_pose, vehicle_width, obj), 3.0);

  // Condition: object on right
  obj.kinematics.initial_pose_with_covariance.pose.position.y = -5.0;
  EXPECT_DOUBLE_EQ(calcLateralDistanceFromEgoToObject(ego_pose, vehicle_width, obj), 3.0);
}

TEST(BehaviorPathPlanningUtilTest, calc_longitudinal_distance_from_ego_to_object)
{
  using autoware::behavior_path_planner::utils::calc_longitudinal_distance_from_ego_to_object;

  auto ego_pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  PredictedObject obj;
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = 1.0;

  // Condition: overlapping
  double base_link2front = 0.0;
  double base_link2rear = 0.0;
  obj.kinematics.initial_pose_with_covariance.pose = createPose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(
    calc_longitudinal_distance_from_ego_to_object(ego_pose, base_link2front, base_link2rear, obj),
    0.0);

  // Condition: object in front
  base_link2front = 1.0;
  base_link2rear = -1.0;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 4.0;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 2.0;
  EXPECT_DOUBLE_EQ(
    calc_longitudinal_distance_from_ego_to_object(ego_pose, base_link2front, base_link2rear, obj),
    2.0);

  // Condition: object in rear
  obj.kinematics.initial_pose_with_covariance.pose.position.x = -4.0;
  EXPECT_DOUBLE_EQ(
    calc_longitudinal_distance_from_ego_to_object(ego_pose, base_link2front, base_link2rear, obj),
    2.0);
}

TEST(BehaviorPathPlanningUtilTest, calcLongitudinalDistanceFromEgoToObjects)
{
  using autoware::behavior_path_planner::utils::calcLongitudinalDistanceFromEgoToObjects;

  auto ego_pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  double base_link2front = 1.0;
  double base_link2rear = -1.0;

  PredictedObjects objs;

  // Condition: none object
  EXPECT_DOUBLE_EQ(
    calcLongitudinalDistanceFromEgoToObjects(ego_pose, base_link2front, base_link2rear, objs),
    std::numeric_limits<double>::max());

  // Condition: both object in front
  PredictedObject near_obj;
  near_obj.kinematics.initial_pose_with_covariance.pose = createPose(5.0, 2.0, 0.0, 0.0, 0.0, 0.0);
  near_obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  near_obj.shape.dimensions.x = 2.0;
  near_obj.shape.dimensions.y = 1.0;

  PredictedObject far_obj;
  far_obj.kinematics.initial_pose_with_covariance.pose = createPose(25.0, 2.0, 0.0, 0.0, 0.0, 0.0);
  far_obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  far_obj.shape.dimensions.x = 5.0;
  far_obj.shape.dimensions.y = 1.0;

  objs.objects.push_back(near_obj);
  objs.objects.push_back(far_obj);
  EXPECT_DOUBLE_EQ(
    calcLongitudinalDistanceFromEgoToObjects(ego_pose, base_link2front, base_link2rear, objs), 3.0);
}

TEST(BehaviorPathPlanningUtilTest, getHighestProbLabel)
{
  using autoware::behavior_path_planner::utils::getHighestProbLabel;

  PredictedObject obj;
  ObjectClassification classification;

  // Condition: no classification
  EXPECT_EQ(getHighestProbLabel(obj.classification), ObjectClassification::Type::UNKNOWN);

  // Condition: with 2 label
  obj.classification.emplace_back(autoware_perception_msgs::build<ObjectClassification>()
                                    .label(ObjectClassification::CAR)
                                    .probability(0.4));
  obj.classification.emplace_back(autoware_perception_msgs::build<ObjectClassification>()
                                    .label(ObjectClassification::TRUCK)
                                    .probability(0.6));
  EXPECT_EQ(getHighestProbLabel(obj.classification), ObjectClassification::Type::TRUCK);
}
