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

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using PredictedObject = autoware_perception_msgs::msg::PredictedObject;
using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;
using PoseWithCovariance = geometry_msgs::msg::PoseWithCovariance;
using TwistWithCovariance = geometry_msgs::msg::TwistWithCovariance;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_planning_msgs::msg::Trajectory;
using tier4_planning_msgs::msg::PathPointWithLaneId;

using autoware::test_utils::createPose;
using autoware::test_utils::generateTrajectory;
using autoware::universe_utils::createPoint;

constexpr double epsilon = 1e-6;

std::vector<PathPointWithLaneId> trajectory_to_path_with_lane_id(const Trajectory & trajectory)
{
  std::vector<PathPointWithLaneId> path_with_lane_id;
  PathPointWithLaneId path_point_with_lane_id;
  for (const auto & point : trajectory.points) {
    path_point_with_lane_id.point.pose = point.pose;
    path_point_with_lane_id.point.lateral_velocity_mps = point.lateral_velocity_mps;
    path_point_with_lane_id.point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    path_point_with_lane_id.point.heading_rate_rps = point.heading_rate_rps;
    path_with_lane_id.push_back(path_point_with_lane_id);
  }
  return path_with_lane_id;
}

PredictedPath trajectory_to_predicted_path(const Trajectory & trajectory)
{
  PredictedPath path;
  geometry_msgs::msg::Pose pose;
  for (const auto & point : trajectory.points) {
    pose = point.pose;
    path.path.push_back(pose);
  }
  return path;
}

TEST(BehaviorPathPlanningObjectsFiltering, velocity_filter)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::filter::velocity_filter;

  PredictedObject predicted_obj;
  predicted_obj.kinematics.initial_twist_with_covariance.twist.linear.x = 4.0;
  predicted_obj.kinematics.initial_twist_with_covariance.twist.linear.y = 3.0;

  EXPECT_TRUE(velocity_filter(predicted_obj, 4.0, 10.0));
  EXPECT_FALSE(velocity_filter(predicted_obj, 6.0, 10.0));
  EXPECT_FALSE(velocity_filter(predicted_obj, 2.0, 4.9));
  EXPECT_FALSE(velocity_filter(predicted_obj, 6.0, 2.0));
}

TEST(BehaviorPathPlanningObjectsFiltering, position_filter)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::filter::position_filter;

  auto current_pos = createPoint(0.0, 0.0, 0.0);
  PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose = createPose(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  auto straight_trajectory = generateTrajectory<Trajectory>(20, 1.0);
  double forward_distance = 20.0;
  double backward_distance = 1.0;

  std::vector<PathPointWithLaneId> empty_path;
  auto straight_path = trajectory_to_path_with_lane_id(generateTrajectory<Trajectory>(20, 1.0));
  auto curved_path =
    trajectory_to_path_with_lane_id(generateTrajectory<Trajectory>(20, 1.0, 0.0, 0.0, 0.01));

  EXPECT_TRUE(
    position_filter(object, straight_path, current_pos, forward_distance, -backward_distance));
  EXPECT_FALSE(
    position_filter(object, empty_path, current_pos, forward_distance, -backward_distance));
  EXPECT_TRUE(
    position_filter(object, curved_path, current_pos, forward_distance, -backward_distance));

  forward_distance = 2.0;
  EXPECT_FALSE(
    position_filter(object, straight_path, current_pos, forward_distance, -backward_distance));
  EXPECT_FALSE(
    position_filter(object, curved_path, current_pos, forward_distance, -backward_distance));
}

TEST(BehaviorPathPlanningObjectsFiltering, is_within_circle)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::filter::is_within_circle;

  auto object_pos = createPoint(0.0, 0.0, 0.0);
  auto ref_point = createPoint(0.0, 0.0, 0.0);
  double search_radius = 1.0;

  EXPECT_TRUE(is_within_circle(object_pos, ref_point, search_radius));

  object_pos.x = 2.0;
  object_pos.x = 2.0;

  EXPECT_FALSE(is_within_circle(object_pos, ref_point, search_radius));

  ref_point.x = 2.5;
  ref_point.y = 2.4;
  object_pos.x = 3.0;
  object_pos.y = 3.0;

  EXPECT_TRUE(is_within_circle(object_pos, ref_point, search_radius));
}

TEST(BehaviorPathPlanningObjectsFiltering, filterObjectsByVelocity)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::filterObjectsByVelocity;

  PredictedObjects objects;
  PredictedObject slow_obj;
  PredictedObject fast_obj;
  double vel_thr = 5.0;

  slow_obj.object_id = autoware::universe_utils::generateUUID();
  slow_obj.kinematics.initial_twist_with_covariance.twist.linear.x = 2.0;
  slow_obj.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;
  objects.objects.push_back(slow_obj);

  fast_obj.object_id = autoware::universe_utils::generateUUID();
  fast_obj.kinematics.initial_twist_with_covariance.twist.linear.x = 10.0;
  fast_obj.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;
  objects.objects.push_back(fast_obj);

  auto filtered_obj = filterObjectsByVelocity(objects, vel_thr, false);
  ASSERT_FALSE(filtered_obj.objects.empty());
  EXPECT_EQ(filtered_obj.objects.front().object_id, slow_obj.object_id);

  filtered_obj = filterObjectsByVelocity(objects, vel_thr, true);
  ASSERT_FALSE(filtered_obj.objects.empty());
  EXPECT_EQ(filtered_obj.objects.front().object_id, fast_obj.object_id);

  vel_thr = 0.0;
  filtered_obj = filterObjectsByVelocity(objects, vel_thr, false);
  EXPECT_TRUE(filtered_obj.objects.empty());

  filtered_obj = filterObjectsByVelocity(objects, vel_thr, true);
  EXPECT_EQ(filtered_obj.objects.size(), 2);
}

TEST(BehaviorPathPlanningObjectsFiltering, filterObjectsByPosition)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::filterObjectsByPosition;
  using autoware::behavior_path_planner::utils::path_safety_checker::filterObjectsWithinRadius;

  auto current_pos = createPoint(0.0, 0.0, 0.0);
  auto straight_path = trajectory_to_path_with_lane_id(generateTrajectory<Trajectory>(20, 1.0));
  double forward_distance = 10.0;
  double backward_distance = 1.0;
  double search_radius = 10.0;

  PredictedObjects objects;
  PredictedObject far_obj;
  PredictedObject near_obj;

  near_obj.object_id = autoware::universe_utils::generateUUID();
  near_obj.kinematics.initial_pose_with_covariance.pose = createPose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  objects.objects.push_back(near_obj);
  auto target_uuid = near_obj.object_id;

  far_obj.object_id = autoware::universe_utils::generateUUID();
  far_obj.kinematics.initial_pose_with_covariance.pose = createPose(50.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  objects.objects.push_back(far_obj);

  filterObjectsByPosition(objects, straight_path, current_pos, forward_distance, backward_distance);

  ASSERT_FALSE(objects.objects.empty());
  EXPECT_EQ(objects.objects.front().object_id, target_uuid);

  objects.objects.clear();
  objects.objects.push_back(far_obj);
  objects.objects.push_back(near_obj);

  filterObjectsWithinRadius(objects, current_pos, search_radius);
  ASSERT_FALSE(objects.objects.empty());
  EXPECT_EQ(objects.objects.front().object_id, target_uuid);
}

TEST(BehaviorPathPlanningObjectsFiltering, createPredictedPath)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::createPredictedPath;

  std::shared_ptr<
    autoware::behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams>
    param = std::make_shared<
      autoware::behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams>();

  param->min_velocity = -1.0;
  param->acceleration = 0.1;
  param->max_velocity = 5.0;
  param->time_horizon_for_front_object = 2.0;
  param->time_horizon_for_rear_object = 1.0;
  param->time_resolution = 0.5;
  param->delay_until_departure = 0.1;

  auto vehicle_pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  std::vector<PathPointWithLaneId> empty_path;
  auto predicted_path = createPredictedPath(param, empty_path, vehicle_pose, 0.0, 0, false, false);
  EXPECT_TRUE(predicted_path.empty());

  size_t i = 0;
  auto straight_path = trajectory_to_path_with_lane_id(generateTrajectory<Trajectory>(20, 1.0));
  predicted_path = createPredictedPath(param, straight_path, vehicle_pose, 0.0, 0, true, false);

  ASSERT_EQ(predicted_path.size(), 4);
  for (const auto & point : predicted_path) {
    auto time =
      std::max(0.0, param->time_resolution * static_cast<double>(i) - param->delay_until_departure);
    EXPECT_NEAR(point.time, i * param->time_resolution, epsilon);
    EXPECT_NEAR(point.pose.position.x, std::pow(time, 2.0) * param->acceleration * 0.5, epsilon);
    EXPECT_NEAR(point.velocity, time * param->acceleration, epsilon);
    i++;
  }

  i = 0;
  predicted_path = createPredictedPath(param, straight_path, vehicle_pose, 0.0, 0, false, false);

  ASSERT_EQ(predicted_path.size(), 2);
  for (const auto & point : predicted_path) {
    auto time =
      std::max(0.0, param->time_resolution * static_cast<double>(i) - param->delay_until_departure);
    EXPECT_NEAR(point.time, i * param->time_resolution, epsilon);
    EXPECT_NEAR(point.pose.position.x, std::pow(time, 2.0) * param->acceleration * 0.5, epsilon);
    EXPECT_NEAR(point.velocity, time * param->acceleration, epsilon);
    i++;
  }
}

TEST(BehaviorPathPlanningObjectsFiltering, transform)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::transform;
  auto velocity = autoware::universe_utils::createVector3(2.0, 0.0, 0.0);
  auto angular = autoware::universe_utils::createVector3(0.0, 0.0, 0.0);

  PredictedObject obj;
  obj.object_id = autoware::universe_utils::generateUUID();
  obj.kinematics.initial_pose_with_covariance.pose = createPose(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  obj.kinematics.initial_twist_with_covariance.twist =
    autoware::universe_utils::createTwist(velocity, angular);
  autoware_perception_msgs::msg::PredictedPath predicted_path;
  auto straight_path = trajectory_to_predicted_path(generateTrajectory<Trajectory>(5, 1.0));
  straight_path.confidence = 0.6;
  straight_path.time_step.sec = 1.0;
  obj.kinematics.predicted_paths.push_back(straight_path);
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = 1.0;
  obj.shape.dimensions.z = 1.0;

  double safety_check_time_horizon = 2.0;
  double safety_check_time_resolution = 0.5;

  auto extended_obj = transform(obj, safety_check_time_horizon, safety_check_time_resolution);
  EXPECT_NEAR(
    extended_obj.predicted_paths.front().confidence,
    obj.kinematics.predicted_paths.front().confidence, epsilon);
  ASSERT_FALSE(extended_obj.predicted_paths.empty());
  EXPECT_EQ(extended_obj.predicted_paths.front().path.size(), 5);

  size_t i = 0;
  for (const auto & point : extended_obj.predicted_paths.front().path) {
    EXPECT_NEAR(point.pose.position.x, i * 0.5, epsilon);
    EXPECT_NEAR(point.pose.position.y, 0.0, epsilon);
    EXPECT_NEAR(point.velocity, velocity.x, epsilon);
    ASSERT_EQ(point.poly.outer().size(), 5);

    EXPECT_NEAR(
      point.poly.outer().at(0).x(), point.pose.position.x + 0.5 * obj.shape.dimensions.x, epsilon);
    EXPECT_NEAR(
      point.poly.outer().at(0).y(), point.pose.position.y + 0.5 * obj.shape.dimensions.y, epsilon);

    EXPECT_NEAR(
      point.poly.outer().at(1).x(), point.pose.position.x + 0.5 * obj.shape.dimensions.x, epsilon);
    EXPECT_NEAR(
      point.poly.outer().at(1).y(), point.pose.position.y - 0.5 * obj.shape.dimensions.y, epsilon);

    EXPECT_NEAR(
      point.poly.outer().at(2).x(), point.pose.position.x - 0.5 * obj.shape.dimensions.x, epsilon);
    EXPECT_NEAR(
      point.poly.outer().at(2).y(), point.pose.position.y - 0.5 * obj.shape.dimensions.y, epsilon);

    EXPECT_NEAR(
      point.poly.outer().at(3).x(), point.pose.position.x - 0.5 * obj.shape.dimensions.x, epsilon);
    EXPECT_NEAR(
      point.poly.outer().at(3).y(), point.pose.position.y + 0.5 * obj.shape.dimensions.y, epsilon);
    i++;
  }
}

TEST(BehaviorPathPlanningObjectsFiltering, filterObjectsByClass)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::filterObjectsByClass;

  PredictedObjects objects;
  PredictedObject car;
  PredictedObject truck;
  PredictedObject pedestrian;
  ObjectClassification classification;
  autoware::behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck types_to_check;

  car.object_id = autoware::universe_utils::generateUUID();
  classification.label = ObjectClassification::Type::CAR;
  classification.probability = 1.0;
  car.classification.push_back(classification);
  objects.objects.push_back(car);

  truck.object_id = autoware::universe_utils::generateUUID();
  classification.label = ObjectClassification::Type::TRUCK;
  classification.probability = 1.0;
  truck.classification.push_back(classification);
  objects.objects.push_back(truck);

  pedestrian.object_id = autoware::universe_utils::generateUUID();
  classification.label = ObjectClassification::Type::PEDESTRIAN;
  classification.probability = 1.0;
  pedestrian.classification.push_back(classification);
  objects.objects.push_back(pedestrian);

  filterObjectsByClass(objects, types_to_check);
  EXPECT_EQ(objects.objects.size(), 3);

  types_to_check.check_pedestrian = false;
  filterObjectsByClass(objects, types_to_check);
  EXPECT_EQ(objects.objects.size(), 2);

  types_to_check.check_car = false;
  types_to_check.check_truck = false;
  filterObjectsByClass(objects, types_to_check);
  EXPECT_TRUE(objects.objects.empty());
}

TEST(BehaviorPathPlanningObjectsFiltering, isTargetObjectType)
{
  using autoware::behavior_path_planner::utils::path_safety_checker::isTargetObjectType;

  autoware::behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck types_to_check;
  PredictedObject obj;
  ObjectClassification classification;
  classification.label = ObjectClassification::Type::CAR;
  classification.probability = 0.6;
  obj.classification.push_back(classification);
  classification.label = ObjectClassification::Type::TRUCK;
  classification.probability = 0.4;
  obj.classification.push_back(classification);

  EXPECT_TRUE(isTargetObjectType(obj, types_to_check));
  types_to_check.check_car = false;
  EXPECT_FALSE(isTargetObjectType(obj, types_to_check));

  obj.classification.front().probability = 0.4;
  obj.classification.at(1).probability = 0.6;
  EXPECT_TRUE(isTargetObjectType(obj, types_to_check));

  obj.classification.at(1).label = ObjectClassification::Type::PEDESTRIAN;
  EXPECT_TRUE(isTargetObjectType(obj, types_to_check));
}
