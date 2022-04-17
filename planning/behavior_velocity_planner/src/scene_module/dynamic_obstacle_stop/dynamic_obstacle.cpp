// Copyright 2022 Tier IV, Inc.
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

#include "scene_module/dynamic_obstacle_stop/dynamic_obstacle.hpp"

namespace behavior_velocity_planner
{
namespace
{
// create quaternion facing to the nearest trajectory point
geometry_msgs::msg::Quaternion createQuaternionFacingToTrajectory(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & point)
{
  const auto nearest_idx = tier4_autoware_utils::findNearestIndex(path_points, point);
  const auto & nearest_pose = path_points.at(nearest_idx).point.pose;

  const auto longitudinal_offset =
    tier4_autoware_utils::calcLongitudinalDeviation(nearest_pose, point);
  const auto vertical_point =
    tier4_autoware_utils::calcOffsetPose(nearest_pose, longitudinal_offset, 0, 0).position;
  const auto azimuth_angle = tier4_autoware_utils::calcAzimuthAngle(point, vertical_point);

  return tier4_autoware_utils::createQuaternionFromYaw(azimuth_angle);
}
}  // namespace

DynamicObstacle::DynamicObstacle() {}
DynamicObstacle::DynamicObstacle(const DynamicObstacleParam & param) { param_ = param; }

void DynamicObstacle::createDynamicObstacle(
  const geometry_msgs::msg::Point & point, const PathWithLaneId & path)
{
  // create pose facing the direction of the lane
  pose_.position = point;
  pose_.orientation = createQuaternionFacingToTrajectory(path.points, pose_.position);

  min_velocity_mps_ = tier4_autoware_utils::kmph2mps(param_.min_vel_kmph);
  max_velocity_mps_ = tier4_autoware_utils::kmph2mps(param_.max_vel_kmph);

  // create classification of points as pedestrian
  ObjectClassification classification;
  classification.label = ObjectClassification::PEDESTRIAN;
  classification.probability = 1.0;
  classification_.emplace_back(classification);

  // create shape of points as cylinder
  shape_.type = Shape::CYLINDER;
  shape_.dimensions.x = param_.diameter;
  shape_.dimensions.y = param_.diameter;
  shape_.dimensions.z = param_.height;

  // create predicted path of points
  PredictedPath predicted_path;
  predicted_path.path =
    createPredictedPath(pose_, param_.time_step, max_velocity_mps_, param_.path_size);
  predicted_path.confidence = 1.0;
  predicted_paths_.emplace_back(predicted_path);
}

// overwrite path of objects to run straight to the lane
void DynamicObstacle::createDynamicObstacle(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const PathWithLaneId & path)
{
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;

  // create pose facing the direction of the lane
  pose_.position = object_pose.position;
  pose_.orientation = createQuaternionFacingToTrajectory(path.points, pose_.position);

  // assume min and max velocity is value specified in param
  min_velocity_mps_ = tier4_autoware_utils::kmph2mps(param_.min_vel_kmph);
  max_velocity_mps_ = tier4_autoware_utils::kmph2mps(param_.max_vel_kmph);

  // use classification of objects
  classification_ = object.classification;

  // use shape of objects
  shape_ = object.shape;

  // replace predicted path with path that runs straight to lane
  PredictedPath predicted_path;
  predicted_path.path =
    createPredictedPath(pose_, param_.time_step, max_velocity_mps_, param_.path_size);
  predicted_path.confidence = 1.0;
  predicted_paths_.emplace_back(predicted_path);
}

void DynamicObstacle::createDynamicObstacle(
  const autoware_auto_perception_msgs::msg::PredictedObject & object)
{
  pose_ = object.kinematics.initial_pose_with_covariance.pose;

  // TODO(Tomohito Ando): calculate from velocity and covariance of object
  min_velocity_mps_ = param_.min_vel_kmph / 3.6;
  max_velocity_mps_ = param_.max_vel_kmph / 3.6;
  classification_ = object.classification;
  shape_ = object.shape;

  // get predicted paths of objects
  for (const auto & path : object.kinematics.predicted_paths) {
    PredictedPath predicted_path;
    predicted_path.confidence = path.confidence;
    predicted_path.path.resize(path.path.size());
    std::copy(path.path.cbegin(), path.path.cend(), predicted_path.path.begin());

    predicted_paths_.emplace_back(predicted_path);
  }
}

// create predicted path assuming that obstacles move with constant velocity
std::vector<geometry_msgs::msg::Pose> DynamicObstacle::createPredictedPath(
  const geometry_msgs::msg::Pose & initial_pose, const float time_step,
  const float max_velocity_mps, const size_t path_size) const
{
  std::vector<geometry_msgs::msg::Pose> path_points;
  for (size_t i = 0; i < path_size; i++) {
    const float travel_dist = max_velocity_mps * time_step * i;
    const auto predicted_pose =
      tier4_autoware_utils::calcOffsetPose(initial_pose, travel_dist, 0, 0);
    path_points.emplace_back(predicted_pose);
  }

  return path_points;
}
}  // namespace behavior_velocity_planner
