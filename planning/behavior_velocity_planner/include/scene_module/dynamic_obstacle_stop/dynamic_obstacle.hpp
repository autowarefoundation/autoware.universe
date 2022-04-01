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

#ifndef DYNAMIC_OBSTACLE_HPP
#define DYNAMIC_OBSTACLE_HPP

#include "scene_module/dynamic_obstacle_stop/utils.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <geometry_msgs/msg/twist.h>

#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::Shape;
using autoware_auto_planning_msgs::msg::Trajectory;
using dynamic_obstacle_stop_utils::DynamicObstacleParam;
using dynamic_obstacle_stop_utils::PoseWithRange;
using dynamic_obstacle_stop_utils::PredictedPath;

class DynamicObstacle
{
public:
  DynamicObstacle();
  explicit DynamicObstacle(const DynamicObstacleParam & param);
  void createDynamicObstacle(
    const geometry_msgs::msg::Point & point, const Trajectory & trajectory);
  void createDynamicObstacle(const autoware_auto_perception_msgs::msg::PredictedObject & object);
  void createDynamicObstacle(
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const Trajectory & trajectory);

  //  obstacle information
  geometry_msgs::msg::Pose pose_;
  std::vector<geometry_msgs::msg::Point> collision_points_;
  geometry_msgs::msg::Point nearest_collision_point_;
  float min_velocity_mps_;
  float max_velocity_mps_;
  std::vector<ObjectClassification> classification_;
  Shape shape_;
  std::vector<PredictedPath> predicted_paths_;

private:
  geometry_msgs::msg::Quaternion createQuaternionFacingToTrajectory(
    const geometry_msgs::msg::Point & point, const Trajectory & trajectory) const;

  std::vector<geometry_msgs::msg::Pose> createPredictedPath(
    const geometry_msgs::msg::Pose & initial_pose, const float time_step,
    const float max_velocity_mps, const size_t path_size) const;

  // params
  DynamicObstacleParam param_;
};
}  // namespace behavior_velocity_planner

#endif  // DYNAMIC_OBSTACLE_HPP
