// Copyright 2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__PATH_SAFETY_CHECKER_PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__PATH_SAFETY_CHECKER_PARAMETERS_HPP_

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace behavior_path_planner::utils::path_safety_checker
{

using geometry_msgs::msg::Pose;
using tier4_autoware_utils::Polygon2d;

struct PoseWithVelocity
{
  Pose pose;
  double velocity{0.0};

  PoseWithVelocity(const Pose & pose, const double velocity) : pose(pose), velocity(velocity) {}
};

struct PoseWithVelocityStamped : public PoseWithVelocity
{
  double time{0.0};

  PoseWithVelocityStamped(const double time, const Pose & pose, const double velocity)
  : PoseWithVelocity(pose, velocity), time(time)
  {
  }
};

struct PoseWithVelocityAndPolygonStamped : public PoseWithVelocityStamped
{
  Polygon2d poly;

  PoseWithVelocityAndPolygonStamped(
    const double time, const Pose & pose, const double velocity, const Polygon2d & poly)
  : PoseWithVelocityStamped(time, pose, velocity), poly(poly)
  {
  }
};

struct PredictedPathWithPolygon
{
  float confidence{0.0};
  std::vector<PoseWithVelocityAndPolygonStamped> path;
};

struct ExtendedPredictedObject
{
  unique_identifier_msgs::msg::UUID uuid;
  geometry_msgs::msg::PoseWithCovariance initial_pose;
  geometry_msgs::msg::TwistWithCovariance initial_twist;
  geometry_msgs::msg::AccelWithCovariance initial_acceleration;
  autoware_auto_perception_msgs::msg::Shape shape;
  std::vector<PredictedPathWithPolygon> predicted_paths;
};

struct ObjectTypesToCheck
{
  bool check_car{true};
  bool check_truck{true};
  bool check_bus{true};
  bool check_trailer{true};
  bool check_unknown{true};
  bool check_bicycle{true};
  bool check_motorcycle{true};
  bool check_pedestrian{true};
};

struct ObjectLaneConfiguration
{
  bool check_current_lane{};
  bool check_right_lane{};
  bool check_left_lane{};
  bool check_shoulder_lane{};
  bool check_other_lane{};
};
struct TargetObjectsOnLane
{
  std::vector<ExtendedPredictedObject> on_current_lane{};
  std::vector<ExtendedPredictedObject> on_right_lane{};
  std::vector<ExtendedPredictedObject> on_left_lane{};
  std::vector<ExtendedPredictedObject> on_shoulder_lane{};
  std::vector<ExtendedPredictedObject> on_other_lane{};
};

struct RSSparams
{
  double rear_vehicle_reaction_time{0.0};
  double rear_vehicle_safety_time_margin{0.0};
  double lateral_distance_max_threshold{0.0};
  double longitudinal_distance_min_threshold{0.0};
  double longitudinal_velocity_delta_time{0.0};
};

struct EgoPredictedPath
{
  // for ego predicted path generation
  double acceleration;
  double time_horizon;
  double time_resolution;
  double min_slow_speed;
  double delay_until_departure;
  double target_velocity;
};

struct ObjectFilteringParams
{
  // for filtering dynamic objects to check
  double safety_check_time_horizon;
  double safety_check_time_resolution;
  double object_check_forward_distance;
  double object_check_backward_distance;
  double ignore_object_velocity_threshold;
  ObjectTypesToCheck object_types_to_check;
  ObjectLaneConfiguration object_lane_configuration;
  bool include_opposite_lane;
  bool invert_opposite_lane;
  bool check_all_predicted_path;
  bool use_all_predicted_path;
  bool use_predicted_path_outside_lanelet;
};

struct SafetyCheckParams
{
  bool enable_safety_check;
  // Trajectory generation parameters
  double backward_lane_length;
  double forward_path_length;

  RSSparams rss_params{};

  // Debug marker publishing option
  bool publish_debug_marker{false};
};

}  // namespace behavior_path_planner::utils::path_safety_checker

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__PATH_SAFETY_CHECKER_PARAMETERS_HPP_
