// Copyright 2022 TIER IV, Inc.
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

#ifndef OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_
#define OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_

#include "obstacle_cruise_planner/type_alias.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/optional.hpp>

#include <string>
#include <vector>

struct TargetObstacle
{
  TargetObstacle(
    const rclcpp::Time & arg_time_stamp, const PredictedObject & object,
    const double aligned_velocity,
    const std::vector<geometry_msgs::msg::PointStamped> & arg_collision_points)
  {
    time_stamp = arg_time_stamp;
    orientation_reliable = true;
    pose = object.kinematics.initial_pose_with_covariance.pose;
    velocity_reliable = true;
    velocity = aligned_velocity;
    classification = object.classification.at(0);
    uuid = tier4_autoware_utils::toHexString(object.object_id);

    predicted_paths.clear();
    for (const auto & path : object.kinematics.predicted_paths) {
      predicted_paths.push_back(path);
    }

    collision_points = arg_collision_points;
    has_stopped = false;
  }

  rclcpp::Time time_stamp;
  geometry_msgs::msg::Pose pose;
  bool orientation_reliable;
  double velocity;
  bool velocity_reliable;
  ObjectClassification classification;
  std::string uuid;
  std::vector<PredictedPath> predicted_paths;
  std::vector<geometry_msgs::msg::PointStamped> collision_points;
  bool has_stopped;
};

struct ObstacleCruisePlannerData
{
  rclcpp::Time current_time;
  Trajectory traj;
  geometry_msgs::msg::Pose current_pose;
  double current_vel;
  double current_acc;
  std::vector<TargetObstacle> target_obstacles;
  bool is_driving_forward;
};

struct LongitudinalInfo
{
  explicit LongitudinalInfo(rclcpp::Node & node)
  {
    max_accel = node.declare_parameter("normal.max_acc").get<double>();
    min_accel = node.declare_parameter("normal.min_acc").get<double>();
    max_jerk = node.declare_parameter("normal.max_jerk").get<double>();
    min_jerk = node.declare_parameter("normal.min_jerk").get<double>();
    limit_max_accel = node.declare_parameter("limit.max_acc").get<double>();
    limit_min_accel = node.declare_parameter("limit.min_acc").get<double>();
    limit_max_jerk = node.declare_parameter("limit.max_jerk").get<double>();
    limit_min_jerk = node.declare_parameter("limit.min_jerk").get<double>();

    idling_time = node.declare_parameter("common.idling_time").get<double>();
    min_ego_accel_for_rss = node.declare_parameter("common.min_ego_accel_for_rss").get<double>();
    min_object_accel_for_rss = node.declare_parameter("common.min_object_accel_for_rss").get<double>();

    safe_distance_margin = node.declare_parameter("common.safe_distance_margin").get<double>();
    terminal_safe_distance_margin =
      node.declare_parameter("common.terminal_safe_distance_margin").get<double>();
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    tier4_autoware_utils::updateParam<double>(parameters, "normal.max_accel", max_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "normal.min_accel", min_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "normal.max_jerk", max_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "normal.min_jerk", min_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.max_accel", limit_max_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.min_accel", limit_min_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.max_jerk", limit_max_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.min_jerk", limit_min_jerk);

    tier4_autoware_utils::updateParam<double>(parameters, "common.idling_time", idling_time);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.min_ego_accel_for_rss", min_ego_accel_for_rss);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.min_object_accel_for_rss", min_object_accel_for_rss);

    tier4_autoware_utils::updateParam<double>(
      parameters, "common.safe_distance_margin", safe_distance_margin);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.terminal_safe_distance_margin", terminal_safe_distance_margin);
  }

  // common parameter
  double max_accel;
  double min_accel;
  double max_jerk;
  double min_jerk;
  double limit_max_accel;
  double limit_min_accel;
  double limit_max_jerk;
  double limit_min_jerk;

  // rss parameter
  double idling_time;
  double min_ego_accel_for_rss;
  double min_object_accel_for_rss;

  // distance margin
  double safe_distance_margin;
  double terminal_safe_distance_margin;
};

struct DebugData
{
  std::vector<PredictedObject> intentionally_ignored_obstacles;
  std::vector<TargetObstacle> obstacles_to_stop;
  std::vector<TargetObstacle> obstacles_to_cruise;
  MarkerArray stop_wall_marker;
  MarkerArray cruise_wall_marker;
  std::vector<tier4_autoware_utils::Polygon2d> detection_polygons;
  std::vector<geometry_msgs::msg::Point> collision_points;
};

struct EgoNearestParam
{
  EgoNearestParam() = default;
  explicit EgoNearestParam(rclcpp::Node & node)
  {
    dist_threshold = node.declare_parameter("ego_nearest_dist_threshold").get<double>();
    yaw_threshold = node.declare_parameter("ego_nearest_yaw_threshold").get<double>();
  }

  double dist_threshold;
  double yaw_threshold;
};

#endif  // OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_
