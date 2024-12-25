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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_

#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/obstacle_cruise_planner/stop/type_alias.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "autoware/universe_utils/ros/uuid_helper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <optional>
#include <string>
#include <vector>

using Metric = tier4_metric_msgs::msg::Metric;
using MetricArray = tier4_metric_msgs::msg::MetricArray;

struct PlannerData
{
  rclcpp::Time current_time;
  std::vector<TrajectoryPoint> traj_points;
  nav_msgs::msg::Odometry current_odometry;
  geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration;
  // geometry_msgs::msg::Pose ego_pose;
  // double ego_vel;
  // double ego_acc;
  bool is_driving_forward;
  VehicleInfo vehicle_info;
  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;

  size_t findIndex(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_points, pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  }

  size_t findSegmentIndex(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  }
};

struct PointWithStamp
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Point point;
};

struct Obstacle
{
  Obstacle(
    const rclcpp::Time & arg_stamp, const PredictedObject & object,
    const geometry_msgs::msg::Pose & arg_pose, const double ego_to_obstacle_distance,
    const double lat_dist_from_obstacle_to_traj, const double longitudinal_velocity,
    const double approach_velocity, const double arg_precise_lat_dist)
  : stamp(arg_stamp),
    ego_to_obstacle_distance(ego_to_obstacle_distance),
    lat_dist_from_obstacle_to_traj(lat_dist_from_obstacle_to_traj),
    longitudinal_velocity(longitudinal_velocity),
    approach_velocity(approach_velocity),
    pose(arg_pose),
    orientation_reliable(true),
    twist(object.kinematics.initial_twist_with_covariance.twist),
    twist_reliable(true),
    classification(object.classification.at(0)),
    uuid(autoware::universe_utils::toHexString(object.object_id)),
    shape(object.shape),
    precise_lat_dist(arg_precise_lat_dist)
  {
    predicted_paths.clear();
    for (const auto & path : object.kinematics.predicted_paths) {
      predicted_paths.push_back(path);
    }
  }

  Obstacle(
    const rclcpp::Time & arg_stamp,
    const std::optional<geometry_msgs::msg::Point> & stop_collision_point,
    const std::optional<geometry_msgs::msg::Point> & slow_down_front_collision_point,
    const std::optional<geometry_msgs::msg::Point> & slow_down_back_collision_point,
    const double ego_to_obstacle_distance, const double lat_dist_from_obstacle_to_traj)
  : stamp(arg_stamp),
    ego_to_obstacle_distance(ego_to_obstacle_distance),
    lat_dist_from_obstacle_to_traj(lat_dist_from_obstacle_to_traj),
    precise_lat_dist(lat_dist_from_obstacle_to_traj),
    stop_collision_point(stop_collision_point),
    slow_down_front_collision_point(slow_down_front_collision_point),
    slow_down_back_collision_point(slow_down_back_collision_point)
  {
  }

  rclcpp::Time stamp;  // This is not the current stamp, but when the object was observed.
  double ego_to_obstacle_distance;
  double lat_dist_from_obstacle_to_traj;
  double longitudinal_velocity;
  double approach_velocity;

  // for PredictedObject
  geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
  bool orientation_reliable;
  Twist twist;
  bool twist_reliable;
  ObjectClassification classification;
  std::string uuid;
  Shape shape;
  double precise_lat_dist;
  std::vector<PredictedPath> predicted_paths;

  // for PointCloud
  std::optional<geometry_msgs::msg::Point> stop_collision_point;
  std::optional<geometry_msgs::msg::Point> slow_down_front_collision_point;
  std::optional<geometry_msgs::msg::Point> slow_down_back_collision_point;
};

struct LongitudinalInfo
{
  explicit LongitudinalInfo(rclcpp::Node & node)
  {
    max_accel = node.declare_parameter<double>("normal.max_acc");
    min_accel = node.declare_parameter<double>("normal.min_acc");
    max_jerk = node.declare_parameter<double>("normal.max_jerk");
    min_jerk = node.declare_parameter<double>("normal.min_jerk");
    limit_max_accel = node.declare_parameter<double>("limit.max_acc");
    limit_min_accel = node.declare_parameter<double>("limit.min_acc");
    limit_max_jerk = node.declare_parameter<double>("limit.max_jerk");
    limit_min_jerk = node.declare_parameter<double>("limit.min_jerk");

    idling_time = node.declare_parameter<double>("common.idling_time");
    min_ego_accel_for_rss = node.declare_parameter<double>("common.min_ego_accel_for_rss");
    min_object_accel_for_rss = node.declare_parameter<double>("common.min_object_accel_for_rss");

    safe_distance_margin = node.declare_parameter<double>("common.safe_distance_margin");
    terminal_safe_distance_margin =
      node.declare_parameter<double>("common.terminal_safe_distance_margin");
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    autoware::universe_utils::updateParam<double>(parameters, "normal.max_accel", max_accel);
    autoware::universe_utils::updateParam<double>(parameters, "normal.min_accel", min_accel);
    autoware::universe_utils::updateParam<double>(parameters, "normal.max_jerk", max_jerk);
    autoware::universe_utils::updateParam<double>(parameters, "normal.min_jerk", min_jerk);
    autoware::universe_utils::updateParam<double>(parameters, "limit.max_accel", limit_max_accel);
    autoware::universe_utils::updateParam<double>(parameters, "limit.min_accel", limit_min_accel);
    autoware::universe_utils::updateParam<double>(parameters, "limit.max_jerk", limit_max_jerk);
    autoware::universe_utils::updateParam<double>(parameters, "limit.min_jerk", limit_min_jerk);

    autoware::universe_utils::updateParam<double>(parameters, "common.idling_time", idling_time);
    autoware::universe_utils::updateParam<double>(
      parameters, "common.min_ego_accel_for_rss", min_ego_accel_for_rss);
    autoware::universe_utils::updateParam<double>(
      parameters, "common.min_object_accel_for_rss", min_object_accel_for_rss);

    autoware::universe_utils::updateParam<double>(
      parameters, "common.safe_distance_margin", safe_distance_margin);
    autoware::universe_utils::updateParam<double>(
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

struct CommonBehaviorDeterminationParam
{
  CommonBehaviorDeterminationParam() = default;

  explicit CommonBehaviorDeterminationParam(rclcpp::Node & node)
  {  // behavior determination
    decimate_trajectory_step_length =
      node.declare_parameter<double>("behavior_determination.decimate_trajectory_step_length");
    pointcloud_search_radius =
      node.declare_parameter<double>("behavior_determination.pointcloud_search_radius");
    pointcloud_voxel_grid_x =
      node.declare_parameter<double>("behavior_determination.pointcloud_voxel_grid_x");
    pointcloud_voxel_grid_y =
      node.declare_parameter<double>("behavior_determination.pointcloud_voxel_grid_y");
    pointcloud_voxel_grid_z =
      node.declare_parameter<double>("behavior_determination.pointcloud_voxel_grid_z");
    pointcloud_cluster_tolerance =
      node.declare_parameter<double>("behavior_determination.pointcloud_cluster_tolerance");
    pointcloud_min_cluster_size =
      node.declare_parameter<int>("behavior_determination.pointcloud_min_cluster_size");
    pointcloud_max_cluster_size =
      node.declare_parameter<int>("behavior_determination.pointcloud_max_cluster_size");
    crossing_obstacle_velocity_threshold = node.declare_parameter<double>(
      "behavior_determination.crossing_obstacle.obstacle_velocity_threshold");
    crossing_obstacle_traj_angle_threshold = node.declare_parameter<double>(
      "behavior_determination.crossing_obstacle.obstacle_traj_angle_threshold");
    prediction_resampling_time_interval =
      node.declare_parameter<double>("behavior_determination.prediction_resampling_time_interval");
    prediction_resampling_time_horizon =
      node.declare_parameter<double>("behavior_determination.prediction_resampling_time_horizon");

    enable_to_consider_current_pose = node.declare_parameter<bool>(
      "behavior_determination.consider_current_pose.enable_to_consider_current_pose");
    time_to_convergence = node.declare_parameter<double>(
      "behavior_determination.consider_current_pose.time_to_convergence");
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    // behavior determination
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.decimate_trajectory_step_length",
      decimate_trajectory_step_length);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.pointcloud_search_radius", pointcloud_search_radius);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.pointcloud_voxel_grid_x", pointcloud_voxel_grid_x);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.pointcloud_voxel_grid_y", pointcloud_voxel_grid_y);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.pointcloud_voxel_grid_z", pointcloud_voxel_grid_z);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.pointcloud_cluster_tolerance",
      pointcloud_cluster_tolerance);
    autoware::universe_utils::updateParam<int>(
      parameters, "behavior_determination.pointcloud_min_cluster_size",
      pointcloud_min_cluster_size);
    autoware::universe_utils::updateParam<int>(
      parameters, "behavior_determination.pointcloud_max_cluster_size",
      pointcloud_max_cluster_size);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.crossing_obstacle.obstacle_velocity_threshold",
      crossing_obstacle_velocity_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.crossing_obstacle.obstacle_traj_angle_threshold",
      crossing_obstacle_traj_angle_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.prediction_resampling_time_interval",
      prediction_resampling_time_interval);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.prediction_resampling_time_horizon",
      prediction_resampling_time_horizon);

    autoware::universe_utils::updateParam<bool>(
      parameters, "behavior_determination.consider_current_pose.enable_to_consider_current_pose",
      enable_to_consider_current_pose);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.consider_current_pose.time_to_convergence",
      time_to_convergence);
  }

  double decimate_trajectory_step_length;
  double pointcloud_search_radius;
  double pointcloud_voxel_grid_x;
  double pointcloud_voxel_grid_y;
  double pointcloud_voxel_grid_z;
  double pointcloud_cluster_tolerance;
  int pointcloud_min_cluster_size;
  int pointcloud_max_cluster_size;
  double crossing_obstacle_velocity_threshold;
  double crossing_obstacle_traj_angle_threshold;
  double prediction_resampling_time_interval;
  double prediction_resampling_time_horizon;

  bool enable_to_consider_current_pose;
  double time_to_convergence;
};

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_
