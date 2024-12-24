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
#include "autoware/obstacle_cruise_planner/type_alias.hpp"
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
  geometry_msgs::msg::Pose ego_pose;
  double ego_vel;
  double ego_acc;
  bool is_driving_forward;
};

struct PoseWithStamp
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;
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

struct TargetObstacleInterface
{
  TargetObstacleInterface(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const geometry_msgs::msg::Pose & arg_pose, const double arg_velocity,
    const double arg_lat_velocity)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_velocity),
    lat_velocity(arg_lat_velocity)
  {
  }
  std::string uuid;
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
  double velocity;                // longitudinal velocity against ego's trajectory
  double lat_velocity;            // lateral velocity against ego's trajectory
};

struct StopObstacle : public TargetObstacleInterface
{
  StopObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const ObjectClassification & object_classification, const geometry_msgs::msg::Pose & arg_pose,
    const Shape & arg_shape, const double arg_lon_velocity, const double arg_lat_velocity,
    const geometry_msgs::msg::Point arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj)
  : TargetObstacleInterface(arg_uuid, arg_stamp, arg_pose, arg_lon_velocity, arg_lat_velocity),
    shape(arg_shape),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification(object_classification)
  {
  }
  Shape shape;
  geometry_msgs::msg::Point
    collision_point;  // TODO(yuki_takagi): this member variable still used in
                      // calculateMarginFromObstacleOnCurve() and  should be removed as it can be
                      // replaced by ”dist_to_collide_on_decimated_traj”
  double dist_to_collide_on_decimated_traj;
  ObjectClassification classification;
};

struct CruiseObstacle : public TargetObstacleInterface
{
  CruiseObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const geometry_msgs::msg::Pose & arg_pose, const double arg_lon_velocity,
    const double arg_lat_velocity, const std::vector<PointWithStamp> & arg_collision_points,
    bool arg_is_yield_obstacle = false)
  : TargetObstacleInterface(arg_uuid, arg_stamp, arg_pose, arg_lon_velocity, arg_lat_velocity),
    collision_points(arg_collision_points),
    is_yield_obstacle(arg_is_yield_obstacle)
  {
  }
  std::vector<PointWithStamp> collision_points;  // time-series collision points
  bool is_yield_obstacle;
};

struct SlowDownObstacle : public TargetObstacleInterface
{
  SlowDownObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const ObjectClassification & object_classification, const geometry_msgs::msg::Pose & arg_pose,
    const double arg_lon_velocity, const double arg_lat_velocity, const double arg_precise_lat_dist,
    const geometry_msgs::msg::Point & arg_front_collision_point,
    const geometry_msgs::msg::Point & arg_back_collision_point)
  : TargetObstacleInterface(arg_uuid, arg_stamp, arg_pose, arg_lon_velocity, arg_lat_velocity),
    precise_lat_dist(arg_precise_lat_dist),
    front_collision_point(arg_front_collision_point),
    back_collision_point(arg_back_collision_point),
    classification(object_classification)
  {
  }
  double precise_lat_dist;  // for efficient calculation
  geometry_msgs::msg::Point front_collision_point;
  geometry_msgs::msg::Point back_collision_point;
  ObjectClassification classification;
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
    slow_down_min_accel = node.declare_parameter<double>("common.slow_down_min_acc");
    slow_down_min_jerk = node.declare_parameter<double>("common.slow_down_min_jerk");

    idling_time = node.declare_parameter<double>("common.idling_time");
    min_ego_accel_for_rss = node.declare_parameter<double>("common.min_ego_accel_for_rss");
    min_object_accel_for_rss = node.declare_parameter<double>("common.min_object_accel_for_rss");

    safe_distance_margin = node.declare_parameter<double>("common.safe_distance_margin");
    terminal_safe_distance_margin =
      node.declare_parameter<double>("common.terminal_safe_distance_margin");

    hold_stop_velocity_threshold =
      node.declare_parameter<double>("common.hold_stop_velocity_threshold");
    hold_stop_distance_threshold =
      node.declare_parameter<double>("common.hold_stop_distance_threshold");
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
    autoware::universe_utils::updateParam<double>(
      parameters, "common.slow_down_min_accel", slow_down_min_accel);
    autoware::universe_utils::updateParam<double>(
      parameters, "common.slow_down_min_jerk", slow_down_min_jerk);

    autoware::universe_utils::updateParam<double>(parameters, "common.idling_time", idling_time);
    autoware::universe_utils::updateParam<double>(
      parameters, "common.min_ego_accel_for_rss", min_ego_accel_for_rss);
    autoware::universe_utils::updateParam<double>(
      parameters, "common.min_object_accel_for_rss", min_object_accel_for_rss);

    autoware::universe_utils::updateParam<double>(
      parameters, "common.safe_distance_margin", safe_distance_margin);
    autoware::universe_utils::updateParam<double>(
      parameters, "common.terminal_safe_distance_margin", terminal_safe_distance_margin);

    autoware::universe_utils::updateParam<double>(
      parameters, "common.hold_stop_velocity_threshold", hold_stop_velocity_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters, "common.hold_stop_distance_threshold", hold_stop_distance_threshold);
  }

  // common parameter
  double max_accel;
  double min_accel;
  double max_jerk;
  double min_jerk;
  double slow_down_min_jerk;
  double slow_down_min_accel;
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

  // hold stop
  double hold_stop_velocity_threshold;
  double hold_stop_distance_threshold;
};

struct DebugData
{
  DebugData() = default;
  std::vector<Obstacle> intentionally_ignored_obstacles;
  std::vector<StopObstacle> obstacles_to_stop;
  std::vector<CruiseObstacle> obstacles_to_cruise;
  std::vector<SlowDownObstacle> obstacles_to_slow_down;
  MarkerArray slow_down_debug_wall_marker;
  MarkerArray stop_wall_marker;
  MarkerArray cruise_wall_marker;
  MarkerArray slow_down_wall_marker;
  std::vector<autoware::universe_utils::Polygon2d> detection_polygons;
  std::optional<std::vector<Metric>> stop_metrics{std::nullopt};
  std::optional<std::vector<Metric>> slow_down_metrics{std::nullopt};
  std::optional<std::vector<Metric>> cruise_metrics{std::nullopt};
};

struct EgoNearestParam
{
  EgoNearestParam() = default;
  explicit EgoNearestParam(rclcpp::Node & node)
  {
    dist_threshold = node.declare_parameter<double>("ego_nearest_dist_threshold");
    yaw_threshold = node.declare_parameter<double>("ego_nearest_yaw_threshold");
  }

  TrajectoryPoint calcInterpolatedPoint(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return autoware::motion_utils::calcInterpolatedPoint(
      autoware::motion_utils::convertToTrajectory(traj_points), pose, dist_threshold,
      yaw_threshold);
  }

  size_t findIndex(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_points, pose, dist_threshold, yaw_threshold);
  }

  size_t findSegmentIndex(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, pose, dist_threshold, yaw_threshold);
  }

  double dist_threshold;
  double yaw_threshold;
};

struct BehaviorDeterminationParam
{
  BehaviorDeterminationParam() = default;

  explicit BehaviorDeterminationParam(rclcpp::Node & node)
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
    obstacle_velocity_threshold_from_cruise_to_stop = node.declare_parameter<double>(
      "behavior_determination.obstacle_velocity_threshold_from_cruise_to_stop");
    obstacle_velocity_threshold_from_stop_to_cruise = node.declare_parameter<double>(
      "behavior_determination.obstacle_velocity_threshold_from_stop_to_cruise");
    crossing_obstacle_velocity_threshold = node.declare_parameter<double>(
      "behavior_determination.crossing_obstacle.obstacle_velocity_threshold");
    crossing_obstacle_traj_angle_threshold = node.declare_parameter<double>(
      "behavior_determination.crossing_obstacle.obstacle_traj_angle_threshold");
    collision_time_margin = node.declare_parameter<double>(
      "behavior_determination.stop.crossing_obstacle.collision_time_margin");
    outside_obstacle_min_velocity_threshold = node.declare_parameter<double>(
      "behavior_determination.cruise.outside_obstacle.obstacle_velocity_threshold");
    ego_obstacle_overlap_time_threshold = node.declare_parameter<double>(
      "behavior_determination.cruise.outside_obstacle.ego_obstacle_overlap_time_threshold");
    max_prediction_time_for_collision_check = node.declare_parameter<double>(
      "behavior_determination.cruise.outside_obstacle.max_prediction_time_for_collision_check");
    stop_obstacle_hold_time_threshold =
      node.declare_parameter<double>("behavior_determination.stop_obstacle_hold_time_threshold");
    prediction_resampling_time_interval =
      node.declare_parameter<double>("behavior_determination.prediction_resampling_time_interval");
    prediction_resampling_time_horizon =
      node.declare_parameter<double>("behavior_determination.prediction_resampling_time_horizon");

    max_lat_margin_for_stop =
      node.declare_parameter<double>("behavior_determination.stop.max_lat_margin");
    max_lat_margin_for_stop_against_unknown =
      node.declare_parameter<double>("behavior_determination.stop.max_lat_margin_against_unknown");
    max_lat_margin_for_cruise =
      node.declare_parameter<double>("behavior_determination.cruise.max_lat_margin");
    enable_yield = node.declare_parameter<bool>("behavior_determination.cruise.yield.enable_yield");
    yield_lat_distance_threshold =
      node.declare_parameter<double>("behavior_determination.cruise.yield.lat_distance_threshold");
    max_lat_dist_between_obstacles = node.declare_parameter<double>(
      "behavior_determination.cruise.yield.max_lat_dist_between_obstacles");
    stopped_obstacle_velocity_threshold = node.declare_parameter<double>(
      "behavior_determination.cruise.yield.stopped_obstacle_velocity_threshold");
    max_obstacles_collision_time = node.declare_parameter<double>(
      "behavior_determination.cruise.yield.max_obstacles_collision_time");
    max_lat_margin_for_slow_down =
      node.declare_parameter<double>("behavior_determination.slow_down.max_lat_margin");
    lat_hysteresis_margin_for_slow_down =
      node.declare_parameter<double>("behavior_determination.slow_down.lat_hysteresis_margin");
    successive_num_to_entry_slow_down_condition = node.declare_parameter<int>(
      "behavior_determination.slow_down.successive_num_to_entry_slow_down_condition");
    successive_num_to_exit_slow_down_condition = node.declare_parameter<int>(
      "behavior_determination.slow_down.successive_num_to_exit_slow_down_condition");
    enable_to_consider_current_pose = node.declare_parameter<bool>(
      "behavior_determination.consider_current_pose.enable_to_consider_current_pose");
    time_to_convergence = node.declare_parameter<double>(
      "behavior_determination.consider_current_pose.time_to_convergence");
    min_velocity_to_reach_collision_point = node.declare_parameter<double>(
      "behavior_determination.stop.min_velocity_to_reach_collision_point");
    max_lat_time_margin_for_stop = node.declare_parameter<double>(
      "behavior_determination.stop.outside_obstacle.max_lateral_time_margin");
    max_lat_time_margin_for_cruise = node.declare_parameter<double>(
      "behavior_determination.cruise.outside_obstacle.max_lateral_time_margin");
    num_of_predicted_paths_for_outside_cruise_obstacle = node.declare_parameter<int>(
      "behavior_determination.cruise.outside_obstacle.num_of_predicted_paths");
    num_of_predicted_paths_for_outside_stop_obstacle = node.declare_parameter<int>(
      "behavior_determination.stop.outside_obstacle.num_of_predicted_paths");
    pedestrian_deceleration_rate = node.declare_parameter<double>(
      "behavior_determination.stop.outside_obstacle.pedestrian_deceleration_rate");
    bicycle_deceleration_rate = node.declare_parameter<double>(
      "behavior_determination.stop.outside_obstacle.bicycle_deceleration_rate");
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
      parameters, "behavior_determination.stop.crossing_obstacle.collision_time_margin",
      collision_time_margin);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.cruise.outside_obstacle.obstacle_velocity_threshold",
      outside_obstacle_min_velocity_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters,
      "behavior_determination.cruise.outside_obstacle.ego_obstacle_overlap_time_threshold",
      ego_obstacle_overlap_time_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters,
      "behavior_determination.cruise.outside_obstacle.max_prediction_time_for_collision_check",
      max_prediction_time_for_collision_check);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.stop_obstacle_hold_time_threshold",
      stop_obstacle_hold_time_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.prediction_resampling_time_interval",
      prediction_resampling_time_interval);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.prediction_resampling_time_horizon",
      prediction_resampling_time_horizon);

    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.stop.max_lat_margin", max_lat_margin_for_stop);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.stop.max_lat_margin_against_unknown",
      max_lat_margin_for_stop_against_unknown);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.cruise.max_lat_margin", max_lat_margin_for_cruise);
    autoware::universe_utils::updateParam<bool>(
      parameters, "behavior_determination.cruise.yield.enable_yield", enable_yield);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.cruise.yield.lat_distance_threshold",
      yield_lat_distance_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.cruise.yield.max_lat_dist_between_obstacles",
      max_lat_dist_between_obstacles);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.cruise.yield.stopped_obstacle_velocity_threshold",
      stopped_obstacle_velocity_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.cruise.yield.max_obstacles_collision_time",
      max_obstacles_collision_time);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.slow_down.max_lat_margin", max_lat_margin_for_slow_down);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.slow_down.lat_hysteresis_margin",
      lat_hysteresis_margin_for_slow_down);
    autoware::universe_utils::updateParam<int>(
      parameters, "behavior_determination.slow_down.successive_num_to_entry_slow_down_condition",
      successive_num_to_entry_slow_down_condition);
    autoware::universe_utils::updateParam<int>(
      parameters, "behavior_determination.slow_down.successive_num_to_exit_slow_down_condition",
      successive_num_to_exit_slow_down_condition);
    autoware::universe_utils::updateParam<bool>(
      parameters, "behavior_determination.consider_current_pose.enable_to_consider_current_pose",
      enable_to_consider_current_pose);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.consider_current_pose.time_to_convergence",
      time_to_convergence);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.stop.min_velocity_to_reach_collision_point",
      min_velocity_to_reach_collision_point);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.stop.outside_obstacle.max_lateral_time_margin",
      max_lat_time_margin_for_stop);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.cruise.outside_obstacle.max_lateral_time_margin",
      max_lat_time_margin_for_cruise);
    autoware::universe_utils::updateParam<int>(
      parameters, "behavior_determination.cruise.outside_obstacle.num_of_predicted_paths",
      num_of_predicted_paths_for_outside_cruise_obstacle);
    autoware::universe_utils::updateParam<int>(
      parameters, "behavior_determination.stop.outside_obstacle.num_of_predicted_paths",
      num_of_predicted_paths_for_outside_stop_obstacle);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.stop.outside_obstacle.pedestrian_deceleration_rate",
      pedestrian_deceleration_rate);
    autoware::universe_utils::updateParam<double>(
      parameters, "behavior_determination.stop.outside_obstacle.bicycle_deceleration_rate",
      bicycle_deceleration_rate);
  }

  double decimate_trajectory_step_length;
  double pointcloud_search_radius;
  double pointcloud_voxel_grid_x;
  double pointcloud_voxel_grid_y;
  double pointcloud_voxel_grid_z;
  double pointcloud_cluster_tolerance;
  int pointcloud_min_cluster_size;
  int pointcloud_max_cluster_size;
  // hysteresis for stop and cruise
  double obstacle_velocity_threshold_from_cruise_to_stop;
  double obstacle_velocity_threshold_from_stop_to_cruise;
  // inside
  double crossing_obstacle_velocity_threshold;
  double collision_time_margin;
  // outside
  double outside_obstacle_min_velocity_threshold;
  double ego_obstacle_overlap_time_threshold;
  double max_prediction_time_for_collision_check;
  double crossing_obstacle_traj_angle_threshold;
  int num_of_predicted_paths_for_outside_cruise_obstacle;
  int num_of_predicted_paths_for_outside_stop_obstacle;
  double pedestrian_deceleration_rate;
  double bicycle_deceleration_rate;
  // obstacle hold
  double stop_obstacle_hold_time_threshold;
  // reach collision point
  double min_velocity_to_reach_collision_point;
  // prediction resampling
  double prediction_resampling_time_interval;
  double prediction_resampling_time_horizon;
  // max lateral time margin
  double max_lat_time_margin_for_stop;
  double max_lat_time_margin_for_cruise;
  // max lateral margin
  double max_lat_margin_for_stop;
  double max_lat_margin_for_stop_against_unknown;
  double max_lat_margin_for_cruise;
  double max_lat_margin_for_slow_down;
  double lat_hysteresis_margin_for_slow_down;
  int successive_num_to_entry_slow_down_condition;
  int successive_num_to_exit_slow_down_condition;
  // consideration for the current ego pose
  double time_to_convergence{1.5};
  bool enable_to_consider_current_pose{false};
  // yield related parameters
  bool enable_yield{false};
  double yield_lat_distance_threshold;
  double max_lat_dist_between_obstacles;
  double stopped_obstacle_velocity_threshold;
  double max_obstacles_collision_time;
};

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_
