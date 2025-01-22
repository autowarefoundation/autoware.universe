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
    const double approach_velocity)
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
    shape(object.shape)
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

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_
