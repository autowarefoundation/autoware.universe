/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>

#include <boost/optional.hpp>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core.hpp>

#include "obstacle_avoidance_planner/debug.h"
#include "obstacle_avoidance_planner/eb_path_optimizer.h"
#include "obstacle_avoidance_planner/node.h"
#include "obstacle_avoidance_planner/spline_interpolate.h"
#include "obstacle_avoidance_planner/util.h"

namespace
{
template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_WARN("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}
}  // namespace

ObstacleAvoidancePlanner::ObstacleAvoidancePlanner()
: min_num_points_for_getting_yaw_(2), nh_(), pnh_("~")
{
  tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);

  trajectory_pub_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/path", 1);
  avoiding_traj_pub_ = pnh_.advertise<autoware_planning_msgs::Trajectory>(
    "/planning/scenario_planning/lane_driving/obstacle_avoidance_candidate_trajectory", 1, true);
  is_avoidance_possible_pub_ = pnh_.advertise<std_msgs::Bool>(
    "/planning/scenario_planning/lane_driving/obstacle_avoidance_ready", 1, true);
  debug_markers_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/marker", 1, true);
  debug_clearance_map_pub_ =
    pnh_.advertise<nav_msgs::OccupancyGrid>("debug/clearance_map", 1, true);
  debug_object_clearance_map_pub_ =
    pnh_.advertise<nav_msgs::OccupancyGrid>("debug/object_clearance_map", 1, true);
  debug_area_with_objects_pub_ =
    pnh_.advertise<nav_msgs::OccupancyGrid>("debug/area_with_objects", 1, true);

  path_sub_ = pnh_.subscribe("input/path", 1, &ObstacleAvoidancePlanner::pathCallback, this);
  twist_sub_ =
    pnh_.subscribe("/localization/twist", 1, &ObstacleAvoidancePlanner::twistCallback, this);
  objects_sub_ =
    pnh_.subscribe("input/objects", 10, &ObstacleAvoidancePlanner::objectsCallback, this);
  is_avoidance_sub_ = pnh_.subscribe(
    "/planning/scenario_planning/lane_driving/obstacle_avoidance_approval", 10,
    &ObstacleAvoidancePlanner::enableAvoidanceCallback, this);

  dynamic_reconfigure_srv_.setCallback(
    boost::bind(&ObstacleAvoidancePlanner::configCallback, this, _1, _2));

  pnh_.param<bool>("is_publishing_clearance_map", is_publishing_clearance_map_, false);
  pnh_.param<bool>("is_publishing_area_with_objects", is_publishing_area_with_objects_, false);
  pnh_.param<bool>("is_showing_debug_info", is_showing_debug_info_, true);
  pnh_.param<bool>("is_using_vehicle_config", is_using_vehicle_config_, false);
  pnh_.param<bool>("enable_avoidance", enable_avoidance_, true);

  qp_param_ = std::make_unique<QPParam>();
  traj_param_ = std::make_unique<TrajectoryParam>();
  constrain_param_ = std::make_unique<ConstrainParam>();
  pnh_.param<int>("qp_max_iteration", qp_param_->max_iteration, 10000);
  pnh_.param<double>("qp_eps_abs", qp_param_->eps_abs, 1.0e-8);
  pnh_.param<double>("qp_eps_rel", qp_param_->eps_rel, 1.0e-11);
  pnh_.param<double>("qp_eps_abs_for_extending", qp_param_->eps_abs_for_extending, 1.0e-6);
  pnh_.param<double>("qp_eps_rel_for_extending", qp_param_->eps_rel_for_extending, 1.0e-8);
  pnh_.param<double>("qp_eps_abs_for_visualizing", qp_param_->eps_abs_for_visualizing, 1.0e-6);
  pnh_.param<double>("qp_eps_rel_for_visualizing", qp_param_->eps_rel_for_visualizing, 1.0e-8);

  pnh_.param<int>("num_sampling_points", traj_param_->num_sampling_points, 100);
  pnh_.param<int>("num_joint_buffer_points", traj_param_->num_joint_buffer_points, 2);
  pnh_.param<int>("num_offset_for_begin_idx", traj_param_->num_offset_for_begin_idx, 2);
  pnh_.param<int>("num_fix_points_for_extending", traj_param_->num_fix_points_for_extending, 2);
  pnh_.param<double>(
    "delta_arc_length_for_optimization", traj_param_->delta_arc_length_for_optimization, 1.0);
  pnh_.param<double>(
    "delta_arc_length_for_trajectory", traj_param_->delta_arc_length_for_trajectory, 0.1);
  pnh_.param<double>(
    "delta_dist_threshold_for_closest_point", traj_param_->delta_dist_threshold_for_closest_point,
    3.0);
  pnh_.param<double>(
    "delta_yaw_threshold_for_closest_point", traj_param_->delta_yaw_threshold_for_closest_point,
    1.0);
  pnh_.param<double>(
    "delta_yaw_threshold_for_straight", traj_param_->delta_yaw_threshold_for_straight, 0.02);
  pnh_.param<double>("trajectory_length", traj_param_->trajectory_length, 200);
  pnh_.param<double>("forward_fixing_distance", traj_param_->forward_fixing_distance, 10.0);
  pnh_.param<double>("backward_fixing_distance", traj_param_->backward_fixing_distance, 5.0);
  pnh_.param<double>(
    "max_avoiding_ego_velocity_ms", traj_param_->max_avoiding_ego_velocity_ms, 6.0);
  pnh_.param<double>(
    "max_avoiding_objects_velocity_ms", traj_param_->max_avoiding_objects_velocity_ms, 0.1);
  pnh_.param<double>("center_line_width", traj_param_->center_line_width, 1.7);
  pnh_.param<double>(
    "acceleration_for_non_deceleration_range", traj_param_->acceleration_for_non_deceleration_range,
    1.0);

  pnh_.param<bool>(
    "is_getting_constraints_close2path_points",
    constrain_param_->is_getting_constraints_close2path_points, false);
  pnh_.param<double>(
    "clearance_for_straight_line_", constrain_param_->clearance_for_straight_line, 0.05);
  pnh_.param<double>("clearance_for_joint_", constrain_param_->clearance_for_joint, 3.2);
  pnh_.param<double>(
    "clearance_for_only_smoothing", constrain_param_->clearance_for_only_smoothing, 0.1);
  pnh_.param<double>(
    "clearance_from_object_for_straight", constrain_param_->clearance_from_object_for_straight,
    10.0);
  pnh_.param<double>("min_clearance_from_road", constrain_param_->min_clearance_from_road, 0.1);
  pnh_.param<double>("min_clearance_from_object", constrain_param_->min_clearance_from_object, 0.6);
  pnh_.param<double>(
    "min_object_clearance_for_joint", constrain_param_->min_object_clearance_for_joint, 3.2);
  pnh_.param<double>(
    "max_x_constrain_search_range", constrain_param_->max_x_constrain_search_range, 0.4);
  pnh_.param<double>(
    "coef_x_cosntrain_search_resolution", constrain_param_->coef_x_cosntrain_search_resolution,
    1.0);
  pnh_.param<double>(
    "coef_y_cosntrain_search_resolution", constrain_param_->coef_y_cosntrain_search_resolution,
    0.5);
  pnh_.param<double>("keep_space_shape_x", constrain_param_->keep_space_shape_x, 3.0);
  pnh_.param<double>("keep_space_shape_y", constrain_param_->keep_space_shape_y, 2.0);
  pnh_.param<double>(
    "max_lon_space_for_driveable_constraint",
    constrain_param_->max_lon_space_for_driveable_constraint, 0.5);
  constrain_param_->clearance_for_fixing = 0.0;

  pnh_.param<double>("min_delta_dist_for_replan", min_delta_dist_for_replan_, 5.0);
  pnh_.param<double>("min_delta_time_sec_for_replan", min_delta_time_sec_for_replan_, 1.0);
  pnh_.param<double>("max_dist_for_extending_end_point", max_dist_for_extending_end_point_, 5.0);
  pnh_.param<double>(
    "distance_for_path_shape_chagne_detection", distance_for_path_shape_chagne_detection_, 2.0);

  if (is_using_vehicle_config_) {
    double vehicle_width = waitForParam<double>(pnh_, "/vehicle_info/vehicle_width");
    traj_param_->center_line_width = vehicle_width;
    constrain_param_->keep_space_shape_y = vehicle_width;
  }
  constrain_param_->min_object_clearance_for_deceleration =
    constrain_param_->min_clearance_from_object + constrain_param_->keep_space_shape_y * 0.5;

  in_objects_ptr_ = std::make_unique<autoware_perception_msgs::DynamicObjectArray>();
  initialize();
}

ObstacleAvoidancePlanner::~ObstacleAvoidancePlanner() {}

// ROS callback functions
void ObstacleAvoidancePlanner::configCallback(
  const obstacle_avoidance_planner::AvoidancePlannerConfig & config, const uint32_t level)
{
  if (!eb_path_optimizer_ptr_) return;  // waiting for initialize

  std::lock_guard<std::mutex> lock(mutex_);

  // trajectory total/fixing length
  traj_param_->trajectory_length = config.trajectory_length;
  traj_param_->forward_fixing_distance = config.forward_fixing_distance;
  traj_param_->backward_fixing_distance = config.backward_fixing_distance;

  // clearance for unique points
  constrain_param_->clearance_for_straight_line = config.clearance_for_straight_line_;
  constrain_param_->clearance_for_joint = config.clearance_for_joint_;
  constrain_param_->clearance_for_only_smoothing = config.clearance_for_only_smoothing;
  constrain_param_->clearance_from_object_for_straight = config.clearance_from_object_for_straight;

  // clearance (distance) when generating trajectory
  constrain_param_->min_clearance_from_road = config.min_clearance_from_road;
  constrain_param_->min_clearance_from_object = config.min_clearance_from_object;
  constrain_param_->min_object_clearance_for_joint = config.min_object_clearance_for_joint;

  // avoiding param
  traj_param_->max_avoiding_ego_velocity_ms = config.max_avoiding_ego_velocity_ms;
  traj_param_->max_avoiding_objects_velocity_ms = config.max_avoiding_objects_velocity_ms;
  traj_param_->center_line_width = config.center_line_width;
  traj_param_->acceleration_for_non_deceleration_range =
    config.acceleration_for_non_deceleration_range;

  /* If any parameters change in dynamic reconfigure, ObstacleAvoidancePlanner reset
   * and delete previous path and previous trajectory 
   */
  ROS_WARN(
    "[ObstacleAvoidancePlanner] Resetting trajectory from changing parameters in dynamic "
    "reconfigure. Changing parameters when vehicle is moving may cause undefined behaviors.");
  initialize();
}

void ObstacleAvoidancePlanner::pathCallback(const autoware_planning_msgs::Path & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_ego_pose_ptr_ = getCurrentEgoPose();
  if (
    msg.points.empty() || msg.drivable_area.data.empty() || !current_ego_pose_ptr_ ||
    !current_twist_ptr_) {
    return;
  }
  autoware_planning_msgs::Trajectory output_trajectory_msg = generateTrajectory(msg);
  trajectory_pub_.publish(output_trajectory_msg);
}

void ObstacleAvoidancePlanner::twistCallback(const geometry_msgs::TwistStamped & msg)
{
  current_twist_ptr_ = std::make_unique<geometry_msgs::TwistStamped>(msg);
}

void ObstacleAvoidancePlanner::objectsCallback(
  const autoware_perception_msgs::DynamicObjectArray & msg)
{
  in_objects_ptr_ = std::make_unique<autoware_perception_msgs::DynamicObjectArray>(msg);
}

void ObstacleAvoidancePlanner::enableAvoidanceCallback(const std_msgs::Bool & msg)
{
  enable_avoidance_ = msg.data;
}
// End ROS callback functions

autoware_planning_msgs::Trajectory ObstacleAvoidancePlanner::generateTrajectory(
  const autoware_planning_msgs::Path & path)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  const auto traj_points = generateOptimizedTrajectory(*current_ego_pose_ptr_, path);

  const auto post_processed_traj =
    generatePostProcessedTrajectory(*current_ego_pose_ptr_, path.points, traj_points);

  autoware_planning_msgs::Trajectory output;
  output.header = path.header;
  output.points = post_processed_traj;

  prev_traj_points_ptr_ =
    std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>(output.points);
  prev_path_points_ptr_ =
    std::make_unique<std::vector<autoware_planning_msgs::PathPoint>>(path.points);

  auto t_end = std::chrono::high_resolution_clock::now();
  float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  ROS_INFO_COND(
    is_showing_debug_info_, "Total time: = %f [ms]\n==========================", elapsed_ms);
  return output;
}

std::vector<autoware_planning_msgs::TrajectoryPoint>
ObstacleAvoidancePlanner::generateOptimizedTrajectory(
  const geometry_msgs::Pose & ego_pose, const autoware_planning_msgs::Path & path)
{
  if (!needReplan(
        ego_pose, prev_ego_pose_ptr_, path.points, prev_replanned_time_ptr_, prev_path_points_ptr_,
        prev_traj_points_ptr_)) {
    return *prev_traj_points_ptr_;
  }
  prev_ego_pose_ptr_ = std::make_unique<geometry_msgs::Pose>(ego_pose);
  prev_replanned_time_ptr_ = std::make_unique<ros::Time>(ros::Time::now());

  DebugData debug_data;
  const auto opt_optimized_points = eb_path_optimizer_ptr_->generateOptimizedTrajectory(
    enable_avoidance_, ego_pose, path, prev_traj_points_ptr_, in_objects_ptr_->objects,
    &debug_data);
  std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
  if (!opt_optimized_points) {
    optimized_points = getOptimizedPointsWhenAborting(path.points);
  } else {
    optimized_points = opt_optimized_points.get();
  }
  prev_optimized_points_ptr_ =
    std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>(optimized_points);

  publishingDebugData(debug_data, path, optimized_points);
  return optimized_points;
}

void ObstacleAvoidancePlanner::initialize()
{
  ROS_WARN("[ObstacleAvoidancePlanner] Resetting");
  eb_path_optimizer_ptr_ = std::make_unique<EBPathOptimizer>(
    is_showing_debug_info_, *qp_param_, *traj_param_, *constrain_param_);
  prev_traj_points_ptr_ = nullptr;
  prev_path_points_ptr_ = nullptr;
}

std::unique_ptr<geometry_msgs::Pose> ObstacleAvoidancePlanner::getCurrentEgoPose()
{
  geometry_msgs::TransformStamped tf_current_pose;

  try {
    tf_current_pose =
      tf_buffer_ptr_->lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("[ObstacleAvoidancePlanner] %s", ex.what());
    return nullptr;
  }

  geometry_msgs::Pose p;
  p.orientation = tf_current_pose.transform.rotation;
  p.position.x = tf_current_pose.transform.translation.x;
  p.position.y = tf_current_pose.transform.translation.y;
  p.position.z = tf_current_pose.transform.translation.z;
  std::unique_ptr<geometry_msgs::Pose> p_ptr = std::make_unique<geometry_msgs::Pose>(p);
  return p_ptr;
}

std::vector<autoware_planning_msgs::TrajectoryPoint>
ObstacleAvoidancePlanner::generatePostProcessedTrajectory(
  const geometry_msgs::Pose & ego_pose,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  std::vector<autoware_planning_msgs::TrajectoryPoint> trajectory_points;
  if (path_points.empty()) {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose = ego_pose;
    tmp_point.twist.linear.x = 0;
    trajectory_points.push_back(tmp_point);
    return trajectory_points;
  }
  if (optimized_points.empty()) {
    trajectory_points = util::convertPathToTrajectory(path_points);
    return trajectory_points;
  }
  trajectory_points = convertPointsToTrajectory(path_points, optimized_points);

  auto t_end = std::chrono::high_resolution_clock::now();
  float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  ROS_INFO_COND(is_showing_debug_info_, "Post processing time: = %f [ms]", elapsed_ms);

  return trajectory_points;
}

bool ObstacleAvoidancePlanner::needReplan(
  const geometry_msgs::Pose & ego_pose, const std::unique_ptr<geometry_msgs::Pose> & prev_ego_pose,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::unique_ptr<ros::Time> & prev_replanned_time,
  const std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>> & prev_path_points,
  std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> & prev_traj_points)
{
  if (!prev_ego_pose || !prev_replanned_time || !prev_path_points || !prev_traj_points) {
    return true;
  }

  if (isPathShapeChanged(ego_pose, path_points, prev_path_points)) {
    prev_traj_points = nullptr;
    return true;
  }

  const int default_idx = -1;
  const int nearest_idx_with_thres = util::getNearestIdx(
    *prev_traj_points, ego_pose, default_idx, traj_param_->delta_yaw_threshold_for_closest_point,
    traj_param_->delta_dist_threshold_for_closest_point);
  if (nearest_idx_with_thres == default_idx) {
    prev_traj_points = nullptr;
    return true;
  }

  const double delta_dist = util::calculate2DDistance(ego_pose.position, prev_ego_pose->position);
  if (delta_dist > min_delta_dist_for_replan_) {
    return true;
  }

  ros::Duration delta_time = ros::Time::now() - *prev_replanned_time;
  const double delta_time_sec = delta_time.toSec();
  if (delta_time_sec > min_delta_time_sec_for_replan_) {
    return true;
  }
  return false;
}

bool ObstacleAvoidancePlanner::isPathShapeChanged(
  const geometry_msgs::Pose & ego_pose,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>> & prev_path_points)
{
  if (!prev_path_points) {
    return true;
  }
  const int default_nearest_prev_path_idx = 0;
  const int nearest_prev_path_idx = util::getNearestIdx(
    *prev_path_points, ego_pose, default_nearest_prev_path_idx,
    traj_param_->delta_yaw_threshold_for_closest_point);
  const int default_nearest_path_idx = 0;
  const int nearest_path_idx = util::getNearestIdx(
    path_points, ego_pose, default_nearest_path_idx,
    traj_param_->delta_yaw_threshold_for_closest_point);

  const auto prev_first = prev_path_points->begin() + nearest_prev_path_idx;
  const auto prev_last = prev_path_points->end();
  std::vector<autoware_planning_msgs::PathPoint> truncated_prev_points(prev_first, prev_last);

  const auto first = path_points.begin() + nearest_path_idx;
  const auto last = path_points.end();
  std::vector<autoware_planning_msgs::PathPoint> truncated_points(first, last);

  for (const auto & prev_point : truncated_prev_points) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto & point : truncated_points) {
      const double dist = util::calculate2DDistance(point.pose.position, prev_point.pose.position);
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
    if (min_dist > distance_for_path_shape_chagne_detection_) {
      return true;
    }
  }
  return false;
}

std::vector<autoware_planning_msgs::TrajectoryPoint>
ObstacleAvoidancePlanner::convertPointsToTrajectory(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points)
{
  std::vector<geometry_msgs::Point> interpolated_points =
    util::getInterpolatedPoints(trajectory_points, traj_param_->delta_arc_length_for_trajectory);
  // add discarded point in the process of interpolation
  interpolated_points.push_back(trajectory_points.back().pose.position);
  if (interpolated_points.size() < min_num_points_for_getting_yaw_) {
    return util::convertPathToTrajectory(path_points);
  }
  std::vector<geometry_msgs::Point> candidate_points = interpolated_points;
  const auto extended_point_opt = getExtendedPoint(path_points, candidate_points);
  if (extended_point_opt) {
    candidate_points.push_back(extended_point_opt.get());
  }
  int zero_velocity_path_points_idx = path_points.size() - 1;
  for (int i = 0; i < path_points.size(); i++) {
    if (path_points[i].twist.linear.x < 1e-6) {
      zero_velocity_path_points_idx = i;
      break;
    }
  }
  geometry_msgs::Pose candidate_back;
  candidate_back.position = candidate_points.back();
  candidate_back.orientation = util::getQuaternionFromPoints(
    candidate_points[candidate_points.size() - 1], candidate_points[candidate_points.size() - 2]);
  const int default_path_idx = path_points.size() - 1;
  const int nearest_path_idx_from_back = util::getNearestIdx(
    path_points, candidate_back, default_path_idx,
    traj_param_->delta_yaw_threshold_for_closest_point);
  const int nearest_path_idx = std::min(nearest_path_idx_from_back, zero_velocity_path_points_idx);
  const int default_idx = candidate_points.size() - 1;
  const int zero_velocity_traj_idx = util::getNearestIdx(
    candidate_points, path_points[nearest_path_idx].pose, default_idx,
    traj_param_->delta_yaw_threshold_for_closest_point);
  const float debug_dist = util::calculate2DDistance(
    path_points[zero_velocity_path_points_idx].pose.position,
    candidate_points[zero_velocity_traj_idx]);
  ROS_INFO_COND(is_showing_debug_info_, "Dist from path 0 velocity point: = %f [m]", debug_dist);

  auto traj_points = util::convertPointsToTrajectoryPoinsWithYaw(candidate_points);
  traj_points = util::fillTrajectoryWithVelocity(traj_points, 1e4);
  if (prev_optimized_points_ptr_) {
    const int max_skip_comparison_velocity_idx_for_optimized_poins =
      calculateNonDecelerationRange(traj_points, *current_ego_pose_ptr_, current_twist_ptr_->twist);
    traj_points = util::alignVelocityWithPoints(
      traj_points, *prev_optimized_points_ptr_, zero_velocity_traj_idx,
      max_skip_comparison_velocity_idx_for_optimized_poins);
  }
  const int max_skip_comparison_idx_for_path_poins = -1;
  traj_points = util::alignVelocityWithPoints(
    traj_points, path_points, zero_velocity_traj_idx, max_skip_comparison_idx_for_path_poins);

  return traj_points;
}

boost::optional<geometry_msgs::Point> ObstacleAvoidancePlanner::getExtendedPoint(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::vector<geometry_msgs::Point> & interpolated_points)
{
  if (interpolated_points.empty() || path_points.empty()) {
    return boost::none;
  }
  const double dx1 = interpolated_points.back().x - path_points.back().pose.position.x;
  const double dy1 = interpolated_points.back().y - path_points.back().pose.position.y;
  const double dist = std::sqrt(dx1 * dx1 + dy1 * dy1);

  const double yaw = tf2::getYaw(path_points.back().pose.orientation);
  const double dx2 = std::cos(yaw);
  const double dy2 = std::sin(yaw);
  const double inner_product = dx1 * dx2 + dy1 * dy2;
  if (dist < max_dist_for_extending_end_point_ && inner_product < 0) {
    return path_points.back().pose.position;
  } else {
    return boost::none;
  }
}

std::vector<autoware_planning_msgs::TrajectoryPoint>
ObstacleAvoidancePlanner::getOptimizedPointsWhenAborting(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points)
{
  if (!prev_optimized_points_ptr_) {
    prev_optimized_points_ptr_ =
      std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
        util::convertPathToTrajectory(path_points));
  }
  return *prev_optimized_points_ptr_;
}

void ObstacleAvoidancePlanner::publishingDebugData(
  const DebugData & debug_data, const autoware_planning_msgs::Path & path,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points)
{
  autoware_planning_msgs::Trajectory traj;
  traj.header = path.header;
  traj.points = debug_data.foa_data.avoiding_traj_points;
  avoiding_traj_pub_.publish(traj);

  std_msgs::Bool is_avoidance_possible;
  is_avoidance_possible.data = debug_data.foa_data.is_avoidance_possible;
  is_avoidance_possible_pub_.publish(is_avoidance_possible);

  debug_markers_pub_.publish(getDebugVisualizationMarker(
    debug_data.interpolated_points, traj_points, debug_data.straight_points,
    debug_data.fixed_points, debug_data.non_fixed_points, debug_data.constrain_rectangles,
    debug_data.avoiding_objects));
  if (is_publishing_area_with_objects_) {
    debug_area_with_objects_pub_.publish(
      getDebugCostmap(debug_data.area_with_objects_map, path.drivable_area));
  }
  if (is_publishing_clearance_map_) {
    debug_clearance_map_pub_.publish(getDebugCostmap(debug_data.clearance_map, path.drivable_area));
    debug_object_clearance_map_pub_.publish(
      getDebugCostmap(debug_data.only_object_clearance_map, path.drivable_area));
  }
}

int ObstacleAvoidancePlanner::calculateNonDecelerationRange(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points,
  const geometry_msgs::Pose & ego_pose, const geometry_msgs::Twist & ego_twist)
{
  const int default_idx = 0;
  const int nearest_idx = util::getNearestIdx(
    traj_points, ego_pose, default_idx, traj_param_->delta_yaw_threshold_for_closest_point);
  const double non_decelerating_arc_length =
    (ego_twist.linear.x - traj_param_->max_avoiding_ego_velocity_ms) /
    traj_param_->acceleration_for_non_deceleration_range;
  if (non_decelerating_arc_length < 0 || traj_points.size() < 2) return 0;

  double accum_arc_length = 0;
  for (int i = nearest_idx + 1; i < traj_points.size(); i++) {
    accum_arc_length +=
      util::calculate2DDistance(traj_points[i].pose.position, traj_points[i - 1].pose.position);
    if (accum_arc_length > non_decelerating_arc_length) {
      return i;
    }
  }
  return 0;
}
