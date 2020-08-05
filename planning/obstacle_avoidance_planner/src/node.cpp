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
#include "obstacle_avoidance_planner/mpt_optimizer.h"
#include "obstacle_avoidance_planner/node.h"
#include "obstacle_avoidance_planner/process_cv.h"
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
  debug_smoothed_points_pub_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/smoothed_poins", 1, true);
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
  pnh_.param<bool>("is_stopping_if_outside_drivable_area", is_stopping_if_outside_drivable_area_, true);
  pnh_.param<bool>("enable_avoidance", enable_avoidance_, true);

  qp_param_ = std::make_unique<QPParam>();
  traj_param_ = std::make_unique<TrajectoryParam>();
  constrain_param_ = std::make_unique<ConstrainParam>();
  vehicle_param_ = std::make_unique<VehicleParam>();
  mpt_param_ = std::make_unique<MPTParam>();
  pnh_.param<int>("qp_max_iteration", qp_param_->max_iteration, 10000);
  pnh_.param<double>("qp_eps_abs", qp_param_->eps_abs, 1.0e-8);
  pnh_.param<double>("qp_eps_rel", qp_param_->eps_rel, 1.0e-11);
  pnh_.param<double>("qp_eps_abs_for_extending", qp_param_->eps_abs_for_extending, 1.0e-6);
  pnh_.param<double>("qp_eps_rel_for_extending", qp_param_->eps_rel_for_extending, 1.0e-8);
  pnh_.param<double>("qp_eps_abs_for_visualizing", qp_param_->eps_abs_for_visualizing, 1.0e-6);
  pnh_.param<double>("qp_eps_rel_for_visualizing", qp_param_->eps_rel_for_visualizing, 1.0e-8);

  pnh_.param<int>("num_sampling_points", traj_param_->num_sampling_points, 100);
  pnh_.param<int>("num_joint_buffer_points", traj_param_->num_joint_buffer_points, 2);
  pnh_.param<int>(
    "num_joint_buffer_points_for_extending", traj_param_->num_joint_buffer_points_for_extending, 4);
  pnh_.param<int>("num_offset_for_begin_idx", traj_param_->num_offset_for_begin_idx, 2);
  pnh_.param<int>("num_fix_points_for_extending", traj_param_->num_fix_points_for_extending, 2);
  pnh_.param<int>("num_fix_points_for_mpt", traj_param_->num_fix_points_for_mpt, 8);
  pnh_.param<double>(
    "delta_arc_length_for_optimization", traj_param_->delta_arc_length_for_optimization, 1.0);
  pnh_.param<double>(
    "delta_arc_length_for_mpt_points", traj_param_->delta_arc_length_for_mpt_points, 1.0);
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
  pnh_.param<double>(
    "max_dist_for_extending_end_point", traj_param_->max_dist_for_extending_end_point, 5.0);

  pnh_.param<bool>(
    "is_getting_constraints_close2path_points",
    constrain_param_->is_getting_constraints_close2path_points, false);
  pnh_.param<double>(
    "clearance_for_straight_line_", constrain_param_->clearance_for_straight_line, 0.05);
  pnh_.param<double>("clearance_for_joint_", constrain_param_->clearance_for_joint, 3.2);
  pnh_.param<double>("range_for_extend_joint", constrain_param_->range_for_extend_joint, 1.6);
  pnh_.param<double>(
    "clearance_for_only_smoothing", constrain_param_->clearance_for_only_smoothing, 0.1);
  pnh_.param<double>(
    "clearance_from_object_for_straight", constrain_param_->clearance_from_object_for_straight,
    10.0);
  pnh_.param<double>("clearance_from_road", constrain_param_->clearance_from_road, 0.1);
  pnh_.param<double>("clearance_from_object", constrain_param_->clearance_from_object, 0.6);
  pnh_.param<double>(
    "extra_desired_clearance_from_road", constrain_param_->extra_desired_clearance_from_road, 0.2);
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
  pnh_.param<double>(
    "distance_for_path_shape_chagne_detection", distance_for_path_shape_chagne_detection_, 2.0);

  if (is_using_vehicle_config_) {
    double vehicle_width = waitForParam<double>(pnh_, "/vehicle_info/vehicle_width");
    traj_param_->center_line_width = vehicle_width;
    constrain_param_->keep_space_shape_y = vehicle_width;
  }
  constrain_param_->min_object_clearance_for_deceleration =
    constrain_param_->clearance_from_object + constrain_param_->keep_space_shape_y * 0.5;

  //vehicle param
  vehicle_param_->width = waitForParam<double>(pnh_, "/vehicle_info/vehicle_width");
  vehicle_param_->length = waitForParam<double>(pnh_, "/vehicle_info/vehicle_length");
  vehicle_param_->wheelbase = waitForParam<double>(pnh_, "/vehicle_info/wheel_base");
  vehicle_param_->rear_overhang = waitForParam<double>(pnh_, "/vehicle_info/rear_overhang");
  vehicle_param_->front_overhang = waitForParam<double>(pnh_, "/vehicle_info/front_overhang");

  double max_steer_deg = 0;
  pnh_.param<double>("max_steer_deg", max_steer_deg, 30.0);
  vehicle_param_->max_steer_rad = max_steer_deg * M_PI / 180.0;
  pnh_.param<double>("steer_tau", vehicle_param_->steer_tau, 0.1);

  // mpt param
  pnh_.param<bool>("is_hard_fix_terminal_point", mpt_param_->is_hard_fix_terminal_point, true);
  pnh_.param<int>("num_curvature_sampling_points", mpt_param_->num_curvature_sampling_points, 5);
  pnh_.param<double>("base_point_weight", mpt_param_->base_point_weight, 2000);
  pnh_.param<double>("top_point_weight", mpt_param_->top_point_weight, 1000);
  pnh_.param<double>("mid_point_weight", mpt_param_->mid_point_weight, 1000);
  pnh_.param<double>("lat_error_weight", mpt_param_->lat_error_weight, 10);
  pnh_.param<double>("yaw_error_weight", mpt_param_->yaw_error_weight, 0);
  pnh_.param<double>("steer_input_weight", mpt_param_->steer_input_weight, 0.1);
  pnh_.param<double>("steer_rate_weight", mpt_param_->steer_rate_weight, 100);
  pnh_.param<double>("steer_acc_weight", mpt_param_->steer_acc_weight, 0.000001);
  pnh_.param<double>("terminal_lat_error_weight", mpt_param_->terminal_lat_error_weight, 0.0);
  pnh_.param<double>("terminal_yaw_error_weight", mpt_param_->terminal_yaw_error_weight, 100.0);
  pnh_.param<double>(
    "terminal_path_lat_error_weight", mpt_param_->terminal_path_lat_error_weight, 1000.0);
  pnh_.param<double>(
    "terminal_path_yaw_error_weight", mpt_param_->terminal_path_yaw_error_weight, 1000.0);
  pnh_.param<double>("zero_ff_steer_angle", mpt_param_->zero_ff_steer_angle, 0.5);

  mpt_param_->clearance_from_road = vehicle_param_->width * 0.5 +
                                    constrain_param_->clearance_from_road +
                                    constrain_param_->extra_desired_clearance_from_road;
  mpt_param_->clearance_from_object =
    vehicle_param_->width * 0.5 + constrain_param_->clearance_from_object;
  mpt_param_->base_point_dist_from_base_link = 0;
  mpt_param_->top_point_dist_from_base_link =
    (vehicle_param_->length - vehicle_param_->rear_overhang);
  mpt_param_->mid_point_dist_from_base_link =
    (mpt_param_->base_point_dist_from_base_link + mpt_param_->top_point_dist_from_base_link) * 0.5;

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
  constrain_param_->clearance_from_road = config.clearance_from_road;
  constrain_param_->clearance_from_object = config.clearance_from_object;
  constrain_param_->min_object_clearance_for_joint = config.min_object_clearance_for_joint;
  constrain_param_->extra_desired_clearance_from_road = config.extra_desired_clearance_from_road;

  // avoiding param
  traj_param_->max_avoiding_ego_velocity_ms = config.max_avoiding_ego_velocity_ms;
  traj_param_->max_avoiding_objects_velocity_ms = config.max_avoiding_objects_velocity_ms;
  traj_param_->center_line_width = config.center_line_width;
  traj_param_->acceleration_for_non_deceleration_range =
    config.acceleration_for_non_deceleration_range;

  // mpt param
  mpt_param_->base_point_weight = config.base_point_weight;
  mpt_param_->top_point_weight = config.top_point_weight;
  mpt_param_->mid_point_weight = config.mid_point_weight;
  mpt_param_->lat_error_weight = config.lat_error_weight;
  mpt_param_->yaw_error_weight = config.yaw_error_weight;
  mpt_param_->steer_input_weight = config.steer_input_weight;
  mpt_param_->steer_rate_weight = config.steer_rate_weight;
  mpt_param_->steer_acc_weight = config.steer_acc_weight;

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
        prev_trajectories_ptr_)) {
    return getPrevTrajectory(path.points);
  }
  prev_ego_pose_ptr_ = std::make_unique<geometry_msgs::Pose>(ego_pose);
  prev_replanned_time_ptr_ = std::make_unique<ros::Time>(ros::Time::now());

  DebugData debug_data;
  const auto optional_trajs = eb_path_optimizer_ptr_->generateOptimizedTrajectory(
    enable_avoidance_, ego_pose, path, prev_trajectories_ptr_, in_objects_ptr_->objects,
    &debug_data);
  if (!optional_trajs) {
    ROS_WARN_THROTTLE(1.0, "[Avoidance] Optimization failed, passing previous trajectory");
    const bool is_prev_traj = true;
    const auto prev_trajs_inside_area = calcTrajectoryInsideArea(
      getPrevTrajs(path.points), path.points, debug_data.clearance_map, path.drivable_area.info,
      &debug_data, is_prev_traj);
    prev_trajectories_ptr_ = std::make_unique<Trajectories>(
      makePrevTrajectories(*current_ego_pose_ptr_, path.points, prev_trajs_inside_area.get()));

    const auto prev_traj = util::concatTraj(prev_trajs_inside_area.get());
    publishingDebugData(debug_data, path, prev_traj);
    return prev_traj;
  }

  const auto trajs_inside_area = getTrajectoryInsideArea(
    optional_trajs.get(), path.points, debug_data.clearance_map, path.drivable_area.info,
    &debug_data);

  prev_trajectories_ptr_ = std::make_unique<Trajectories>(
    makePrevTrajectories(*current_ego_pose_ptr_, path.points, trajs_inside_area));
  const auto optimized_trajectory = util::concatTraj(trajs_inside_area);
  publishingDebugData(debug_data, path, optimized_trajectory);
  return optimized_trajectory;
}

void ObstacleAvoidancePlanner::initialize()
{
  ROS_WARN("[ObstacleAvoidancePlanner] Resetting");
  eb_path_optimizer_ptr_ = std::make_unique<EBPathOptimizer>(
    is_showing_debug_info_, *qp_param_, *traj_param_, *constrain_param_, *vehicle_param_,
    *mpt_param_);
  prev_path_points_ptr_ = nullptr;
  prev_trajectories_ptr_ = nullptr;
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
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points) const
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
  std::unique_ptr<Trajectories> & prev_trajs)
{
  if (!prev_ego_pose || !prev_replanned_time || !prev_path_points || !prev_trajs) {
    return true;
  }

  if (isPathShapeChanged(ego_pose, path_points, prev_path_points)) {
    ROS_INFO("[Avoidance] Path shanpe is changed, reset prev trajs");
    prev_trajs = nullptr;
    return true;
  }

  if (!util::hasValidNearestPointFromEgo(
        *current_ego_pose_ptr_, *prev_trajectories_ptr_, *traj_param_)) {
    ROS_INFO("[Avoidnace] Could not find valid nearest point from ego, reset prev trajs");
    prev_trajs = nullptr;
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
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points) const
{
  std::vector<geometry_msgs::Point> interpolated_points =
    util::getInterpolatedPoints(trajectory_points, traj_param_->delta_arc_length_for_trajectory);
  // add discarded point in the process of interpolation
  interpolated_points.push_back(trajectory_points.back().pose.position);
  if (interpolated_points.size() < min_num_points_for_getting_yaw_) {
    return util::convertPathToTrajectory(path_points);
  }
  std::vector<geometry_msgs::Point> candidate_points = interpolated_points;
  geometry_msgs::Pose last_pose;
  last_pose.position = candidate_points.back();
  last_pose.orientation = util::getQuaternionFromPoints(
    candidate_points.back(), candidate_points[candidate_points.size() - 2]);
  const auto extended_point_opt = util::getLastExtendedPoint(
    path_points.back(), last_pose, traj_param_->delta_yaw_threshold_for_closest_point,
    traj_param_->max_dist_for_extending_end_point);
  if (extended_point_opt) {
    candidate_points.push_back(extended_point_opt.get());
  }

  const int zero_velocity_idx = util::getZeroVelocityIdx(
    is_showing_debug_info_, candidate_points, path_points, prev_trajectories_ptr_, *traj_param_);

  auto traj_points = util::convertPointsToTrajectoryPoinsWithYaw(candidate_points);
  traj_points = util::fillTrajectoryWithVelocity(traj_points, 1e4);
  if (prev_trajectories_ptr_) {
    const int max_skip_comparison_velocity_idx_for_optimized_poins =
      calculateNonDecelerationRange(traj_points, *current_ego_pose_ptr_, current_twist_ptr_->twist);
    const auto optimized_trajectory = util::concatTraj(*prev_trajectories_ptr_);
    traj_points = util::alignVelocityWithPoints(
      traj_points, optimized_trajectory, zero_velocity_idx,
      max_skip_comparison_velocity_idx_for_optimized_poins);
  }
  const int max_skip_comparison_idx_for_path_poins = -1;
  traj_points = util::alignVelocityWithPoints(
    traj_points, path_points, zero_velocity_idx, max_skip_comparison_idx_for_path_poins);

  return traj_points;
}

void ObstacleAvoidancePlanner::publishingDebugData(
  const DebugData & debug_data, const autoware_planning_msgs::Path & path,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points)
{
  autoware_planning_msgs::Trajectory traj;
  traj.header = path.header;
  traj.points = debug_data.foa_data.avoiding_traj_points;
  avoiding_traj_pub_.publish(traj);

  autoware_planning_msgs::Trajectory debug_smoothed_points;
  debug_smoothed_points.header = path.header;
  debug_smoothed_points.points = debug_data.smoothed_points;
  debug_smoothed_points_pub_.publish(debug_smoothed_points);

  std_msgs::Bool is_avoidance_possible;
  is_avoidance_possible.data = debug_data.foa_data.is_avoidance_possible;
  is_avoidance_possible_pub_.publish(is_avoidance_possible);

  debug_markers_pub_.publish(getDebugVisualizationMarker(debug_data, traj_points));
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
  const geometry_msgs::Pose & ego_pose, const geometry_msgs::Twist & ego_twist) const
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

Trajectories ObstacleAvoidancePlanner::getTrajectoryInsideArea(
  const Trajectories & trajs, const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const cv::Mat & road_clearance_map, const nav_msgs::MapMetaData & map_info,
  DebugData * debug_data) const
{
  debug_data->current_vehicle_footprints =
    util::getVehicleFootprints(trajs.model_predictive_trajectory, *vehicle_param_);
  const auto current_trajs_inside_area =
    calcTrajectoryInsideArea(trajs, path_points, road_clearance_map, map_info, debug_data);
  if (!current_trajs_inside_area) {
    ROS_WARN(
      "[Avoidance] Current trajectory is not inside drivable area, passing previous one. "
      "Might stop at the end of drivable trajectory.");
    auto prev_trajs = getPrevTrajs(path_points);
    const bool is_prev_traj = true;
    const auto prev_trajs_inside_area = calcTrajectoryInsideArea(
      prev_trajs, path_points, road_clearance_map, map_info, debug_data, is_prev_traj);
    return prev_trajs_inside_area.get();
  }
  return current_trajs_inside_area.get();
}

boost::optional<Trajectories> ObstacleAvoidancePlanner::calcTrajectoryInsideArea(
  const Trajectories & trajs, const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const cv::Mat & road_clearance_map, const nav_msgs::MapMetaData & map_info,
  DebugData * debug_data, const bool is_prev_traj) const
{
  if (!is_stopping_if_outside_drivable_area_) {
    const auto stop_idx = getStopIdx(path_points, trajs, map_info, road_clearance_map, debug_data);
    if(stop_idx){
      ROS_WARN_THROTTLE(5.0, "[Avoidance] Expecting over drivable area");
    }
    return getBaseTrajectory(path_points, trajs);
  }
  const auto optional_stop_idx =
    getStopIdx(path_points, trajs, map_info, road_clearance_map, debug_data);
  if (!is_prev_traj && optional_stop_idx) {
    return boost::none;
  }

  auto tmp_trajs = getBaseTrajectory(path_points, trajs);
  if (is_prev_traj) {
    tmp_trajs.extended_trajectory = std::vector<autoware_planning_msgs::TrajectoryPoint>{};
    debug_data->is_expected_to_over_drivable_area = true;
  }

  if (optional_stop_idx && !prev_trajectories_ptr_) {
    if (optional_stop_idx.get() < trajs.model_predictive_trajectory.size()) {
      tmp_trajs.model_predictive_trajectory = std::vector<autoware_planning_msgs::TrajectoryPoint>{
        trajs.model_predictive_trajectory.begin(),
        trajs.model_predictive_trajectory.begin() + optional_stop_idx.get()};
      tmp_trajs.extended_trajectory = std::vector<autoware_planning_msgs::TrajectoryPoint>{};
      debug_data->is_expected_to_over_drivable_area = true;
    }
  }
  return tmp_trajs;
}

Trajectories ObstacleAvoidancePlanner::getPrevTrajs(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points) const
{
  if (!prev_trajectories_ptr_) {
    const auto traj = util::convertPathToTrajectory(path_points);
    Trajectories trajs;
    trajs.smoothed_trajectory = traj;
    trajs.model_predictive_trajectory = traj;
    return trajs;
  } else {
    return *prev_trajectories_ptr_;
  }
}

std::vector<autoware_planning_msgs::TrajectoryPoint> ObstacleAvoidancePlanner::getPrevTrajectory(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points) const
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> traj;
  const auto & trajs = getPrevTrajs(path_points);
  traj.insert(
    traj.end(), trajs.model_predictive_trajectory.begin(), trajs.model_predictive_trajectory.end());
  traj.insert(traj.end(), trajs.extended_trajectory.begin(), trajs.extended_trajectory.end());
  return traj;
}

Trajectories ObstacleAvoidancePlanner::makePrevTrajectories(
  const geometry_msgs::Pose & ego_pose,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const Trajectories & trajs) const
{
  const auto post_processed_smoothed_traj =
    generatePostProcessedTrajectory(ego_pose, path_points, trajs.smoothed_trajectory);
  Trajectories trajectories;
  trajectories.smoothed_trajectory = post_processed_smoothed_traj;
  trajectories.model_predictive_trajectory = trajs.model_predictive_trajectory;
  trajectories.extended_trajectory = trajs.extended_trajectory;
  return trajectories;
}

Trajectories ObstacleAvoidancePlanner::getBaseTrajectory(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const Trajectories & trajs) const
{
  auto basee_trajs = trajs;
  if (!trajs.extended_trajectory.empty()) {
    const auto extended_point_opt = util::getLastExtendedTrajPoint(
      path_points.back(), trajs.extended_trajectory.back().pose,
      traj_param_->delta_yaw_threshold_for_closest_point,
      traj_param_->max_dist_for_extending_end_point);
    if (extended_point_opt) {
      basee_trajs.extended_trajectory.push_back(extended_point_opt.get());
    }
  } else if (!trajs.model_predictive_trajectory.empty()) {
    const auto extended_point_opt = util::getLastExtendedTrajPoint(
      path_points.back(), trajs.model_predictive_trajectory.back().pose,
      traj_param_->delta_yaw_threshold_for_closest_point,
      traj_param_->max_dist_for_extending_end_point);
    if (extended_point_opt) {
      basee_trajs.extended_trajectory.push_back(extended_point_opt.get());
    }
  }
  double prev_velocity = 1e4;
  for (auto & p : basee_trajs.model_predictive_trajectory) {
    if (p.twist.linear.x < 1e-6) {
      p.twist.linear.x = prev_velocity;
    } else {
      prev_velocity = p.twist.linear.x;
    }
  }
  for (auto & p : basee_trajs.extended_trajectory) {
    if (p.twist.linear.x < 1e-6) {
      p.twist.linear.x = prev_velocity;
    } else {
      prev_velocity = p.twist.linear.x;
    }
  }
  return basee_trajs;
}

boost::optional<int> ObstacleAvoidancePlanner::getStopIdx(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points, const Trajectories & trajs,
  const nav_msgs::MapMetaData & map_info, const cv::Mat & road_clearance_map,
  DebugData * debug_data) const
{
  const int nearest_idx = util::getNearestIdx(
    path_points, *current_ego_pose_ptr_, 0, traj_param_->delta_yaw_threshold_for_closest_point);
  const double accum_ds = util::getArcLength(path_points, nearest_idx);

  auto target_points = trajs.model_predictive_trajectory;
  if (accum_ds < traj_param_->num_sampling_points * traj_param_->delta_arc_length_for_mpt_points) {
    target_points.insert(
      target_points.end(), trajs.extended_trajectory.begin(), trajs.extended_trajectory.end());
    const auto extended_mpt_point_opt = util::getLastExtendedTrajPoint(
      path_points.back(), trajs.model_predictive_trajectory.back().pose,
      traj_param_->delta_yaw_threshold_for_closest_point,
      traj_param_->max_dist_for_extending_end_point);
    if (extended_mpt_point_opt) {
      target_points.push_back(extended_mpt_point_opt.get());
    }
  }

  const auto footprints = util::getVehicleFootprints(target_points, *vehicle_param_);
  const int debug_nearest_footprint_idx =
    util::getNearestIdx(target_points, current_ego_pose_ptr_->position);
  std::vector<util::Footprint> debug_truncated_footprints(
    footprints.begin() + debug_nearest_footprint_idx, footprints.end());
  debug_data->vehicle_footprints = debug_truncated_footprints;

  const auto optional_idx =
    process_cv::getStopIdx(footprints, *current_ego_pose_ptr_, road_clearance_map, map_info);
  return optional_idx;
}
