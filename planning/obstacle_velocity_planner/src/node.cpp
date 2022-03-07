// Copyright 2020 Tier IV, Inc.
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

#include "obstacle_velocity_planner/node.hpp"

#include "obstacle_velocity_planner/resample.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <tier4_autoware_utils/ros/update_param.hpp>
#include <interpolation/linear_interpolation.hpp>
#include <interpolation/spline_interpolation.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <chrono>

constexpr double ZERO_VEL_THRESHOLD = 0.01;
constexpr double CLOSE_S_DIST_THRESHOLD = 1e-3;

std::string toHexString(const unique_identifier_msgs::msg::UUID & id)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
  }
  return ss.str();
}

inline void convertEulerAngleToMonotonic(std::vector<double> & a)
{
  for (unsigned int i = 1; i < a.size(); ++i) {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + tier4_autoware_utils::normalizeRadian(da);
  }
}

inline tf2::Vector3 getTransVector3(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
{
  double dx = to.position.x - from.position.x;
  double dy = to.position.y - from.position.y;
  double dz = to.position.z - from.position.z;
  return tf2::Vector3(dx, dy, dz);
}

ObstacleVelocityPlanner::ObstacleVelocityPlanner(const rclcpp::NodeOptions & node_options)
: Node("obstacle_velocity_planner", node_options), self_pose_listener_(this), previous_acc_(0.0)
{
  using std::placeholders::_1;

  // Subscriber
  trajectory_sub_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlanner::trajectoryCallback, this, _1));
  smoothed_trajectory_sub_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlanner::smoothedTrajectoryCallback, this, _1));
  objects_sub_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "/perception/object_recognition/objects", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlanner::objectsCallback, this, _1));
  twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/localization/twist", rclcpp::QoS{1},
    std::bind(&ObstacleVelocityPlanner::twistCallback, this, _1));
  sub_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ObstacleVelocityPlanner::mapCallback, this, std::placeholders::_1));
  sub_external_velocity_limit_ = create_subscription<tier4_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/max_velocity", 1,
    std::bind(&ObstacleVelocityPlanner::onExternalVelocityLimit, this, _1));

  // Publisher
  trajectory_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);
  optimized_sv_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/optimized_sv_trajectory", 1);
  optimized_st_graph_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/optimized_st_graph", 1);
  boundary_pub_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/boundary", 1);
  distance_to_closest_obj_pub_ =
    create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/distance_to_closest_obj", 1);
  debug_calculation_time_ =
    create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/calculation_time", 1);

  // Obstacle
  in_objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>();

  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  wheel_base_ = vehicle_info.wheel_base_m;
  front_overhang_ = vehicle_info.front_overhang_m;
  rear_overhang_ = vehicle_info.rear_overhang_m;
  left_overhang_ = vehicle_info.left_overhang_m;
  right_overhang_ = vehicle_info.right_overhang_m;
  vehicle_width_ = vehicle_info.vehicle_width_m;
  vehicle_length_ = vehicle_info.vehicle_length_m;

  // Parameters
  max_accel_ = declare_parameter("max_accel", 1.0);
  min_accel_ = declare_parameter("min_accel", -1.0);
  max_jerk_ = declare_parameter("max_jerk", 1.0);
  min_jerk_ = declare_parameter("min_jerk", -1.0);
  min_object_accel_ = declare_parameter("min_object_accel", -3.0);
  resampling_s_interval_ = declare_parameter("resampling_s_interval", 1.0);
  max_trajectory_length_ = declare_parameter("max_trajectory_length", 200.0);
  dense_resampling_time_interval_ = declare_parameter("dense_resampling_time_interval", 0.1);
  sparse_resampling_time_interval_ = declare_parameter("sparse_resampling_time_interval", 0.5);
  dense_time_horizon_ = declare_parameter("dense_time_horizon", 5.0);
  max_time_horizon_ = declare_parameter("max_time_horizon", 15.0);

  delta_yaw_threshold_of_nearest_index_ =
    tier4_autoware_utils::deg2rad(declare_parameter("delta_yaw_threshold_of_nearest_index", 60.0));
  delta_yaw_threshold_of_object_and_ego_ =
    tier4_autoware_utils::deg2rad(declare_parameter("delta_yaw_threshold_of_object_and_ego", 180.0));
  object_zero_velocity_threshold_ = declare_parameter("object_zero_velocity_threshold", 1.5);
  object_low_velocity_threshold_ = declare_parameter("object_low_velocity_threshold", 3.0);
  external_velocity_limit_ = declare_parameter("external_velocity_limit", 20.0);
  collision_time_threshold_ = declare_parameter("collision_time_threshold", 10.0);
  safe_distance_margin_ = declare_parameter("safe_distance_margin", 2.0);
  t_dangerous_ = declare_parameter("t_dangerous", 0.5);
  t_idling_ = declare_parameter("t_idling", 2.0);
  initial_velocity_margin_ = declare_parameter("initial_velocity_margin", 0.2);
  enable_adaptive_cruise_ = declare_parameter("enable_adaptive_cruise", true);
  use_object_acceleration_ = declare_parameter("use_object_acceleration", true);
  use_hd_map_ = declare_parameter("use_hd_map", true);

  replan_vel_deviation_ = declare_parameter("replan_vel_deviation", 5.53);
  engage_velocity_ = declare_parameter("engage_velocity", 0.25);
  engage_acceleration_ = declare_parameter("engage_acceleration", 0.1);
  engage_exit_ratio_ = declare_parameter("engage_exit_ratio", 0.5);
  stop_dist_to_prohibit_engage_ = declare_parameter("stop_dist_to_prohibit_engage", 0.5);

  // Velocity Optimizer
  const double max_s_weight = declare_parameter("max_s_weight", 10.0);
  const double max_v_weight = declare_parameter("max_v_weight", 100.0);
  const double over_s_safety_weight = declare_parameter("over_s_safety_weight", 1000000.0);
  const double over_s_ideal_weight = declare_parameter("over_s_ideal_weight", 800.0);
  const double over_v_weight = declare_parameter("over_v_weight", 500000.0);
  const double over_a_weight = declare_parameter("over_a_weight", 1000.0);
  const double over_j_weight = declare_parameter("over_j_weight", 50000.0);
  velocity_optimizer_ptr_ = std::make_shared<VelocityOptimizer>(
    max_s_weight, max_v_weight, over_s_safety_weight, over_s_ideal_weight, over_v_weight,
    over_a_weight, over_j_weight);

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();
}

void ObstacleVelocityPlanner::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[Obstacle Velocity Planner]: Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "[Obstacle Velocity Planner]: Map is loaded");
}

void ObstacleVelocityPlanner::objectsCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  in_objects_ptr_ = msg;
}

void ObstacleVelocityPlanner::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  current_twist_ptr_ = msg;
}

void ObstacleVelocityPlanner::smoothedTrajectoryCallback(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  smoothed_trajectory_ptr_ = msg;
}

void ObstacleVelocityPlanner::onExternalVelocityLimit(
  const tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg)
{
  external_velocity_limit_ = msg->max_velocity;
}

void ObstacleVelocityPlanner::trajectoryCallback(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->points.empty() || !current_twist_ptr_) {
    return;
  }

  // Get Current Pose
  current_pose_ptr_ = self_pose_listener_.getCurrentPose();

  // Create Time Vector defined by resampling time interval
  std::vector<double> time_vec;
  for (double t = 0.0; t < dense_time_horizon_; t += dense_resampling_time_interval_) {
    time_vec.push_back(t);
  }
  if (dense_time_horizon_ - time_vec.back() < 1e-3) {
    time_vec.back() = dense_time_horizon_;
  } else {
    time_vec.push_back(dense_time_horizon_);
  }

  for (double t = dense_time_horizon_ + sparse_resampling_time_interval_; t < max_time_horizon_; t += sparse_resampling_time_interval_) {
    time_vec.push_back(t);
  }
  if (max_time_horizon_ - time_vec.back() < 1e-3) {
    time_vec.back() = max_time_horizon_;
  } else {
    time_vec.push_back(max_time_horizon_);
  }

  if (time_vec.size() < 2) {
    RCLCPP_ERROR(get_logger(), "Resolution size is not enough");
    trajectory_pub_->publish(*msg);
    prev_output_ = *msg;
    return;
  }

  // Get the nearest point on the trajectory
  const auto closest_idx = tier4_autoware_utils::findNearestIndex(
    msg->points, current_pose_ptr_->pose, delta_yaw_threshold_of_nearest_index_);

  // Check validity of the closest index
  if (!closest_idx) {
    RCLCPP_ERROR(get_logger(), "Closest Index is Invalid");
    trajectory_pub_->publish(*msg);
    prev_output_ = *msg;
    return;
  }

  // Transfrom original trajectory to TrajectoryData
  const auto base_traj_data = getTrajectoryData(*msg, *closest_idx);
  if (base_traj_data.traj.points.size() < 2) {
    RCLCPP_DEBUG(get_logger(), "The number of points on the trajectory data is too small");
    trajectory_pub_->publish(*msg);
    prev_output_ = *msg;
    return;
  }

  // Get Closest Stop Point for static obstacles
  double closest_stop_dist = getClosestStopDistance(base_traj_data, time_vec);

  // Compute maximum velocity
  double v_max = 0.0;
  for (const auto & point : msg->points) {
    v_max = std::max(v_max, static_cast<double>(point.longitudinal_velocity_mps));
  }
  v_max = std::min(v_max, external_velocity_limit_);

  // Get Current Velocity
  double v0;
  double a0;
  std::tie(v0, a0) = calcInitialMotion(*msg, *closest_idx, prev_output_, closest_stop_dist);
  v0 = std::min(v0 + initial_velocity_margin_, v_max);
  a0 = std::min(max_accel_, std::max(a0, min_accel_));

  // If closest distance is too close, return zero velocity
  if (closest_stop_dist < 0.01) {
    RCLCPP_DEBUG(get_logger(), "Closest Stop Point is too close");

    auto output_traj = *msg;
    output_traj.points.at(*closest_idx).longitudinal_velocity_mps = v0;
    for (size_t i = *closest_idx + 1; i < output_traj.points.size(); ++i) {
      output_traj.points.at(i).longitudinal_velocity_mps = 0.0;
    }
    trajectory_pub_->publish(output_traj);
    prev_output_ = output_traj;
    return;
  }

  // Check trajectory size
  if (msg->points.size() - *closest_idx <= 2) {
    RCLCPP_DEBUG(get_logger(), "The number of points on the trajectory is too small");
    trajectory_pub_->publish(*msg);
    prev_output_ = *msg;
    return;
  }

  // Check if reached goal
  if (checkHasReachedGoal(*msg, *closest_idx, v0) || !enable_adaptive_cruise_) {
    trajectory_pub_->publish(*msg);
    prev_output_ = *msg;
    return;
  }

  // Resample base trajectory data by time
  const auto resampled_traj_data = resampleTrajectoryData(
    base_traj_data, resampling_s_interval_, max_trajectory_length_, closest_stop_dist);
  if (resampled_traj_data.traj.points.size() < 2) {
    RCLCPP_DEBUG(
      get_logger(), "The number of points on the resampled trajectory data is too small");
    trajectory_pub_->publish(*msg);
    prev_output_ = *msg;
    return;
  }

  // Get S Boundaries from the obstacle
  const auto s_boundaries = getSBoundaries(current_pose_ptr_->pose, resampled_traj_data, time_vec);
  if (!s_boundaries) {
    RCLCPP_DEBUG(get_logger(), "No Dangerous Objects around the ego vehicle");
    trajectory_pub_->publish(*msg);
    prev_output_ = *msg;
    return;
  }

  // Optimization
  VelocityOptimizer::OptimizationData data;
  data.time_vec = time_vec;
  data.s0 = resampled_traj_data.s.front();
  data.a0 = a0;
  data.v_max = v_max;
  data.a_max = max_accel_;
  data.a_min = min_accel_;
  data.j_max = max_jerk_;
  data.j_min = min_jerk_;
  data.t_dangerous = t_dangerous_;
  data.t_idling = t_idling_;
  data.s_boundary = *s_boundaries;
  data.v0 = v0;
  RCLCPP_DEBUG(get_logger(), "v0 %f", v0);

  stop_watch_.tic();

  const auto optimized_result = velocity_optimizer_ptr_->optimize(data);

  tier4_debug_msgs::msg::Float32Stamped calculation_time_data{};
  calculation_time_data.stamp = this->now();
  calculation_time_data.data = stop_watch_.toc();
  debug_calculation_time_->publish(calculation_time_data);
  RCLCPP_DEBUG(get_logger(), "Optimization Time; %f[ms]", calculation_time_data.data);

  // Publish Debug trajectories
  publishDebugTrajectory(*msg, *closest_idx, time_vec, *s_boundaries, optimized_result);

  // Transformation from t to s
  const auto processed_result = processOptimizedResult(data.v0, optimized_result);
  if (!processed_result) {
    RCLCPP_DEBUG(get_logger(), "Processed Result is empty");
    trajectory_pub_->publish(*msg);
    prev_output_ = *msg;
    return;
  }
  const auto & opt_position = processed_result->s;
  const auto & opt_velocity = processed_result->v;

  // Check Size
  if (opt_position.size() == 1 && opt_velocity.front() < ZERO_VEL_THRESHOLD) {
    auto output = *msg;
    output.points.at(*closest_idx).longitudinal_velocity_mps = data.v0;
    for (size_t i = *closest_idx + 1; i < output.points.size(); ++i) {
      output.points.at(i).longitudinal_velocity_mps = 0.0;
    }
    trajectory_pub_->publish(output);
    prev_output_ = output;
    return;
  } else if (opt_position.size() == 1) {
    RCLCPP_DEBUG(get_logger(), "Optimized Trajectory is too small");
    trajectory_pub_->publish(*msg);
    prev_output_ = *msg;
    return;
  }

  // Get Zero Velocity Position
  for (size_t i = 0; i < opt_velocity.size(); ++i) {
    if (opt_velocity.at(i) < ZERO_VEL_THRESHOLD) {
      const double zero_vel_s = opt_position.at(i);
      closest_stop_dist = std::min(closest_stop_dist, zero_vel_s);
      break;
    }
  }

  size_t break_id = base_traj_data.s.size();
  bool has_insert_stop_point = false;
  std::vector<double> resampled_opt_position = {base_traj_data.s.front()};
  for (size_t i = 1; i < base_traj_data.s.size(); ++i) {
    const double query_s = base_traj_data.s.at(i);
    if (
      !has_insert_stop_point && query_s > closest_stop_dist &&
      closest_stop_dist < opt_position.back()) {
      const double prev_query_s = resampled_opt_position.back();
      if (closest_stop_dist - prev_query_s > 1e-3) {
        resampled_opt_position.push_back(closest_stop_dist);
      } else {
        resampled_opt_position.back() = closest_stop_dist;
      }
      has_insert_stop_point = true;
    }

    if (query_s > opt_position.back()) {
      break_id = i;
      break;
    }

    const double prev_query_s = resampled_opt_position.back();
    if (query_s - prev_query_s > 1e-3) {
      resampled_opt_position.push_back(query_s);
    }
  }
  const auto resampled_opt_velocity =
    interpolation::lerp(opt_position, opt_velocity, resampled_opt_position);

  // Push positions after the last position of the opt position
  for (size_t i = break_id; i < base_traj_data.s.size(); ++i) {
    const double query_s = base_traj_data.s.at(i);
    const double prev_query_s = resampled_opt_position.back();
    if (query_s - prev_query_s > 1e-3) {
      resampled_opt_position.push_back(query_s);
    }
  }

  std::vector<double> base_s(base_traj_data.s.size());
  for (size_t i = 0; i < base_traj_data.s.size(); ++i) {
    base_s.at(i) = base_traj_data.s.at(i);
  }

  auto resampled_traj =
    resampling::applyLinearInterpolation(base_s, base_traj_data.traj, resampled_opt_position);
  for (size_t i = 0; i < resampled_opt_velocity.size(); ++i) {
    resampled_traj.points.at(i).longitudinal_velocity_mps =
      std::min(resampled_opt_velocity.at(i), static_cast<double>(resampled_traj.points.at(i).longitudinal_velocity_mps));
  }
  for (size_t i = 0; i < resampled_opt_position.size(); ++i) {
    if (resampled_opt_position.at(i) >= closest_stop_dist) {
      resampled_traj.points.at(i).longitudinal_velocity_mps = 0.0;
    }
  }

  autoware_auto_planning_msgs::msg::Trajectory output;
  output.header = msg->header;
  for (size_t i = 0; i < *closest_idx; ++i) {
    auto point = msg->points.at(i);
    point.longitudinal_velocity_mps = data.v0;
    output.points.push_back(point);
  }
  output.points.insert(
    output.points.end(), resampled_traj.points.begin(), resampled_traj.points.end());
  output.points.back().longitudinal_velocity_mps = 0.0;  // terminal velocity is zero

  // Zero Velocity Padding
  const auto zero_vel_idx =
    tier4_autoware_utils::searchZeroVelocityIndex(output.points, *closest_idx, output.points.size());
  if (zero_vel_idx) {
    for (size_t i = *zero_vel_idx; i < output.points.size(); ++i) {
      output.points.at(i).longitudinal_velocity_mps = 0.0;
    }
  }

  prev_output_ = output;
  trajectory_pub_->publish(output);
}

double ObstacleVelocityPlanner::getClosestStopDistance(const TrajectoryData & ego_traj_data, const std::vector<double> & resolutions)
{
  const auto current_time = get_clock()->now();
  double closest_stop_dist = ego_traj_data.s.back();
  const auto closest_stop_id = tier4_autoware_utils::searchZeroVelocityIndex(ego_traj_data.traj.points);
  closest_stop_dist =
    closest_stop_id ? std::min(closest_stop_dist, ego_traj_data.s.at(*closest_stop_id)) : closest_stop_dist;
  if (!in_objects_ptr_) {
    return closest_stop_dist;
  }

  double closest_obj_distance = ego_traj_data.s.back();
  autoware_auto_perception_msgs::msg::PredictedObject closest_obj;
  const auto obj_base_time = rclcpp::Time(in_objects_ptr_->header.stamp);
  for (const auto & obj : in_objects_ptr_->objects) {
    // Step1. Ignore obstacles that are not vehicles
    if (
      obj.classification.at(0).label != autoware_auto_perception_msgs::msg::ObjectClassification::CAR &&
      obj.classification.at(0).label != autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK &&
      obj.classification.at(0).label != autoware_auto_perception_msgs::msg::ObjectClassification::BUS &&
      obj.classification.at(0).label != autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE) {
      continue;
    }

    // Step2 Check object position
    if (!checkIsFrontObject(obj, ego_traj_data.traj)) {
      continue;
    }

    // Get the predicted path, which has the most high confidence
    const auto predicted_path =
      resampledPredictedPath(obj, obj_base_time, current_time, resolutions, max_time_horizon_);
    if (!predicted_path) {
      continue;
    }

    // Get current pose from object's predicted path
    const auto current_object_pose =
      getCurrentObjectPoseFromPredictedPath(*predicted_path, obj_base_time, current_time);
    if (!current_object_pose) {
      continue;
    }

    // Get current object.kinematics
    ObjectData obj_data;
    obj_data.pose = current_object_pose.get();
    obj_data.length = obj.shape.dimensions.x;
    obj_data.width = obj.shape.dimensions.y;
    obj_data.time =
      std::max((obj_base_time - current_time).seconds(), 0.0);
    const double current_delta_yaw_threshold = 3.14;
    const auto dist_to_collision_point =
      getDistanceToCollisionPoint(ego_traj_data, obj_data, current_delta_yaw_threshold);

    // Calculate Safety Distance
    const double ego_vehicle_offset = wheel_base_ + front_overhang_;
    const double object_offset = obj_data.length / 2.0;
    const double safety_distance = ego_vehicle_offset + object_offset + safe_distance_margin_;

    // If the object is on the current ego trajectory,
    // we assume the object travels along ego trajectory
    const double obj_vel = std::fabs(obj.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (
      dist_to_collision_point &&
      obj_vel < object_zero_velocity_threshold_) {
      const double current_stop_point = std::max(*dist_to_collision_point - safety_distance, 0.0);
      closest_stop_dist = std::min(current_stop_point, closest_stop_dist);
    }

    // Update Distance to the closest object on the ego trajectory
    if (dist_to_collision_point) {
      const double current_obj_distance = std::max(
        *dist_to_collision_point - safety_distance + safe_distance_margin_, -safety_distance);
      closest_obj_distance = std::min(closest_obj_distance, current_obj_distance);
      closest_obj = obj;
    }
  }

  // Publish distance from the ego vehicle to the object which is on the trajectory
  if (closest_obj_distance < ego_traj_data.s.back()) {
    tier4_debug_msgs::msg::Float32Stamped dist_to_obj;
    dist_to_obj.stamp = this->now();
    dist_to_obj.data = closest_obj_distance;
    distance_to_closest_obj_pub_->publish(dist_to_obj);

    const auto predicted_path =
      resampledPredictedPath(closest_obj, obj_base_time, current_time, resolutions, max_time_horizon_);

    // Get current pose from object's predicted path
    const auto current_object_pose =
      getCurrentObjectPoseFromPredictedPath(*predicted_path, obj_base_time, current_time);
    const size_t nearest_idx =
    tier4_autoware_utils::findNearestIndex(ego_traj_data.traj.points, current_object_pose.get().position);
    const double ego_yaw = tf2::getYaw(ego_traj_data.traj.points.at(nearest_idx).pose.orientation);
    const double obj_yaw = tf2::getYaw(current_object_pose.get().orientation);
    const double diff_yaw = tier4_autoware_utils::normalizeRadian(obj_yaw - ego_yaw);
    RCLCPP_DEBUG(get_logger(), "Closest Object Distance %f", closest_obj_distance);
    RCLCPP_DEBUG(get_logger(), "Closest Object Velocity %f", closest_obj.kinematics.initial_twist_with_covariance.twist.linear.x);
  }

  return closest_stop_dist;
}

// v0, a0
std::tuple<double, double> ObstacleVelocityPlanner::calcInitialMotion(
  const autoware_auto_planning_msgs::msg::Trajectory & input_traj, const size_t input_closest,
  const autoware_auto_planning_msgs::msg::Trajectory & prev_traj, const double closest_stop_dist)
{
  const double vehicle_speed{std::fabs(current_twist_ptr_->twist.linear.x)};
  const double target_vel{std::fabs(input_traj.points.at(input_closest).longitudinal_velocity_mps)};

  double initial_vel{};
  double initial_acc{};

  // first time
  if (prev_traj.points.empty()) {
    initial_vel = vehicle_speed;
    initial_acc = 0.0;
    return std::make_tuple(initial_vel, initial_acc);
  }

  autoware_auto_planning_msgs::msg::TrajectoryPoint prev_output_closest_point;
  if (smoothed_trajectory_ptr_) {
    prev_output_closest_point = calcInterpolatedTrajectoryPoint(
      *smoothed_trajectory_ptr_, input_traj.points.at(input_closest).pose);
  } else {
    prev_output_closest_point =
      calcInterpolatedTrajectoryPoint(prev_traj, input_traj.points.at(input_closest).pose);
  }

  // when velocity tracking deviation is large
  const double desired_vel{prev_output_closest_point.longitudinal_velocity_mps};
  const double vel_error{vehicle_speed - std::fabs(desired_vel)};
  if (std::fabs(vel_error) > replan_vel_deviation_) {
    initial_vel = vehicle_speed;  // use current vehicle speed
    initial_acc = prev_output_closest_point.acceleration_mps2;
    RCLCPP_DEBUG(
      get_logger(),
      "calcInitialMotion : Large deviation error for speed control. Use current speed for "
      "initial value, desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
      desired_vel, vehicle_speed, vel_error, replan_vel_deviation_);
    return std::make_tuple(initial_vel, initial_acc);
  }

  // if current vehicle velocity is low && base_desired speed is high,
  // use engage_velocity for engage vehicle
  const double engage_vel_thr = engage_velocity_ * engage_exit_ratio_;
  if (vehicle_speed < engage_vel_thr) {
    if (target_vel >= engage_velocity_) {
      const auto idx = tier4_autoware_utils::searchZeroVelocityIndex(input_traj.points);
      const double zero_vel_dist =
        idx ? tier4_autoware_utils::calcDistance2d(
                input_traj.points.at(*idx), input_traj.points.at(input_closest))
            : 0.0;
      const double stop_dist = std::min(zero_vel_dist, closest_stop_dist);
      if (!idx || stop_dist > stop_dist_to_prohibit_engage_) {
        initial_vel = engage_velocity_;
        initial_acc = engage_acceleration_;
        RCLCPP_DEBUG(
          get_logger(),
          "calcInitialMotion : vehicle speed is low (%.3f), and desired speed is high (%.3f). Use "
          "engage speed (%.3f) until vehicle speed reaches engage_vel_thr (%.3f). stop_dist = %.3f",
          vehicle_speed, target_vel, engage_velocity_, engage_vel_thr, stop_dist);
        return std::make_tuple(initial_vel, initial_acc);
      } else {
        RCLCPP_DEBUG(
          get_logger(), "calcInitialMotion : stop point is close (%.3f[m]). no engage.", stop_dist);
      }
    } else if (target_vel > 0.0) {
      auto clock{rclcpp::Clock{RCL_ROS_TIME}};
      RCLCPP_WARN_THROTTLE(
        get_logger(), clock, 3000,
        "calcInitialMotion : target velocity(%.3f[m/s]) is lower than engage velocity(%.3f[m/s]). ",
        target_vel, engage_velocity_);
    }
  }

  // normal update: use closest in prev_output
  initial_vel = prev_output_closest_point.longitudinal_velocity_mps;
  initial_acc = prev_output_closest_point.acceleration_mps2;
  /*
  RCLCPP_DEBUG(
    get_logger(),
    "calcInitialMotion : normal update. v0 = %f, a0 = %f, vehicle_speed = %f, target_vel = %f",
    initial_vel, initial_acc, vehicle_speed, target_vel);
  */
  return std::make_tuple(initial_vel, initial_acc);
}

autoware_auto_planning_msgs::msg::TrajectoryPoint
ObstacleVelocityPlanner::calcInterpolatedTrajectoryPoint(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & target_pose)
{
  autoware_auto_planning_msgs::msg::TrajectoryPoint traj_p{};
  traj_p.pose = target_pose;

  if (trajectory.points.empty()) {
    traj_p.longitudinal_velocity_mps = 0.0;
    traj_p.acceleration_mps2 = 0.0;
    return traj_p;
  }

  if (trajectory.points.size() == 1) {
    traj_p.longitudinal_velocity_mps = trajectory.points.at(0).longitudinal_velocity_mps;
    traj_p.acceleration_mps2 = trajectory.points.at(0).acceleration_mps2;
    return traj_p;
  }

  const size_t segment_idx =
    tier4_autoware_utils::findNearestSegmentIndex(trajectory.points, target_pose.position);

  auto v1 = getTransVector3(
    trajectory.points.at(segment_idx).pose, trajectory.points.at(segment_idx + 1).pose);
  auto v2 = getTransVector3(trajectory.points.at(segment_idx).pose, target_pose);
  // calc internal proportion
  const double prop{std::max(0.0, std::min(1.0, v1.dot(v2) / v1.length2()))};

  auto interpolate = [&prop](double x1, double x2) { return prop * x1 + (1.0 - prop) * x2; };

  {
    const auto & seg_pt = trajectory.points.at(segment_idx);
    const auto & next_pt = trajectory.points.at(segment_idx + 1);
    traj_p.longitudinal_velocity_mps = interpolate(next_pt.longitudinal_velocity_mps, seg_pt.longitudinal_velocity_mps);
    traj_p.acceleration_mps2 = interpolate(next_pt.acceleration_mps2, seg_pt.acceleration_mps2);
    traj_p.pose.position.x = interpolate(next_pt.pose.position.x, seg_pt.pose.position.x);
    traj_p.pose.position.y = interpolate(next_pt.pose.position.y, seg_pt.pose.position.y);
    traj_p.pose.position.z = interpolate(next_pt.pose.position.z, seg_pt.pose.position.z);
  }

  return traj_p;
}

bool ObstacleVelocityPlanner::checkHasReachedGoal(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const size_t closest_idx, const double v0)
{
  // If goal is close and current velocity is low, we don't optimize trajectory
  const auto closest_stop_id = tier4_autoware_utils::searchZeroVelocityIndex(traj.points);
  if (closest_stop_id) {
    const double closest_stop_dist =
      tier4_autoware_utils::calcSignedArcLength(traj.points, closest_idx, *closest_stop_id);
    if (closest_stop_dist < 0.5 && v0 < 0.6) {
      RCLCPP_DEBUG(get_logger(), "Goal Reached");
      return true;
    }
  }

  return false;
}

TrajectoryData ObstacleVelocityPlanner::getTrajectoryData(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const size_t closest_idx)
{
  TrajectoryData base_traj;
  base_traj.traj.points.push_back(traj.points.at(closest_idx));
  base_traj.s.push_back(0.0);

  for (size_t id = closest_idx + 1; id < traj.points.size(); ++id) {
    const double ds = tier4_autoware_utils::calcSignedArcLength(traj.points, id - 1, id);
    const double current_s = base_traj.s.back() + ds;

    base_traj.traj.points.push_back(traj.points.at(id));
    base_traj.s.push_back(current_s);
  }

  return base_traj;
}

TrajectoryData ObstacleVelocityPlanner::resampleTrajectoryData(
  const TrajectoryData & base_traj_data, const double resampling_s_interval,
  const double max_traj_length, const double stop_dist)
{
  // Create Base Keys
  std::vector<double> base_s(base_traj_data.s.size());
  for (size_t i = 0; i < base_s.size(); ++i) {
    base_s.at(i) = base_traj_data.s.at(i);
  }

  // Obtain trajectory length until the velocity is zero or stop dist
  const auto closest_stop_id = tier4_autoware_utils::searchZeroVelocityIndex(base_traj_data.traj.points);
  const double closest_stop_dist =
    closest_stop_id ? std::min(stop_dist, base_traj_data.s.at(*closest_stop_id)) : stop_dist;
  const double traj_length = std::min(closest_stop_dist, std::min(base_s.back(), max_traj_length));

  // Create Query Keys
  std::vector<double> resampled_s;
  for (double s = 0.0; s < traj_length; s += resampling_s_interval) {
    resampled_s.push_back(s);
  }

  if(resampled_s.empty()) {
    return TrajectoryData{};
  }

  if (traj_length - resampled_s.back() < 1e-3) {
    resampled_s.back() = traj_length;
  } else {
    resampled_s.push_back(traj_length);
  }

  // Resmaple trajectory
  const auto resampled_traj = resampleTrajectory(base_s, base_traj_data.traj, resampled_s);

  // Store Data
  TrajectoryData resampled_traj_data;
  resampled_traj_data.traj = resampled_traj;
  resampled_traj_data.s = resampled_s;

  return resampled_traj_data;
}

autoware_auto_planning_msgs::msg::Trajectory ObstacleVelocityPlanner::resampleTrajectory(
  const std::vector<double> & base_index,
  const autoware_auto_planning_msgs::msg::Trajectory & base_trajectory,
  const std::vector<double> & query_index, const bool use_spline_for_pose)
{
  std::vector<double> px, py, pz, pyaw, tlx, taz, alx, aaz;
  for (const auto & p : base_trajectory.points) {
    px.push_back(p.pose.position.x);
    py.push_back(p.pose.position.y);
    pz.push_back(p.pose.position.z);
    pyaw.push_back(tf2::getYaw(p.pose.orientation));
    tlx.push_back(p.longitudinal_velocity_mps);
    taz.push_back(p.heading_rate_rps);
    alx.push_back(p.acceleration_mps2);
  }

  convertEulerAngleToMonotonic(pyaw);

  std::vector<double> px_p, py_p, pz_p, pyaw_p;
  if (use_spline_for_pose) {
    px_p = interpolation::slerp(base_index, px, query_index);
    py_p = interpolation::slerp(base_index, py, query_index);
    pz_p = interpolation::slerp(base_index, pz, query_index);
    pyaw_p = interpolation::slerp(base_index, pyaw, query_index);
  } else {
    px_p = interpolation::lerp(base_index, px, query_index);
    py_p = interpolation::lerp(base_index, py, query_index);
    pz_p = interpolation::lerp(base_index, pz, query_index);
    pyaw_p = interpolation::lerp(base_index, pyaw, query_index);
  }
  const auto tlx_p = interpolation::lerp(base_index, tlx, query_index);
  const auto taz_p = interpolation::lerp(base_index, taz, query_index);
  const auto alx_p = interpolation::lerp(base_index, alx, query_index);
  const auto aaz_p = interpolation::lerp(base_index, aaz, query_index);

  autoware_auto_planning_msgs::msg::Trajectory resampled_trajectory;
  resampled_trajectory.header = base_trajectory.header;
  resampled_trajectory.points.resize(query_index.size());

  for (size_t i = 0; i < query_index.size(); ++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = px_p.at(i);
    point.pose.position.y = py_p.at(i);
    point.pose.position.z = pz_p.at(i);
    point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(pyaw_p.at(i));

    point.longitudinal_velocity_mps = tlx_p.at(i);
    point.heading_rate_rps = taz_p.at(i);
    point.acceleration_mps2 = alx_p.at(i);
    resampled_trajectory.points.at(i) = point;
  }

  return resampled_trajectory;
}

boost::optional<SBoundaries> ObstacleVelocityPlanner::getSBoundaries(
  const geometry_msgs::msg::Pose & current_pose, const TrajectoryData & ego_traj_data,
  const std::vector<double> & time_vec)
{
  if (!in_objects_ptr_ || ego_traj_data.traj.points.empty()) {
    return boost::none;
  }
  const auto surrounding_lanelets = getSurroundingLanelets(current_pose);

  SBoundaries s_boundaries(time_vec.size());
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    s_boundaries.at(i).max_s = ego_traj_data.s.back();
  }

  const auto obj_base_time = rclcpp::Time(in_objects_ptr_->header.stamp);
  for (const auto & obj : in_objects_ptr_->objects) {
    // Step1. Ignore obstacles that are not vehicles
    if (
      obj.classification.at(0).label != autoware_auto_perception_msgs::msg::ObjectClassification::CAR &&
      obj.classification.at(0).label != autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK &&
      obj.classification.at(0).label != autoware_auto_perception_msgs::msg::ObjectClassification::BUS &&
      obj.classification.at(0).label != autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE) {
      // RCLCPP_DEBUG(get_logger(), "Ignore non-vehicle obstacle");
      continue;
    }

    // Step2. Check if the object is on the map
    if (use_hd_map_ && !checkOnMapObject(obj, surrounding_lanelets)) {
      continue;
    }

    // Step3 Check object position
    if (!checkIsFrontObject(obj, ego_traj_data.traj)) {
      // RCLCPP_DEBUG(get_logger(), "Ignore obstacle behind the trajectory");
      continue;
    }

    // Step4 Check the predicted path
    if (obj.kinematics.predicted_paths.empty()) {
      // RCLCPP_DEBUG(get_logger(), "This object does not have predicted paths");
      continue;
    }

    // Step5 Get S boundary from the obstacle
    const auto obj_s_boundaries = getSBoundaries(ego_traj_data, obj, obj_base_time, time_vec);
    if (!obj_s_boundaries) {
      continue;
    }

    // Step6 update s boundaries
    for (size_t i = 0; i < obj_s_boundaries->size(); ++i) {
      if (obj_s_boundaries->at(i).max_s < s_boundaries.at(i).max_s) {
        s_boundaries.at(i) = obj_s_boundaries->at(i);
      }
    }
  }

  return s_boundaries;
}

boost::optional<SBoundaries> ObstacleVelocityPlanner::getSBoundaries(
  const TrajectoryData & ego_traj_data, const autoware_auto_perception_msgs::msg::PredictedObject & object, const rclcpp::Time & obj_base_time,
  const std::vector<double> & time_vec)
{
  const auto current_time = get_clock()->now();

  // Get the predicted path, which has the most high confidence
  const double max_horizon = time_vec.back();
  const auto predicted_path =
    resampledPredictedPath(object, obj_base_time, current_time, time_vec, max_horizon);
  if (!predicted_path) {
    RCLCPP_DEBUG(get_logger(), "Closest Obstacle does not have a predicted path");
    return boost::none;
  }

  // Get current pose from object's predicted path
  const auto current_object_pose =
    getCurrentObjectPoseFromPredictedPath(*predicted_path, obj_base_time, current_time);
  if (!current_object_pose) {
    RCLCPP_DEBUG(get_logger(), "Failed to get current pose from the predicted path");
    return boost::none;
  }

  // Check current object.kinematics
  ObjectData obj_data;
  obj_data.pose = current_object_pose.get();
  obj_data.length = object.shape.dimensions.x;
  obj_data.width = object.shape.dimensions.y;
  obj_data.time =
    std::max((obj_base_time - current_time).seconds(), 0.0);
  const double current_delta_yaw_threshold = 3.14;
  const auto current_collision_dist =
    getDistanceToCollisionPoint(ego_traj_data, obj_data, current_delta_yaw_threshold);

  // Calculate Safety Distance
  const double ego_vehicle_offset = wheel_base_ + front_overhang_;
  const double object_offset = obj_data.length / 2.0;
  const double safety_distance = ego_vehicle_offset + object_offset + safe_distance_margin_;

  // If the object is on the current ego trajectory,
  // we assume the object travels along ego trajectory
  if (current_collision_dist) {
    RCLCPP_DEBUG(get_logger(), "On Trajectory Object");

    return getSBoundaries(
      ego_traj_data, time_vec, safety_distance, object, *current_collision_dist);
  }

  // Ignore low velocity objects that are not on the trajectory
  const double obj_vel = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
  if (obj_vel > object_low_velocity_threshold_) {
    // RCLCPP_DEBUG(get_logger(), "Off Trajectory Object");
    return getSBoundaries(ego_traj_data, time_vec, safety_distance, object, obj_base_time, *predicted_path);
  }

  return boost::none;
}

boost::optional<SBoundaries> ObstacleVelocityPlanner::getSBoundaries(
  const TrajectoryData & ego_traj_data, const std::vector<double> & time_vec,
  const double safety_distance, const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const double dist_to_collision_point)
{
  SBoundaries s_boundaries(time_vec.size());
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    s_boundaries.at(i).max_s = ego_traj_data.s.back();
  }

  const double v_obj = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
  const double a_obj = object.kinematics.initial_acceleration_with_covariance.accel.linear.x;

  double current_v_obj = v_obj < object_zero_velocity_threshold_ ? 0.0 : v_obj;
  double current_s_obj = std::max(dist_to_collision_point - safety_distance, 0.0);
  const double initial_s_obj = current_s_obj;
  s_boundaries.front().max_s = current_s_obj;
  s_boundaries.front().is_object = true;
  for (size_t i = 1; i < time_vec.size(); ++i) {
    const double dt = time_vec.at(i) - time_vec.at(i - 1);
    if (i * dt <= 1.0 && use_object_acceleration_) {
      current_s_obj =
        std::max(current_s_obj + current_v_obj * dt + 0.5 * a_obj * dt * dt, initial_s_obj);
      current_v_obj = std::max(current_v_obj + a_obj * dt, 0.0);
    } else {
      current_s_obj = std::max(current_s_obj + current_v_obj * dt, initial_s_obj);
    }

    double s_upper_bound =
      current_s_obj + (current_v_obj * current_v_obj) / (2 * std::fabs(min_object_accel_)) - 0.5 * max_accel_ * t_idling_ * t_idling_ - 0.5 * max_accel_ * max_accel_ * t_idling_ * t_idling_ / std::fabs(min_accel_);
    s_upper_bound = std::max(s_upper_bound, 0.0);
    if (s_upper_bound < s_boundaries.at(i).max_s) {
      s_boundaries.at(i).max_s = s_upper_bound;
      s_boundaries.at(i).is_object = true;
    }
  }

  RCLCPP_DEBUG(get_logger(), "v_obj %f", v_obj);
  RCLCPP_DEBUG(
    get_logger(), "distance to obstacle %f",
    dist_to_collision_point - safety_distance + safe_distance_margin_);

  return s_boundaries;
}

boost::optional<SBoundaries> ObstacleVelocityPlanner::getSBoundaries(
  const TrajectoryData & ego_traj_data, const std::vector<double> & time_vec,
  const double safety_distance, const autoware_auto_perception_msgs::msg::PredictedObject & object,const rclcpp::Time & obj_base_time,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path)
{
  const auto current_time = get_clock()->now();
  const double abs_obj_vel = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
  const double v_obj =
    abs_obj_vel < object_zero_velocity_threshold_
      ? 0.0
      : abs_obj_vel;

  SBoundaries s_boundaries(time_vec.size());
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    s_boundaries.at(i).max_s = ego_traj_data.s.back();
  }

  for (size_t predicted_path_id = 0; predicted_path_id < predicted_path.path.size();
       ++predicted_path_id) {
    const auto predicted_pose = predicted_path.path.at(predicted_path_id);
    const double object_time = (obj_base_time - current_time).seconds();
    if (object_time < 0) {
      // Ignore Past Positions
      continue;
    }

    ObjectData obj_data;
    obj_data.pose = predicted_pose;
    obj_data.length = object.shape.dimensions.x;
    obj_data.width = object.shape.dimensions.y;
    obj_data.time = object_time;

    const auto dist_to_collision_point =
      getDistanceToCollisionPoint(ego_traj_data, obj_data, delta_yaw_threshold_of_object_and_ego_);
    if (!dist_to_collision_point) {
      continue;
    }

    const double s_upper_bound = std::max(*dist_to_collision_point - safety_distance, 0.0);
    for (size_t i = 0; i < predicted_path_id; ++i) {
      if (s_upper_bound < s_boundaries.at(i).max_s) {
        s_boundaries.at(i).max_s =
          s_upper_bound + (v_obj * v_obj) / (2 * std::fabs(min_object_accel_));
        s_boundaries.at(i).is_object = true;
      }
    }
  }

  return s_boundaries;
}

boost::optional<double> ObstacleVelocityPlanner::getDistanceToCollisionPoint(
  const TrajectoryData & ego_traj_data, const ObjectData & obj_data,
  const double delta_yaw_threshold)
{
  const auto obj_pose = obj_data.pose;
  const auto obj_length = obj_data.length;
  const auto obj_width = obj_data.width;

  // check diff yaw
  const size_t nearest_idx =
    tier4_autoware_utils::findNearestIndex(ego_traj_data.traj.points, obj_pose.position);
  const double ego_yaw = tf2::getYaw(ego_traj_data.traj.points.at(nearest_idx).pose.orientation);
  const double obj_yaw = tf2::getYaw(obj_pose.orientation);
  const double diff_yaw = tier4_autoware_utils::normalizeRadian(obj_yaw - ego_yaw);
  if (diff_yaw > delta_yaw_threshold) {
    // ignore object whose yaw difference from ego is too large
    RCLCPP_DEBUG(get_logger(), "Ignore object since the yaw difference is above the threshold");
    return boost::none;
  }

  const auto object_box = Box2d(obj_pose, obj_length, obj_width);
  const auto object_points = object_box.GetAllCorners();

  // Get nearest segment index for each point
  size_t min_nearest_idx = ego_traj_data.s.size();
  size_t max_nearest_idx = 0;
  for (const auto & obj_p : object_points) {
    size_t nearest_idx = tier4_autoware_utils::findNearestSegmentIndex(ego_traj_data.traj.points, obj_p);
    min_nearest_idx = std::min(nearest_idx, min_nearest_idx);
    max_nearest_idx = std::max(nearest_idx, max_nearest_idx);
  }

  double min_len = 0.0;
  size_t start_idx = 0;
  for (size_t i = min_nearest_idx; i > 0; --i) {
    min_len += (ego_traj_data.s.at(i) - ego_traj_data.s.at(i - 1));
    if (min_len > 5.0) {
      start_idx = i - 1;
      break;
    }
  }

  double max_len = 0.0;
  size_t end_idx = ego_traj_data.s.size() - 1;
  for (size_t i = max_nearest_idx; i < ego_traj_data.s.size() - 1; ++i) {
    max_len += (ego_traj_data.s.at(i + 1) - ego_traj_data.s.at(i));
    if (max_len > 5.0) {
      end_idx = i + 1;
      break;
    }
  }

  // Check collision
  const auto collision_idx = getCollisionIdx(ego_traj_data, object_box, start_idx, end_idx);

  if (collision_idx) {
    // TODO: Consider the time difference between ego vehicle and objects
    return getObjectLongitudinalPosition(ego_traj_data, obj_pose);
  }

  return boost::none;
}

double ObstacleVelocityPlanner::getObjectLongitudinalPosition(
  const TrajectoryData & traj_data, const geometry_msgs::msg::Pose & obj_pose)
{
  const auto closest_segment_idx =
    tier4_autoware_utils::findNearestSegmentIndex(traj_data.traj.points, obj_pose.position);
  const double s_segment = tier4_autoware_utils::calcLongitudinalOffsetToSegment(
    traj_data.traj.points, closest_segment_idx, obj_pose.position);
  return traj_data.s.at(closest_segment_idx) + s_segment;
}

boost::optional<size_t> ObstacleVelocityPlanner::getCollisionIdx(
  const TrajectoryData & ego_traj, const Box2d & obj_box, const size_t start_idx,
  const size_t end_idx)
{
  for (size_t ego_idx = start_idx; ego_idx <= end_idx; ++ego_idx) {
    const auto ego_center_pose = transformBaseLink2Center(ego_traj.traj.points.at(ego_idx).pose);
    const auto ego_box = Box2d(ego_center_pose, vehicle_length_, vehicle_width_);
    if (ego_box.HasOverlap(obj_box)) {
      return ego_idx;
    }
  }

  return boost::none;
}

bool ObstacleVelocityPlanner::checkOnMapObject(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const lanelet::ConstLanelets & valid_lanelets)
{
  // If we do not have a map, return true
  if (!lanelet_map_ptr_) {
    return true;
  }

  // obstacle point
  lanelet::BasicPoint2d search_point(
    object.kinematics.initial_pose_with_covariance.pose.position.x, object.kinematics.initial_pose_with_covariance.pose.position.y);

  // nearest lanelet
  std::vector<std::pair<double, lanelet::Lanelet>> surrounding_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  // No Closest Lanelets
  if (surrounding_lanelets.empty()) {
    return false;
  }

  // Check if the vehicle is inside the lanelet
  bool has_find_close_lanelet = false;
  for (const auto & lanelet : surrounding_lanelets) {
    if (lanelet::geometry::inside(lanelet.second, search_point)) {
      has_find_close_lanelet = true;
      //return true;
      break;
    }
  }

  if (!has_find_close_lanelet) {
    // This object is out of the any lanelets in this map
    return false;
  }

  for (const auto & valid_lanelet : valid_lanelets) {
    if (lanelet::geometry::inside(valid_lanelet, search_point)) {
      return true;
    }
  }

  return false;
}

lanelet::ConstLanelets ObstacleVelocityPlanner::getSurroundingLanelets(
  const geometry_msgs::msg::Pose & current_pose)
{
  // If we do not have a map, return true
  if (!lanelet_map_ptr_) {
    return {};
  }

  const lanelet::BasicPoint2d search_point(current_pose.position.x, current_pose.position.y);

  // nearest lanelet
  const std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  // Get Current Lanelets
  lanelet::Lanelets current_lanelets;
  for (const auto & lanelet : nearest_lanelets) {
    if (lanelet::geometry::inside(lanelet.second, search_point)) {
      current_lanelets.push_back(lanelet.second);
    }
  }

  lanelet::ConstLanelets surrounding_lanelets;

  const double initial_search_dist = 200.0;
  const double delta_search_dist = 50.0;
  for (const auto & current_lanelet : current_lanelets) {
    // Step1.1 Get the left lanelet
    lanelet::routing::LaneletPaths left_paths;
    auto opt_left = routing_graph_ptr_->left(current_lanelet);
    if (!!opt_left) {
      for (double search_dist = initial_search_dist; search_dist > 0;
           search_dist -= delta_search_dist) {
        const auto tmp_paths = routing_graph_ptr_->possiblePaths(*opt_left, search_dist, 0, false);
        addValidLanelet(tmp_paths, surrounding_lanelets);
      }
    }

    // Step1.2 Get the right lanelet
    lanelet::routing::LaneletPaths right_paths;
    auto opt_right = routing_graph_ptr_->right(current_lanelet);
    if (!!opt_right) {
      for (double search_dist = initial_search_dist; search_dist > 0;
           search_dist -= delta_search_dist) {
        const auto tmp_paths = routing_graph_ptr_->possiblePaths(*opt_right, search_dist, 0, false);
        addValidLanelet(tmp_paths, surrounding_lanelets);
      }
    }

    // Step1.3 Get the centerline
    lanelet::routing::LaneletPaths center_paths;
    for (double search_dist = initial_search_dist; search_dist > 0;
         search_dist -= delta_search_dist) {
      const auto tmp_paths =
        routing_graph_ptr_->possiblePaths(current_lanelet, search_dist, 0, false);
      addValidLanelet(tmp_paths, surrounding_lanelets);
    }
  }

  return surrounding_lanelets;
}

void ObstacleVelocityPlanner::addValidLanelet(
  const lanelet::routing::LaneletPaths & candidate_paths, lanelet::ConstLanelets & valid_lanelets)
{
  // Check if candidate paths are already in the valid paths
  for (const auto & candidate_path : candidate_paths) {
    for (const auto & candidate_lanelet : candidate_path) {
      const bool is_new_lanelet =
        std::find(valid_lanelets.begin(), valid_lanelets.end(), candidate_lanelet) ==
        valid_lanelets.end();
      if (is_new_lanelet) {
        valid_lanelets.push_back(candidate_lanelet);
      }
    }
  }
}

bool ObstacleVelocityPlanner::checkIsFrontObject(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const autoware_auto_planning_msgs::msg::Trajectory & traj)
{
  const auto point = object.kinematics.initial_pose_with_covariance.pose.position;

  // Get nearest segment index
  size_t nearest_idx = tier4_autoware_utils::findNearestSegmentIndex(traj.points, point);

  // Calculate longitudinal length
  const double l = tier4_autoware_utils::calcLongitudinalOffsetToSegment(traj.points, nearest_idx, point);

  if (nearest_idx == 0 && l < 0) {
    return false;
  }

  return true;
}

boost::optional<autoware_auto_perception_msgs::msg::PredictedPath>
ObstacleVelocityPlanner::resampledPredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const rclcpp::Time & obj_base_time, const rclcpp::Time & current_time,
  const std::vector<double> & resolutions, const double horizon)
{
  if (object.kinematics.predicted_paths.empty()) {
    return boost::none;
  }

  // Get the most reliable path
  const auto reliable_path = std::max_element(
    object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
    [](
      const autoware_auto_perception_msgs::msg::PredictedPath & a,
      const autoware_auto_perception_msgs::msg::PredictedPath & b) {
      return a.confidence < b.confidence;
    });

  // Resample Predicted Path
  const double duration = std::min(
    std::max((obj_base_time - current_time).seconds(), 0.0), horizon);

  // Calculate relative valid time vector
  // rel_valid_time_vec is relative to obj_base_time.
  const auto rel_valid_time_vec =
    resampling::resampledValidRelativeTimeVector(current_time, obj_base_time, resolutions, duration);

  return resampling::resamplePredictedPath(*reliable_path, rel_valid_time_vec);
}

boost::optional<geometry_msgs::msg::Pose>
ObstacleVelocityPlanner::getCurrentObjectPoseFromPredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path, const rclcpp::Time & obj_base_time,
  const rclcpp::Time & current_time)
{
  for (const auto & obj_p : predicted_path.path) {
    const double object_time = (obj_base_time - current_time).seconds();
    if (object_time >= 0) {
      return obj_p;
    }
  }

  return boost::none;
}

geometry_msgs::msg::Pose ObstacleVelocityPlanner::transformBaseLink2Center(
  const geometry_msgs::msg::Pose & pose_base_link)
{
  tf2::Transform tf_map2base;
  tf2::fromMsg(pose_base_link, tf_map2base);

  // set vertices at map coordinate
  tf2::Vector3 base2center;
  base2center.setX(std::fabs(vehicle_length_ / 2.0 - rear_overhang_));
  base2center.setY(0.0);
  base2center.setZ(0.0);
  base2center.setW(1.0);

  // transform vertices from map coordinate to object coordinate
  const auto map2center = tf_map2base * base2center;

  geometry_msgs::msg::Pose center_pose;
  center_pose.position =
    tier4_autoware_utils::createPoint(map2center.x(), map2center.y(), map2center.z());
  center_pose.orientation = pose_base_link.orientation;

  return center_pose;
}

boost::optional<VelocityOptimizer::OptimizationResult>
ObstacleVelocityPlanner::processOptimizedResult(
  const double v0, const VelocityOptimizer::OptimizationResult & opt_result)
{
  if (
    opt_result.t.empty() || opt_result.s.empty() || opt_result.v.empty() || opt_result.a.empty() ||
    opt_result.j.empty()) {
    return boost::none;
  }

  size_t break_id = opt_result.s.size();
  VelocityOptimizer::OptimizationResult processed_result;
  processed_result.t.push_back(0.0);
  processed_result.s.push_back(0.0);
  processed_result.v.push_back(v0);
  processed_result.a.push_back(opt_result.a.front());
  processed_result.j.push_back(opt_result.j.front());

  for (size_t i = 1; i < opt_result.s.size(); ++i) {
    const double prev_s = processed_result.s.back();
    const double current_s = std::max(opt_result.s.at(i), 0.0);
    if (prev_s >= current_s) {
      processed_result.v.back() = 0.0;
      break_id = i;
      break;
    }

    const double current_v = opt_result.v.at(i);
    if (current_v < ZERO_VEL_THRESHOLD) {
      break_id = i;
      break;
    }

    if (std::fabs(current_s - prev_s) < CLOSE_S_DIST_THRESHOLD) {
      processed_result.v.back() = current_v;
      processed_result.s.back() = prev_s > 0.0 ? current_s : processed_result.s.back();
      continue;
    }

    processed_result.t.push_back(opt_result.t.at(i));
    processed_result.s.push_back(current_s);
    processed_result.v.push_back(current_v);
    processed_result.a.push_back(opt_result.a.at(i));
    processed_result.j.push_back(opt_result.j.at(i));
  }

  // Padding Zero Velocity after break id
  for (size_t i = break_id; i < opt_result.s.size(); ++i) {
    const double prev_s = processed_result.s.back();
    const double current_s = std::max(opt_result.s.at(i), 0.0);
    if (prev_s >= current_s) {
      processed_result.v.back() = 0.0;
      continue;
    } else if (std::fabs(current_s - prev_s) < CLOSE_S_DIST_THRESHOLD) {
      processed_result.s.back() = prev_s > 0.0 ? current_s : processed_result.s.back();
      continue;
    }
    processed_result.t.push_back(opt_result.t.at(i));
    processed_result.s.push_back(current_s);
    processed_result.v.push_back(0.0);
    processed_result.a.push_back(0.0);
    processed_result.j.push_back(0.0);
  }

  return processed_result;
}

void ObstacleVelocityPlanner::publishDebugTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const size_t closest_idx,
  const std::vector<double> & time_vec, const SBoundaries & s_boundaries, const VelocityOptimizer::OptimizationResult & opt_result)
{
  const std::vector<double> time = opt_result.t;
  // Publish optimized result and boundary
  autoware_auto_planning_msgs::msg::Trajectory boundary_traj;
  boundary_traj.header.stamp = get_clock()->now();
  boundary_traj.points.resize(s_boundaries.size());
  double boundary_s_max = 0.0;
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    const double bound_s = s_boundaries.at(i).max_s;
    const double bound_t = time_vec.at(i);
    boundary_traj.points.at(i).pose.position.x = bound_s;
    boundary_traj.points.at(i).pose.position.y = bound_t;
    boundary_s_max = std::max(bound_s, boundary_s_max);
  }
  boundary_pub_->publish(boundary_traj);

  const double s_before = tier4_autoware_utils::calcSignedArcLength(traj.points, 0, closest_idx);
  autoware_auto_planning_msgs::msg::Trajectory optimized_sv_traj;
  optimized_sv_traj.header.stamp = get_clock()->now();
  optimized_sv_traj.points.resize(opt_result.s.size());
  for (size_t i = 0; i < opt_result.s.size(); ++i) {
    const double s = opt_result.s.at(i);
    const double v = opt_result.v.at(i);
    optimized_sv_traj.points.at(i).pose.position.x = s + s_before;
    optimized_sv_traj.points.at(i).pose.position.y = v;
  }
  optimized_sv_pub_->publish(optimized_sv_traj);

  autoware_auto_planning_msgs::msg::Trajectory optimized_st_graph;
  optimized_st_graph.header.stamp = get_clock()->now();
  optimized_st_graph.points.resize(opt_result.s.size());
  for (size_t i = 0; i < opt_result.s.size(); ++i) {
    const double bound_s = opt_result.s.at(i);
    const double bound_t = opt_result.t.at(i);
    optimized_st_graph.points.at(i).pose.position.x = bound_s;
    optimized_st_graph.points.at(i).pose.position.y = bound_t;
  }
  optimized_st_graph_pub_->publish(optimized_st_graph);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ObstacleVelocityPlanner)
