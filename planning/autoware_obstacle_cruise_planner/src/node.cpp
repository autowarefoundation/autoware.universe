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

#include "autoware/obstacle_cruise_planner/node.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware/universe_utils/ros/marker_helper.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"

#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
VelocityLimitClearCommand createVelocityLimitClearCommandMessage(
  const rclcpp::Time & current_time, const std::string & module_name)
{
  VelocityLimitClearCommand msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_cruise_planner." + module_name;
  msg.command = true;
  return msg;
}

template <typename T>
std::optional<T> getObstacleFromUuid(
  const std::vector<T> & obstacles, const std::string & target_uuid)
{
  const auto itr = std::find_if(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid == target_uuid;
  });

  if (itr == obstacles.end()) {
    return std::nullopt;
  }
  return *itr;
}

std::optional<double> calcDistanceToFrontVehicle(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_idx,
  const geometry_msgs::msg::Point & obstacle_pos)
{
  const size_t obstacle_idx = autoware::motion_utils::findNearestIndex(traj_points, obstacle_pos);
  const auto ego_to_obstacle_distance =
    autoware::motion_utils::calcSignedArcLength(traj_points, ego_idx, obstacle_idx);
  if (ego_to_obstacle_distance < 0.0) return std::nullopt;
  return ego_to_obstacle_distance;
}

std::vector<PredictedPath> resampleHighestConfidencePredictedPaths(
  const std::vector<PredictedPath> & predicted_paths, const double time_interval,
  const double time_horizon, const size_t num_paths)
{
  std::vector<PredictedPath> sorted_paths = predicted_paths;

  // Sort paths by descending confidence
  std::sort(
    sorted_paths.begin(), sorted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence > b.confidence; });

  std::vector<PredictedPath> selected_paths;
  size_t path_count = 0;

  // Select paths that meet the confidence thresholds
  for (const auto & path : sorted_paths) {
    if (path_count < num_paths) {
      selected_paths.push_back(path);
      ++path_count;
    }
  }

  // Resample each selected path
  std::vector<PredictedPath> resampled_paths;
  for (const auto & path : selected_paths) {
    if (path.path.size() < 2) {
      continue;
    }
    resampled_paths.push_back(
      autoware::object_recognition_utils::resamplePredictedPath(path, time_interval, time_horizon));
  }

  return resampled_paths;
}

double calcDiffAngleAgainstTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & target_pose)
{
  const size_t nearest_idx =
    autoware::motion_utils::findNearestIndex(traj_points, target_pose.position);
  const double traj_yaw = tf2::getYaw(traj_points.at(nearest_idx).pose.orientation);

  const double target_yaw = tf2::getYaw(target_pose.orientation);

  const double diff_yaw = autoware::universe_utils::normalizeRadian(target_yaw - traj_yaw);
  return diff_yaw;
}

/**
 * @brief Calculates the obstacle's longitudinal and approach velocities relative to the trajectory.
 *
 * This function calculates the obstacle's velocity components relative to the trajectory.
 * It returns the longitudinal and approach components of the obstacle's velocity
 * with respect to the trajectory. Negative approach velocity indicates that the
 * obstacle is getting far away from the trajectory.
 *
 * @param traj_points The trajectory points.
 * @param obstacle_pose The current pose of the obstacle.
 * @param obstacle_twist The twist (velocity) of the obstacle.
 * @return A pair containing the longitudinal and approach velocity components.
 */
std::pair<double, double> calculateObstacleVelocitiesRelativeToTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & obstacle_pose,
  const geometry_msgs::msg::Twist & obstacle_twist)
{
  const size_t object_idx =
    autoware::motion_utils::findNearestIndex(traj_points, obstacle_pose.position);

  const auto & nearest_point = traj_points.at(object_idx);

  const double traj_yaw = tf2::getYaw(nearest_point.pose.orientation);
  const double obstacle_yaw = tf2::getYaw(obstacle_pose.orientation);
  const Eigen::Rotation2Dd R_ego_to_obstacle(
    autoware::universe_utils::normalizeRadian(obstacle_yaw - traj_yaw));

  // Calculate the trajectory direction and the vector from the trajectory to the obstacle
  const Eigen::Vector2d traj_direction(std::cos(traj_yaw), std::sin(traj_yaw));
  const Eigen::Vector2d traj_to_obstacle(
    obstacle_pose.position.x - nearest_point.pose.position.x,
    obstacle_pose.position.y - nearest_point.pose.position.y);

  // Determine if the obstacle is to the left or right of the trajectory using the cross product
  const double cross_product =
    traj_direction.x() * traj_to_obstacle.y() - traj_direction.y() * traj_to_obstacle.x();
  const int sign = (cross_product > 0) ? -1 : 1;

  const Eigen::Vector2d obstacle_velocity(obstacle_twist.linear.x, obstacle_twist.linear.y);
  const Eigen::Vector2d projected_velocity = R_ego_to_obstacle * obstacle_velocity;

  return std::make_pair(projected_velocity[0], sign * projected_velocity[1]);
}

double calcObstacleMaxLength(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in obstacle_cruise_planner.");
}

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point, const bool is_driving_forward)
{
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = autoware::universe_utils::calcOffsetPose(
    goal_point.pose, extend_distance * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
  extend_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extend_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extend_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extend_trajectory_point;
}

std::vector<TrajectoryPoint> extendTrajectoryPoints(
  const std::vector<TrajectoryPoint> & input_points, const double extend_distance,
  const double step_length)
{
  auto output_points = input_points;
  const auto is_driving_forward_opt =
    autoware::motion_utils::isDrivingForwardWithTwist(input_points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  if (extend_distance < std::numeric_limits<double>::epsilon()) {
    return output_points;
  }

  const auto goal_point = input_points.back();

  double extend_sum = 0.0;
  while (extend_sum <= (extend_distance - step_length)) {
    const auto extend_trajectory_point =
      getExtendTrajectoryPoint(extend_sum, goal_point, is_driving_forward);
    output_points.push_back(extend_trajectory_point);
    extend_sum += step_length;
  }
  const auto extend_trajectory_point =
    getExtendTrajectoryPoint(extend_distance, goal_point, is_driving_forward);
  output_points.push_back(extend_trajectory_point);

  return output_points;
}

std::vector<int> getTargetObjectType(rclcpp::Node & node, const std::string & param_prefix)
{
  std::unordered_map<std::string, int> types_map{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK},     {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN}};

  std::vector<int> types;
  for (const auto & type : types_map) {
    if (node.declare_parameter<bool>(param_prefix + type.first)) {
      types.push_back(type.second);
    }
  }
  return types;
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(traj, interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

geometry_msgs::msg::Point toGeomPoint(const pcl::PointXYZ & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x;
  geom_point.y = point.y;
  geom_point.z = point.z;
  return geom_point;
}

geometry_msgs::msg::Point toGeomPoint(const autoware::universe_utils::Point2d & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  return geom_point;
}

bool isLowerConsideringHysteresis(
  const double current_val, const bool was_low, const double high_val, const double low_val)
{
  if (was_low) {
    if (high_val < current_val) {
      return false;
    }
    return true;
  }
  if (current_val < low_val) {
    return true;
  }
  return false;
}

template <typename T>
void concatenate(std::vector<T> & first, const std::vector<T> & last)
{
  first.insert(first.end(), last.begin(), last.end());
}
}  // namespace

namespace autoware::motion_planning
{
ObstacleCruisePlannerNode::BehaviorDeterminationParam::BehaviorDeterminationParam(
  rclcpp::Node & node)
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

void ObstacleCruisePlannerNode::BehaviorDeterminationParam::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
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
    parameters, "behavior_determination.pointcloud_min_cluster_size", pointcloud_min_cluster_size);
  autoware::universe_utils::updateParam<int>(
    parameters, "behavior_determination.pointcloud_max_cluster_size", pointcloud_max_cluster_size);
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

ObstacleCruisePlannerNode::ObstacleCruisePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_cruise_planner", node_options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  debug_data_ptr_(std::make_shared<DebugData>())
{
  using std::placeholders::_1;

  // subscriber
  traj_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&ObstacleCruisePlannerNode::onTrajectory, this, _1));

  // publisher
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  vel_limit_pub_ =
    create_publisher<VelocityLimit>("~/output/velocity_limit", rclcpp::QoS{1}.transient_local());
  clear_vel_limit_pub_ = create_publisher<VelocityLimitClearCommand>(
    "~/output/clear_velocity_limit", rclcpp::QoS{1}.transient_local());

  // debug publisher
  debug_calculation_time_pub_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);
  debug_cruise_wall_marker_pub_ = create_publisher<MarkerArray>("~/virtual_wall/cruise", 1);
  debug_stop_wall_marker_pub_ = create_publisher<MarkerArray>("~/virtual_wall/stop", 1);
  debug_slow_down_wall_marker_pub_ = create_publisher<MarkerArray>("~/virtual_wall/slow_down", 1);
  debug_marker_pub_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  debug_stop_planning_info_pub_ =
    create_publisher<Float32MultiArrayStamped>("~/debug/stop_planning_info", 1);
  debug_cruise_planning_info_pub_ =
    create_publisher<Float32MultiArrayStamped>("~/debug/cruise_planning_info", 1);
  debug_slow_down_planning_info_pub_ =
    create_publisher<Float32MultiArrayStamped>("~/debug/slow_down_planning_info", 1);

  // tf listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto longitudinal_info = LongitudinalInfo(*this);

  ego_nearest_param_ = EgoNearestParam(*this);

  enable_debug_info_ = declare_parameter<bool>("common.enable_debug_info");
  enable_calculation_time_info_ = declare_parameter<bool>("common.enable_calculation_time_info");
  enable_slow_down_planning_ = declare_parameter<bool>("common.enable_slow_down_planning");

  use_pointcloud_for_stop_ = declare_parameter<bool>("common.stop_obstacle_type.pointcloud");
  use_pointcloud_for_slow_down_ =
    declare_parameter<bool>("common.slow_down_obstacle_type.pointcloud");
  use_pointcloud_ = use_pointcloud_for_stop_ || use_pointcloud_for_slow_down_;

  behavior_determination_param_ = BehaviorDeterminationParam(*this);

  {  // planning algorithm
    const std::string planning_algorithm_param =
      declare_parameter<std::string>("common.planning_algorithm");
    planning_algorithm_ = getPlanningAlgorithmType(planning_algorithm_param);

    if (planning_algorithm_ == PlanningAlgorithm::OPTIMIZATION_BASE) {
      planner_ptr_ = std::make_unique<OptimizationBasedPlanner>(
        *this, longitudinal_info, vehicle_info_, ego_nearest_param_, debug_data_ptr_);
    } else if (planning_algorithm_ == PlanningAlgorithm::PID_BASE) {
      planner_ptr_ = std::make_unique<PIDBasedPlanner>(
        *this, longitudinal_info, vehicle_info_, ego_nearest_param_, debug_data_ptr_);
    } else {
      throw std::logic_error("Designated algorithm is not supported.");
    }

    min_behavior_stop_margin_ = declare_parameter<double>("common.min_behavior_stop_margin");
    additional_safe_distance_margin_on_curve_ =
      declare_parameter<double>("common.stop_on_curve.additional_safe_distance_margin");
    enable_approaching_on_curve_ =
      declare_parameter<bool>("common.stop_on_curve.enable_approaching");
    min_safe_distance_margin_on_curve_ =
      declare_parameter<double>("common.stop_on_curve.min_safe_distance_margin");
    suppress_sudden_obstacle_stop_ =
      declare_parameter<bool>("common.suppress_sudden_obstacle_stop");
    planner_ptr_->setParam(
      enable_debug_info_, enable_calculation_time_info_, use_pointcloud_, min_behavior_stop_margin_,
      enable_approaching_on_curve_, additional_safe_distance_margin_on_curve_,
      min_safe_distance_margin_on_curve_, suppress_sudden_obstacle_stop_);
  }

  {  // stop/cruise/slow down obstacle type
    inside_stop_obstacle_types_ = getTargetObjectType(*this, "common.stop_obstacle_type.inside.");
    outside_stop_obstacle_types_ = getTargetObjectType(*this, "common.stop_obstacle_type.outside.");
    inside_cruise_obstacle_types_ =
      getTargetObjectType(*this, "common.cruise_obstacle_type.inside.");
    outside_cruise_obstacle_types_ =
      getTargetObjectType(*this, "common.cruise_obstacle_type.outside.");
    slow_down_obstacle_types_ = getTargetObjectType(*this, "common.slow_down_obstacle_type.");
  }

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleCruisePlannerNode::onParam, this, std::placeholders::_1));

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
}

ObstacleCruisePlannerNode::PlanningAlgorithm ObstacleCruisePlannerNode::getPlanningAlgorithmType(
  const std::string & param) const
{
  if (param == "pid_base") {
    return PlanningAlgorithm::PID_BASE;
  } else if (param == "optimization_base") {
    return PlanningAlgorithm::OPTIMIZATION_BASE;
  }
  return PlanningAlgorithm::INVALID;
}

rcl_interfaces::msg::SetParametersResult ObstacleCruisePlannerNode::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  planner_ptr_->onParam(parameters);

  autoware::universe_utils::updateParam<bool>(
    parameters, "common.enable_debug_info", enable_debug_info_);
  autoware::universe_utils::updateParam<bool>(
    parameters, "common.enable_calculation_time_info", enable_calculation_time_info_);

  autoware::universe_utils::updateParam<bool>(
    parameters, "common.stop_on_curve.enable_approaching", enable_approaching_on_curve_);
  autoware::universe_utils::updateParam<double>(
    parameters, "common.stop_on_curve.additional_safe_distance_margin",
    additional_safe_distance_margin_on_curve_);
  autoware::universe_utils::updateParam<double>(
    parameters, "common.stop_on_curve.min_safe_distance_margin",
    min_safe_distance_margin_on_curve_);

  planner_ptr_->setParam(
    enable_debug_info_, enable_calculation_time_info_, use_pointcloud_, min_behavior_stop_margin_,
    enable_approaching_on_curve_, additional_safe_distance_margin_on_curve_,
    min_safe_distance_margin_on_curve_, suppress_sudden_obstacle_stop_);

  autoware::universe_utils::updateParam<bool>(
    parameters, "common.enable_slow_down_planning", enable_slow_down_planning_);

  behavior_determination_param_.onParam(parameters);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void ObstacleCruisePlannerNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  const auto ego_odom_ptr = ego_odom_sub_.takeData();
  const auto objects_ptr = objects_sub_.takeData();
  const auto pointcloud_ptr = use_pointcloud_ ? pointcloud_sub_.takeData() : nullptr;
  const auto acc_ptr = acc_sub_.takeData();
  const bool can_detect_obstacles = objects_ptr || pointcloud_ptr;
  if (!ego_odom_ptr || !can_detect_obstacles || !acc_ptr) {
    return;
  }

  const auto & ego_odom = *ego_odom_ptr;
  const auto & acc = *acc_ptr;

  const auto traj_points = autoware::motion_utils::convertToTrajectoryPointArray(*msg);
  // check if subscribed variables are ready
  if (traj_points.empty()) {
    return;
  }

  stop_watch_.tic(__func__);
  *debug_data_ptr_ = DebugData();

  const auto is_driving_forward = autoware::motion_utils::isDrivingForwardWithTwist(traj_points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.value() : is_driving_forward_;

  const auto & [stop_obstacles, cruise_obstacles, slow_down_obstacles] = [&]() {
    std::vector<StopObstacle> stop_obstacles;
    std::vector<CruiseObstacle> cruise_obstacles;
    std::vector<SlowDownObstacle> slow_down_obstacles;
    if (objects_ptr) {
      // 1. Convert predicted objects to obstacles which are
      //    (1) with a proper label
      //    (2) in front of ego
      //    (3) not too far from trajectory
      const auto target_obstacles = convertToObstacles(ego_odom, *objects_ptr, traj_points);

      //  2. Determine ego's behavior against each obstacle from stop, cruise and slow down.
      const auto & [stop_object_obstacles, cruise_object_obstacles, slow_down_object_obstacles] =
        determineEgoBehaviorAgainstPredictedObjectObstacles(
          ego_odom, *objects_ptr, traj_points, target_obstacles);

      concatenate(stop_obstacles, stop_object_obstacles);
      concatenate(cruise_obstacles, cruise_object_obstacles);
      concatenate(slow_down_obstacles, slow_down_object_obstacles);
    }
    if (pointcloud_ptr) {
      const auto target_obstacles =
        convertToObstacles(ego_odom, *pointcloud_ptr, traj_points, msg->header);

      const auto & [stop_pc_obstacles, cruise_pc_obstacles, slow_down_pc_obstacles] =
        determineEgoBehaviorAgainstPointCloudObstacles(ego_odom, traj_points, target_obstacles);

      concatenate(stop_obstacles, stop_pc_obstacles);
      concatenate(cruise_obstacles, cruise_pc_obstacles);
      concatenate(slow_down_obstacles, slow_down_pc_obstacles);
    }
    return std::make_tuple(stop_obstacles, cruise_obstacles, slow_down_obstacles);
  }();

  // 3. Create data for planning
  const auto planner_data = createPlannerData(ego_odom, acc, traj_points);

  // 4. Stop planning
  const auto stop_traj_points = planner_ptr_->generateStopTrajectory(planner_data, stop_obstacles);

  // 5. Cruise planning
  std::optional<VelocityLimit> cruise_vel_limit;
  const auto cruise_traj_points = planner_ptr_->generateCruiseTrajectory(
    planner_data, stop_traj_points, cruise_obstacles, cruise_vel_limit);
  publishVelocityLimit(cruise_vel_limit, "cruise");

  // 6. Slow down planning
  std::optional<VelocityLimit> slow_down_vel_limit;
  const auto slow_down_traj_points = planner_ptr_->generateSlowDownTrajectory(
    planner_data, cruise_traj_points, slow_down_obstacles, slow_down_vel_limit);
  publishVelocityLimit(slow_down_vel_limit, "slow_down");

  // 7. Publish trajectory
  const auto output_traj =
    autoware::motion_utils::convertToTrajectory(slow_down_traj_points, msg->header);
  trajectory_pub_->publish(output_traj);

  // 8. Publish debug data
  published_time_publisher_->publish_if_subscribed(trajectory_pub_, output_traj.header.stamp);
  planner_ptr_->publishMetrics(now());
  planner_ptr_->publishPlanningFactors();
  publishDebugMarker();
  publishDebugInfo();
  objects_of_interest_marker_interface_.publishMarkerArray();

  // 9. Publish and print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  publishCalculationTime(calculation_time);
  RCLCPP_INFO_EXPRESSION(
    get_logger(), enable_calculation_time_info_, "%s := %f [ms]", __func__, calculation_time);
}

std::vector<Polygon2d> ObstacleCruisePlannerNode::createOneStepPolygons(
  const std::vector<TrajectoryPoint> & traj_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin) const
{
  const auto & p = behavior_determination_param_;

  const double front_length = vehicle_info.max_longitudinal_offset_m;
  const double rear_length = vehicle_info.rear_overhang_m;
  const double vehicle_width = vehicle_info.vehicle_width_m;

  const size_t nearest_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, current_ego_pose.position);
  const auto nearest_pose = traj_points.at(nearest_idx).pose;
  const auto current_ego_pose_error =
    autoware::universe_utils::inverseTransformPose(current_ego_pose, nearest_pose);
  const double current_ego_lat_error = current_ego_pose_error.position.y;
  const double current_ego_yaw_error = tf2::getYaw(current_ego_pose_error.orientation);
  double time_elapsed{0.0};

  std::vector<Polygon2d> output_polygons;
  Polygon2d tmp_polys{};
  for (size_t i = 0; i < traj_points.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> current_poses = {traj_points.at(i).pose};

    // estimate the future ego pose with assuming that the pose error against the reference path
    // will decrease to zero by the time_to_convergence
    if (p.enable_to_consider_current_pose && time_elapsed < p.time_to_convergence) {
      const double rem_ratio = (p.time_to_convergence - time_elapsed) / p.time_to_convergence;
      geometry_msgs::msg::Pose indexed_pose_err;
      indexed_pose_err.set__orientation(
        autoware::universe_utils::createQuaternionFromYaw(current_ego_yaw_error * rem_ratio));
      indexed_pose_err.set__position(
        autoware::universe_utils::createPoint(0.0, current_ego_lat_error * rem_ratio, 0.0));
      current_poses.push_back(
        autoware::universe_utils::transformPose(indexed_pose_err, traj_points.at(i).pose));
      if (traj_points.at(i).longitudinal_velocity_mps != 0.0) {
        time_elapsed +=
          p.decimate_trajectory_step_length / std::abs(traj_points.at(i).longitudinal_velocity_mps);
      } else {
        time_elapsed = std::numeric_limits<double>::max();
      }
    }

    Polygon2d idx_poly{};
    for (const auto & pose : current_poses) {
      if (i == 0 && traj_points.at(i).longitudinal_velocity_mps > 1e-3) {
        boost::geometry::append(
          idx_poly,
          autoware::universe_utils::toFootprint(pose, front_length, rear_length, vehicle_width)
            .outer());
        boost::geometry::append(
          idx_poly, autoware::universe_utils::fromMsg(
                      autoware::universe_utils::calcOffsetPose(
                        pose, front_length, vehicle_width * 0.5 + lat_margin, 0.0)
                        .position)
                      .to_2d());
        boost::geometry::append(
          idx_poly, autoware::universe_utils::fromMsg(
                      autoware::universe_utils::calcOffsetPose(
                        pose, front_length, -vehicle_width * 0.5 - lat_margin, 0.0)
                        .position)
                      .to_2d());
      } else {
        boost::geometry::append(
          idx_poly, autoware::universe_utils::toFootprint(
                      pose, front_length, rear_length, vehicle_width + lat_margin * 2.0)
                      .outer());
      }
    }

    boost::geometry::append(tmp_polys, idx_poly.outer());
    Polygon2d hull_polygon;
    boost::geometry::convex_hull(tmp_polys, hull_polygon);
    boost::geometry::correct(hull_polygon);

    output_polygons.push_back(hull_polygon);
    tmp_polys = std::move(idx_poly);
  }
  return output_polygons;
}

std::vector<Obstacle> ObstacleCruisePlannerNode::convertToObstacles(
  const Odometry & odometry, const PredictedObjects & objects,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  stop_watch_.tic(__func__);

  const auto obj_stamp = rclcpp::Time(objects.header.stamp);
  const auto & p = behavior_determination_param_;

  constexpr double epsilon = 1e-6;
  const double max_lat_margin = std::max(
    {p.max_lat_margin_for_stop, p.max_lat_margin_for_stop_against_unknown,
     p.max_lat_margin_for_cruise, p.max_lat_margin_for_slow_down});
  const double max_lat_time_margin =
    std::max({p.max_lat_time_margin_for_stop, p.max_lat_time_margin_for_cruise});
  const size_t ego_idx = ego_nearest_param_.findIndex(traj_points, odometry.pose.pose);

  std::vector<Obstacle> target_obstacles;
  for (const auto & predicted_object : objects.objects) {
    const auto & object_id =
      autoware::universe_utils::toHexString(predicted_object.object_id).substr(0, 4);

    // brkay54: When use_prediction is true, we observed wrong orientation for the object in
    // scenario simulator.
    const auto & current_obstacle_pose =
      obstacle_cruise_utils::getCurrentObjectPose(predicted_object, obj_stamp, now(), true);

    // 1. Check if the obstacle's label is target
    const uint8_t label = predicted_object.classification.front().label;
    const bool is_target_obstacle =
      isStopObstacle(label) || isCruiseObstacle(label) || isSlowDownObstacle(label);
    if (!is_target_obstacle) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_, "Ignore obstacle (%s) since its label is not target.",
        object_id.c_str());
      continue;
    }

    const auto projected_vel = calculateObstacleVelocitiesRelativeToTrajectory(
      traj_points, current_obstacle_pose.pose,
      predicted_object.kinematics.initial_twist_with_covariance.twist);

    // 2. Check if the obstacle is in front of the ego.
    const auto ego_to_obstacle_distance =
      calcDistanceToFrontVehicle(traj_points, ego_idx, current_obstacle_pose.pose.position);
    if (!ego_to_obstacle_distance) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_, "Ignore obstacle (%s) since it is not front obstacle.",
        object_id.c_str());
      continue;
    }

    // 3. Check if rough lateral distance and time to reach trajectory are smaller than the
    // threshold
    const double lat_dist_from_obstacle_to_traj =
      autoware::motion_utils::calcLateralOffset(traj_points, current_obstacle_pose.pose.position);

    const double min_lat_dist_to_traj_poly = [&]() {
      const double obstacle_max_length = calcObstacleMaxLength(predicted_object.shape);
      return std::abs(lat_dist_from_obstacle_to_traj) - vehicle_info_.vehicle_width_m -
             obstacle_max_length;
    }();

    if (max_lat_margin < min_lat_dist_to_traj_poly) {
      if (projected_vel.second > 0.0) {
        const auto time_to_traj =
          min_lat_dist_to_traj_poly / std::max(epsilon, projected_vel.second);
        if (time_to_traj > max_lat_time_margin) {
          RCLCPP_INFO_EXPRESSION(
            get_logger(), enable_debug_info_,
            "Ignore obstacle (%s) since it is too far from the trajectory.", object_id.c_str());
          continue;
        }
      } else {
        RCLCPP_INFO_EXPRESSION(
          get_logger(), enable_debug_info_,
          "Ignore obstacle (%s) since it is too far from the trajectory.", object_id.c_str());
        continue;
      }
    }

    const auto target_obstacle = Obstacle(
      obj_stamp, predicted_object, current_obstacle_pose.pose, *ego_to_obstacle_distance,
      lat_dist_from_obstacle_to_traj, projected_vel.first, projected_vel.second);
    target_obstacles.push_back(target_obstacle);
  }

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_debug_info_, "  %s := %f [ms]", __func__,
    calculation_time);

  return target_obstacles;
}

std::vector<Obstacle> ObstacleCruisePlannerNode::convertToObstacles(
  const Odometry & odometry, const PointCloud2 & pointcloud,
  const std::vector<TrajectoryPoint> & traj_points, const std_msgs::msg::Header & traj_header) const
{
  stop_watch_.tic(__func__);

  const auto & p = behavior_determination_param_;

  std::vector<Obstacle> target_obstacles;

  std::optional<geometry_msgs::msg::TransformStamped> transform_stamped{};
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      traj_header.frame_id, pointcloud.header.frame_id, pointcloud.header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to look up transform from " << traj_header.frame_id << " to "
                                                        << pointcloud.header.frame_id);
    transform_stamped = std::nullopt;
  }

  if (!pointcloud.data.empty() && transform_stamped) {
    // 1. transform pointcloud
    PointCloud::Ptr pointcloud_ptr(new PointCloud);
    pcl::fromROSMsg(pointcloud, *pointcloud_ptr);
    const Eigen::Matrix4f transform =
      tf2::transformToEigen(transform_stamped.value().transform).matrix().cast<float>();
    pcl::transformPointCloud(*pointcloud_ptr, *pointcloud_ptr, transform);

    // 2. downsample & cluster pointcloud
    PointCloud::Ptr filtered_points_ptr(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(pointcloud_ptr);
    filter.setLeafSize(
      p.pointcloud_voxel_grid_x, p.pointcloud_voxel_grid_y, p.pointcloud_voxel_grid_z);
    filter.filter(*filtered_points_ptr);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered_points_ptr);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(p.pointcloud_cluster_tolerance);
    ec.setMinClusterSize(p.pointcloud_min_cluster_size);
    ec.setMaxClusterSize(p.pointcloud_max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_points_ptr);
    ec.extract(clusters);

    const auto max_lat_margin =
      std::max(p.max_lat_margin_for_stop_against_unknown, p.max_lat_margin_for_slow_down);
    const size_t ego_idx = ego_nearest_param_.findIndex(traj_points, odometry.pose.pose);

    // 3. convert clusters to obstacles
    for (const auto & cluster_indices : clusters) {
      double ego_to_stop_collision_distance = std::numeric_limits<double>::max();
      double ego_to_slow_down_front_collision_distance = std::numeric_limits<double>::max();
      double ego_to_slow_down_back_collision_distance = std::numeric_limits<double>::min();
      double lat_dist_from_obstacle_to_traj = std::numeric_limits<double>::max();
      double ego_to_obstacle_distance = std::numeric_limits<double>::max();
      std::optional<geometry_msgs::msg::Point> stop_collision_point = std::nullopt;
      std::optional<geometry_msgs::msg::Point> slow_down_front_collision_point = std::nullopt;
      std::optional<geometry_msgs::msg::Point> slow_down_back_collision_point = std::nullopt;

      for (const auto & index : cluster_indices.indices) {
        const auto obstacle_point = toGeomPoint(filtered_points_ptr->points[index]);
        const auto current_lat_dist_from_obstacle_to_traj =
          autoware::motion_utils::calcLateralOffset(traj_points, obstacle_point);
        const auto min_lat_dist_to_traj_poly =
          std::abs(current_lat_dist_from_obstacle_to_traj) - vehicle_info_.vehicle_width_m;

        if (min_lat_dist_to_traj_poly < max_lat_margin) {
          const auto current_ego_to_obstacle_distance =
            calcDistanceToFrontVehicle(traj_points, ego_idx, obstacle_point);
          if (current_ego_to_obstacle_distance) {
            ego_to_obstacle_distance =
              std::min(ego_to_obstacle_distance, *current_ego_to_obstacle_distance);
          } else {
            continue;
          }

          lat_dist_from_obstacle_to_traj =
            std::min(lat_dist_from_obstacle_to_traj, current_lat_dist_from_obstacle_to_traj);

          if (min_lat_dist_to_traj_poly < p.max_lat_margin_for_stop_against_unknown) {
            if (*current_ego_to_obstacle_distance < ego_to_stop_collision_distance) {
              stop_collision_point = obstacle_point;
              ego_to_stop_collision_distance = *current_ego_to_obstacle_distance;
            }
          } else if (min_lat_dist_to_traj_poly < p.max_lat_margin_for_slow_down) {
            if (*current_ego_to_obstacle_distance < ego_to_slow_down_front_collision_distance) {
              slow_down_front_collision_point = obstacle_point;
              ego_to_slow_down_front_collision_distance = *current_ego_to_obstacle_distance;
            } else if (
              *current_ego_to_obstacle_distance > ego_to_slow_down_back_collision_distance) {
              slow_down_back_collision_point = obstacle_point;
              ego_to_slow_down_back_collision_distance = *current_ego_to_obstacle_distance;
            }
          }
        }
      }

      if (stop_collision_point || slow_down_front_collision_point) {
        target_obstacles.emplace_back(
          pointcloud.header.stamp, stop_collision_point, slow_down_front_collision_point,
          slow_down_back_collision_point, ego_to_obstacle_distance, lat_dist_from_obstacle_to_traj);
      }
    }
  }

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_debug_info_, "  %s := %f [ms]", __func__,
    calculation_time);

  return target_obstacles;
}

bool ObstacleCruisePlannerNode::isInsideStopObstacle(const uint8_t label) const
{
  const auto & types = inside_stop_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isOutsideStopObstacle(const uint8_t label) const
{
  const auto & types = outside_stop_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isStopObstacle(const uint8_t label) const
{
  return isInsideStopObstacle(label) || isOutsideStopObstacle(label);
}

bool ObstacleCruisePlannerNode::isInsideCruiseObstacle(const uint8_t label) const
{
  const auto & types = inside_cruise_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isOutsideCruiseObstacle(const uint8_t label) const
{
  const auto & types = outside_cruise_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isCruiseObstacle(const uint8_t label) const
{
  return isInsideCruiseObstacle(label) || isOutsideCruiseObstacle(label);
}

bool ObstacleCruisePlannerNode::isSlowDownObstacle(const uint8_t label) const
{
  const auto & types = slow_down_obstacle_types_;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruisePlannerNode::isFrontCollideObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
  const size_t first_collision_idx) const
{
  const auto obstacle_idx =
    autoware::motion_utils::findNearestIndex(traj_points, obstacle.pose.position);

  const double obstacle_to_col_points_distance =
    autoware::motion_utils::calcSignedArcLength(traj_points, obstacle_idx, first_collision_idx);
  const double obstacle_max_length = calcObstacleMaxLength(obstacle.shape);

  // If the obstacle is far in front of the collision point, the obstacle is behind the ego.
  return obstacle_to_col_points_distance > -obstacle_max_length;
}

std::tuple<std::vector<StopObstacle>, std::vector<CruiseObstacle>, std::vector<SlowDownObstacle>>
ObstacleCruisePlannerNode::determineEgoBehaviorAgainstPredictedObjectObstacles(
  const Odometry & odometry, const PredictedObjects & objects,
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Obstacle> & obstacles)
{
  stop_watch_.tic(__func__);

  // calculated decimated trajectory points and trajectory polygon
  const auto decimated_traj_points = decimateTrajectoryPoints(odometry, traj_points);
  const auto decimated_traj_polys =
    createOneStepPolygons(decimated_traj_points, vehicle_info_, odometry.pose.pose);
  debug_data_ptr_->detection_polygons = decimated_traj_polys;

  // determine ego's behavior from stop, cruise and slow down
  std::vector<StopObstacle> stop_obstacles;
  std::vector<CruiseObstacle> cruise_obstacles;
  std::vector<SlowDownObstacle> slow_down_obstacles;
  slow_down_condition_counter_.resetCurrentUuids();
  for (const auto & obstacle : obstacles) {
    const auto obstacle_poly = autoware::universe_utils::toPolygon2d(obstacle.pose, obstacle.shape);

    // Calculate distance between trajectory and obstacle first
    double precise_lat_dist = std::numeric_limits<double>::max();
    for (const auto & traj_poly : decimated_traj_polys) {
      const double current_precise_lat_dist = bg::distance(traj_poly, obstacle_poly);
      precise_lat_dist = std::min(precise_lat_dist, current_precise_lat_dist);
    }

    // Filter obstacles for cruise, stop and slow down
    const auto cruise_obstacle = createCruiseObstacle(
      odometry, decimated_traj_points, decimated_traj_polys, obstacle, precise_lat_dist);
    if (cruise_obstacle) {
      cruise_obstacles.push_back(*cruise_obstacle);
      continue;
    }
    const auto stop_obstacle = createStopObstacleForPredictedObject(
      odometry, decimated_traj_points, decimated_traj_polys, obstacle, precise_lat_dist);
    if (stop_obstacle) {
      stop_obstacles.push_back(*stop_obstacle);
      continue;
    }
    const auto slow_down_obstacle = createSlowDownObstacleForPredictedObject(
      odometry, decimated_traj_points, obstacle, precise_lat_dist);
    if (slow_down_obstacle) {
      slow_down_obstacles.push_back(*slow_down_obstacle);
      continue;
    }
  }
  const auto & p = behavior_determination_param_;
  if (p.enable_yield) {
    const auto yield_obstacles = findYieldCruiseObstacles(obstacles, decimated_traj_points);
    if (yield_obstacles) {
      for (const auto & y : yield_obstacles.value()) {
        // Check if there is no member with the same UUID in cruise_obstacles
        auto it = std::find_if(
          cruise_obstacles.begin(), cruise_obstacles.end(),
          [&y](const auto & c) { return y.uuid == c.uuid; });

        // If no matching UUID found, insert yield obstacle into cruise_obstacles
        if (it == cruise_obstacles.end()) {
          cruise_obstacles.push_back(y);
        }
      }
    }
  }
  slow_down_condition_counter_.removeCounterUnlessUpdated();

  // Check target obstacles' consistency
  checkConsistency(objects.header.stamp, objects, stop_obstacles);

  // update previous obstacles
  prev_stop_object_obstacles_ = stop_obstacles;
  prev_cruise_object_obstacles_ = cruise_obstacles;
  prev_slow_down_object_obstacles_ = slow_down_obstacles;

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_calculation_time_info_, "  %s := %f [ms]",
    __func__, calculation_time);

  return {stop_obstacles, cruise_obstacles, slow_down_obstacles};
}

std::tuple<std::vector<StopObstacle>, std::vector<CruiseObstacle>, std::vector<SlowDownObstacle>>
ObstacleCruisePlannerNode::determineEgoBehaviorAgainstPointCloudObstacles(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<Obstacle> & obstacles)
{
  stop_watch_.tic(__func__);

  const auto & p = behavior_determination_param_;

  // calculated decimated trajectory points and trajectory polygon
  const auto decimated_traj_points = decimateTrajectoryPoints(odometry, traj_points);
  const auto decimated_traj_polys =
    createOneStepPolygons(decimated_traj_points, vehicle_info_, odometry.pose.pose);
  debug_data_ptr_->detection_polygons = decimated_traj_polys;

  // determine ego's behavior from stop and slow down
  std::vector<StopObstacle> stop_obstacles;
  std::vector<SlowDownObstacle> slow_down_obstacles;
  for (const auto & obstacle : obstacles) {
    const auto & precise_lat_dist = obstacle.lat_dist_from_obstacle_to_traj;

    // Filter obstacles for stop and slow down
    const auto stop_obstacle =
      createStopObstacleForPointCloud(decimated_traj_points, obstacle, precise_lat_dist);
    if (stop_obstacle) {
      stop_obstacles.push_back(*stop_obstacle);
      continue;
    }
    const auto slow_down_obstacle = createSlowDownObstacleForPointCloud(obstacle, precise_lat_dist);
    if (slow_down_obstacle) {
      slow_down_obstacles.push_back(*slow_down_obstacle);
      continue;
    }
  }

  std::vector<StopObstacle> past_stop_obstacles;
  for (auto itr = stop_pc_obstacle_history_.begin(); itr != stop_pc_obstacle_history_.end();) {
    const double elapsed_time = (rclcpp::Time(odometry.header.stamp) - itr->stamp).seconds();
    if (elapsed_time >= p.stop_obstacle_hold_time_threshold) {
      itr = stop_pc_obstacle_history_.erase(itr);
      continue;
    }

    const auto lat_dist_from_obstacle_to_traj =
      autoware::motion_utils::calcLateralOffset(traj_points, itr->collision_point);
    const auto min_lat_dist_to_traj_poly =
      std::abs(lat_dist_from_obstacle_to_traj) - vehicle_info_.vehicle_width_m;

    if (min_lat_dist_to_traj_poly < p.max_lat_margin_for_stop_against_unknown) {
      auto stop_obstacle = *itr;
      stop_obstacle.dist_to_collide_on_decimated_traj = autoware::motion_utils::calcSignedArcLength(
        decimated_traj_points, 0, stop_obstacle.collision_point);
      past_stop_obstacles.push_back(stop_obstacle);
    }

    ++itr;
  }

  concatenate(stop_pc_obstacle_history_, stop_obstacles);
  concatenate(stop_obstacles, past_stop_obstacles);

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_calculation_time_info_, "  %s := %f [ms]",
    __func__, calculation_time);

  return {stop_obstacles, {}, slow_down_obstacles};
}

std::vector<TrajectoryPoint> ObstacleCruisePlannerNode::decimateTrajectoryPoints(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points) const
{
  const auto & p = behavior_determination_param_;

  // trim trajectory
  const size_t ego_seg_idx = ego_nearest_param_.findSegmentIndex(traj_points, odometry.pose.pose);
  const size_t traj_start_point_idx = ego_seg_idx;
  const auto trimmed_traj_points =
    std::vector<TrajectoryPoint>(traj_points.begin() + traj_start_point_idx, traj_points.end());

  // decimate trajectory
  const auto decimated_traj_points =
    resampleTrajectoryPoints(trimmed_traj_points, p.decimate_trajectory_step_length);

  // extend trajectory
  const auto extended_traj_points = extendTrajectoryPoints(
    decimated_traj_points, planner_ptr_->getSafeDistanceMargin(),
    p.decimate_trajectory_step_length);
  if (extended_traj_points.size() < 2) {
    return traj_points;
  }
  return extended_traj_points;
}

std::optional<CruiseObstacle> ObstacleCruisePlannerNode::createCruiseObstacle(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<Polygon2d> & traj_polys, const Obstacle & obstacle,
  const double precise_lat_dist)
{
  const auto & object_id = obstacle.uuid.substr(0, 4);
  const auto & p = behavior_determination_param_;

  // NOTE: When driving backward, Stop will be planned instead of cruise.
  //       When the obstacle is crossing the ego's trajectory, cruise can be ignored.
  if (!isCruiseObstacle(obstacle.classification.label) || !is_driving_forward_) {
    return std::nullopt;
  }

  if (obstacle.longitudinal_velocity < 0.0) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore obstacle (%s) since it's driving in opposite direction.", object_id.c_str());
    return std::nullopt;
  }

  if (p.max_lat_margin_for_cruise < precise_lat_dist) {
    const auto time_to_traj = precise_lat_dist / std::max(1e-6, obstacle.approach_velocity);
    if (time_to_traj > p.max_lat_time_margin_for_cruise) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_,
        "[Cruise] Ignore obstacle (%s) since it's far from trajectory.", object_id.c_str());
      return std::nullopt;
    }
  }

  if (isObstacleCrossing(traj_points, obstacle)) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore obstacle (%s) since it's crossing the ego's trajectory..",
      object_id.c_str());
    return std::nullopt;
  }

  const auto collision_points = [&]() -> std::optional<std::vector<PointWithStamp>> {
    constexpr double epsilon = 1e-6;
    if (precise_lat_dist < epsilon) {
      // obstacle is inside the trajectory
      return createCollisionPointsForInsideCruiseObstacle(traj_points, traj_polys, obstacle);
    }
    // obstacle is outside the trajectory
    // If the ego is stopping, do not plan cruise for outside obstacles. Stop will be planned.
    if (odometry.twist.twist.linear.x < 0.1) {
      return std::nullopt;
    }
    return createCollisionPointsForOutsideCruiseObstacle(traj_points, traj_polys, obstacle);
  }();
  if (!collision_points) {
    return std::nullopt;
  }

  return CruiseObstacle{
    obstacle.uuid,
    obstacle.stamp,
    obstacle.pose,
    obstacle.longitudinal_velocity,
    obstacle.approach_velocity,
    *collision_points};
}

std::optional<CruiseObstacle> ObstacleCruisePlannerNode::createYieldCruiseObstacle(
  const Obstacle & obstacle, const std::vector<TrajectoryPoint> & traj_points)
{
  if (traj_points.empty()) return std::nullopt;
  // check label
  const auto & object_id = obstacle.uuid.substr(0, 4);
  const auto & p = behavior_determination_param_;

  if (!isOutsideCruiseObstacle(obstacle.classification.label)) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore yield obstacle (%s) since its type is not designated.", object_id.c_str());
    return std::nullopt;
  }

  if (
    std::hypot(obstacle.twist.linear.x, obstacle.twist.linear.y) <
    p.outside_obstacle_min_velocity_threshold) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore yield obstacle (%s) since the obstacle velocity is low.", object_id.c_str());
    return std::nullopt;
  }

  if (isObstacleCrossing(traj_points, obstacle)) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore yield obstacle (%s) since it's crossing the ego's trajectory..",
      object_id.c_str());
    return std::nullopt;
  }

  const auto collision_points = [&]() -> std::optional<std::vector<PointWithStamp>> {
    const auto obstacle_idx = autoware::motion_utils::findNearestIndex(traj_points, obstacle.pose);
    if (!obstacle_idx) return std::nullopt;
    const auto collision_traj_point = traj_points.at(obstacle_idx.value());
    const auto object_time = now() + traj_points.at(obstacle_idx.value()).time_from_start;

    PointWithStamp collision_traj_point_with_stamp;
    collision_traj_point_with_stamp.stamp = object_time;
    collision_traj_point_with_stamp.point.x = collision_traj_point.pose.position.x;
    collision_traj_point_with_stamp.point.y = collision_traj_point.pose.position.y;
    collision_traj_point_with_stamp.point.z = collision_traj_point.pose.position.z;
    std::vector<PointWithStamp> collision_points_vector{collision_traj_point_with_stamp};
    return collision_points_vector;
  }();

  if (!collision_points) return std::nullopt;
  // check if obstacle is driving on the opposite direction
  if (obstacle.longitudinal_velocity < 0.0) return std::nullopt;
  return CruiseObstacle{
    obstacle.uuid,
    obstacle.stamp,
    obstacle.pose,
    obstacle.longitudinal_velocity,
    obstacle.approach_velocity,
    collision_points.value(),
    true};
}

std::optional<std::vector<CruiseObstacle>> ObstacleCruisePlannerNode::findYieldCruiseObstacles(
  const std::vector<Obstacle> & obstacles, const std::vector<TrajectoryPoint> & traj_points)
{
  if (obstacles.empty() || traj_points.empty()) return std::nullopt;
  const auto & p = behavior_determination_param_;

  std::vector<Obstacle> stopped_obstacles;
  std::vector<Obstacle> moving_obstacles;

  std::for_each(
    obstacles.begin(), obstacles.end(),
    [&stopped_obstacles, &moving_obstacles, &p](const auto & o) {
      const bool is_moving =
        std::hypot(o.twist.linear.x, o.twist.linear.y) > p.stopped_obstacle_velocity_threshold;
      if (is_moving) {
        const bool is_within_lat_dist_threshold =
          o.lat_dist_from_obstacle_to_traj < p.yield_lat_distance_threshold;
        if (is_within_lat_dist_threshold) moving_obstacles.push_back(o);
        return;
      }
      // lat threshold is larger for stopped obstacles
      const bool is_within_lat_dist_threshold =
        o.lat_dist_from_obstacle_to_traj <
        p.yield_lat_distance_threshold + p.max_lat_dist_between_obstacles;
      if (is_within_lat_dist_threshold) stopped_obstacles.push_back(o);
      return;
    });

  if (stopped_obstacles.empty() || moving_obstacles.empty()) return std::nullopt;

  std::sort(
    stopped_obstacles.begin(), stopped_obstacles.end(), [](const auto & o1, const auto & o2) {
      return o1.ego_to_obstacle_distance < o2.ego_to_obstacle_distance;
    });

  std::sort(moving_obstacles.begin(), moving_obstacles.end(), [](const auto & o1, const auto & o2) {
    return o1.ego_to_obstacle_distance < o2.ego_to_obstacle_distance;
  });

  std::vector<CruiseObstacle> yield_obstacles;
  for (const auto & moving_obstacle : moving_obstacles) {
    for (const auto & stopped_obstacle : stopped_obstacles) {
      const bool is_moving_obs_behind_of_stopped_obs =
        moving_obstacle.ego_to_obstacle_distance < stopped_obstacle.ego_to_obstacle_distance;
      const bool is_moving_obs_ahead_of_ego_front =
        moving_obstacle.ego_to_obstacle_distance > vehicle_info_.vehicle_length_m;

      if (!is_moving_obs_ahead_of_ego_front || !is_moving_obs_behind_of_stopped_obs) continue;

      const double lateral_distance_between_obstacles = std::abs(
        moving_obstacle.lat_dist_from_obstacle_to_traj -
        stopped_obstacle.lat_dist_from_obstacle_to_traj);

      const double longitudinal_distance_between_obstacles = std::abs(
        moving_obstacle.ego_to_obstacle_distance - stopped_obstacle.ego_to_obstacle_distance);

      const double moving_obstacle_speed =
        std::hypot(moving_obstacle.twist.linear.x, moving_obstacle.twist.linear.y);

      const bool are_obstacles_aligned =
        lateral_distance_between_obstacles < p.max_lat_dist_between_obstacles;
      const bool obstacles_collide_within_threshold_time =
        longitudinal_distance_between_obstacles / moving_obstacle_speed <
        p.max_obstacles_collision_time;
      if (are_obstacles_aligned && obstacles_collide_within_threshold_time) {
        const auto yield_obstacle = createYieldCruiseObstacle(moving_obstacle, traj_points);
        if (yield_obstacle) {
          yield_obstacles.push_back(*yield_obstacle);
          using autoware::objects_of_interest_marker_interface::ColorName;
          objects_of_interest_marker_interface_.insertObjectData(
            stopped_obstacle.pose, stopped_obstacle.shape, ColorName::RED);
        }
      }
    }
  }
  if (yield_obstacles.empty()) return std::nullopt;
  return yield_obstacles;
}

std::optional<std::vector<PointWithStamp>>
ObstacleCruisePlannerNode::createCollisionPointsForInsideCruiseObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle) const
{
  const auto & object_id = obstacle.uuid.substr(0, 4);
  const auto & p = behavior_determination_param_;

  // check label
  if (!isInsideCruiseObstacle(obstacle.classification.label)) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore inside obstacle (%s) since its type is not designated.", object_id.c_str());
    return std::nullopt;
  }

  {  // consider hysteresis
    // const bool is_prev_obstacle_stop = getObstacleFromUuid(prev_stop_obstacles_,
    // obstacle.uuid).has_value();
    const bool is_prev_obstacle_cruise =
      getObstacleFromUuid(prev_cruise_object_obstacles_, obstacle.uuid).has_value();

    if (is_prev_obstacle_cruise) {
      if (obstacle.longitudinal_velocity < p.obstacle_velocity_threshold_from_cruise_to_stop) {
        return std::nullopt;
      }
      // NOTE: else is keeping cruise
    } else {  // if (is_prev_obstacle_stop) {
      // TODO(murooka) consider hysteresis for slow down
      // If previous obstacle is stop or does not exist.
      if (obstacle.longitudinal_velocity < p.obstacle_velocity_threshold_from_stop_to_cruise) {
        return std::nullopt;
      }
      // NOTE: else is cruise from stop
    }
  }

  // Get highest confidence predicted path
  const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths(
    obstacle.predicted_paths, p.prediction_resampling_time_interval,
    p.prediction_resampling_time_horizon, 1);

  if (resampled_predicted_paths.empty()) {
    return std::nullopt;
  }

  // calculate nearest collision point
  std::vector<size_t> collision_index;
  const auto collision_points = polygon_utils::getCollisionPoints(
    traj_points, traj_polys, obstacle.stamp, resampled_predicted_paths.front(), obstacle.shape,
    now(), is_driving_forward_, collision_index,
    calcObstacleMaxLength(obstacle.shape) + p.decimate_trajectory_step_length +
      std::hypot(
        vehicle_info_.vehicle_length_m,
        vehicle_info_.vehicle_width_m * 0.5 + p.max_lat_margin_for_cruise));
  return collision_points;
}

std::optional<std::vector<PointWithStamp>>
ObstacleCruisePlannerNode::createCollisionPointsForOutsideCruiseObstacle(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
  const Obstacle & obstacle) const
{
  const auto & p = behavior_determination_param_;
  const auto & object_id = obstacle.uuid.substr(0, 4);

  // check label
  if (!isOutsideCruiseObstacle(obstacle.classification.label)) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore outside obstacle (%s) since its type is not designated.", object_id.c_str());
    return std::nullopt;
  }

  if (
    std::hypot(obstacle.twist.linear.x, obstacle.twist.linear.y) <
    p.outside_obstacle_min_velocity_threshold) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore outside obstacle (%s) since the obstacle velocity is low.",
      object_id.c_str());
    return std::nullopt;
  }

  // Get the highest confidence predicted paths
  const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths(
    obstacle.predicted_paths, p.prediction_resampling_time_interval,
    p.prediction_resampling_time_horizon, p.num_of_predicted_paths_for_outside_cruise_obstacle);

  // calculate collision condition for cruise
  std::vector<size_t> collision_index;
  const auto getCollisionPoints = [&]() -> std::vector<PointWithStamp> {
    for (const auto & predicted_path : resampled_predicted_paths) {
      const auto collision_points = polygon_utils::getCollisionPoints(
        traj_points, traj_polys, obstacle.stamp, predicted_path, obstacle.shape, now(),
        is_driving_forward_, collision_index,
        calcObstacleMaxLength(obstacle.shape) + p.decimate_trajectory_step_length +
          std::hypot(
            vehicle_info_.vehicle_length_m,
            vehicle_info_.vehicle_width_m * 0.5 + p.max_lat_margin_for_cruise),
        p.max_prediction_time_for_collision_check);
      if (!collision_points.empty()) {
        return collision_points;
      }
    }
    return {};
  };

  const auto collision_points = getCollisionPoints();

  if (collision_points.empty()) {
    // Ignore vehicle obstacles outside the trajectory without collision
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore outside obstacle (%s) since there are no collision points.",
      object_id.c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
    return std::nullopt;
  }

  const double overlap_time =
    (rclcpp::Time(collision_points.back().stamp) - rclcpp::Time(collision_points.front().stamp))
      .seconds();
  if (overlap_time < p.ego_obstacle_overlap_time_threshold) {
    // Ignore vehicle obstacles outside the trajectory, whose predicted path
    // overlaps the ego trajectory in a certain time.
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Cruise] Ignore outside obstacle (%s) since it will not collide with the ego.",
      object_id.c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
    return std::nullopt;
  }

  // Ignore obstacles behind the ego vehicle.
  // Note: Only using isFrontObstacle(), behind obstacles cannot be filtered
  // properly when the trajectory is crossing or overlapping.
  const size_t first_collision_index = collision_index.front();
  if (!isFrontCollideObstacle(traj_points, obstacle, first_collision_index)) {
    return std::nullopt;
  }
  return collision_points;
}

std::optional<std::pair<geometry_msgs::msg::Point, double>>
ObstacleCruisePlannerNode::createCollisionPointForOutsideStopObstacle(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<Polygon2d> & traj_polys, const Obstacle & obstacle,
  const PredictedPath & resampled_predicted_path, double max_lat_margin_for_stop) const
{
  const auto & object_id = obstacle.uuid.substr(0, 4);
  const auto & p = behavior_determination_param_;

  std::vector<size_t> collision_index;
  const auto collision_points = polygon_utils::getCollisionPoints(
    traj_points, traj_polys, obstacle.stamp, resampled_predicted_path, obstacle.shape, now(),
    is_driving_forward_, collision_index,
    calcObstacleMaxLength(obstacle.shape) + p.decimate_trajectory_step_length +
      std::hypot(
        vehicle_info_.vehicle_length_m,
        vehicle_info_.vehicle_width_m * 0.5 + max_lat_margin_for_stop));
  if (collision_points.empty()) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Stop] Ignore outside obstacle (%s) since there is no collision point between the "
      "predicted path "
      "and the ego.",
      object_id.c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
    return std::nullopt;
  }

  const double collision_time_margin =
    calcCollisionTimeMargin(odometry, collision_points, traj_points, is_driving_forward_);
  if (p.collision_time_margin < collision_time_margin) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Stop] Ignore outside obstacle (%s) since it will not collide with the ego.",
      object_id.c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
    return std::nullopt;
  }

  return polygon_utils::getCollisionPoint(
    traj_points, collision_index.front(), collision_points, is_driving_forward_, vehicle_info_);
}

std::optional<StopObstacle> ObstacleCruisePlannerNode::createStopObstacleForPredictedObject(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<Polygon2d> & traj_polys, const Obstacle & obstacle,
  const double precise_lat_dist) const
{
  const auto & p = behavior_determination_param_;
  const auto & object_id = obstacle.uuid.substr(0, 4);

  if (!isStopObstacle(obstacle.classification.label)) {
    return std::nullopt;
  }
  const double max_lat_margin_for_stop =
    (obstacle.classification.label == ObjectClassification::UNKNOWN)
      ? p.max_lat_margin_for_stop_against_unknown
      : p.max_lat_margin_for_stop;

  // Obstacle that is not inside of trajectory
  if (precise_lat_dist > std::max(max_lat_margin_for_stop, 1e-3)) {
    if (!isOutsideStopObstacle(obstacle.classification.label)) {
      return std::nullopt;
    }

    const auto time_to_traj = precise_lat_dist / std::max(1e-6, obstacle.approach_velocity);
    if (time_to_traj > p.max_lat_time_margin_for_stop) {
      RCLCPP_INFO_EXPRESSION(
        get_logger(), enable_debug_info_,
        "[Stop] Ignore outside obstacle (%s) since it's far from trajectory.", object_id.c_str());
      return std::nullopt;
    }

    // brkay54: For the pedestrians and bicycles, we need to check the collision point by thinking
    // they will stop with a predefined deceleration rate to avoid unnecessary stops.
    double resample_time_horizon = p.prediction_resampling_time_horizon;
    if (obstacle.classification.label == ObjectClassification::PEDESTRIAN) {
      resample_time_horizon =
        std::sqrt(std::pow(obstacle.twist.linear.x, 2) + std::pow(obstacle.twist.linear.y, 2)) /
        (2.0 * p.pedestrian_deceleration_rate);
    } else if (obstacle.classification.label == ObjectClassification::BICYCLE) {
      resample_time_horizon =
        std::sqrt(std::pow(obstacle.twist.linear.x, 2) + std::pow(obstacle.twist.linear.y, 2)) /
        (2.0 * p.bicycle_deceleration_rate);
    }

    // Get the highest confidence predicted path
    const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths(
      obstacle.predicted_paths, p.prediction_resampling_time_interval, resample_time_horizon,
      p.num_of_predicted_paths_for_outside_stop_obstacle);
    if (resampled_predicted_paths.empty()) {
      return std::nullopt;
    }

    const auto getCollisionPoint =
      [&]() -> std::optional<std::pair<geometry_msgs::msg::Point, double>> {
      for (const auto & predicted_path : resampled_predicted_paths) {
        const auto collision_point = createCollisionPointForOutsideStopObstacle(
          odometry, traj_points, traj_polys, obstacle, predicted_path, max_lat_margin_for_stop);
        if (collision_point) {
          return collision_point;
        }
      }
      return std::nullopt;
    };

    const auto collision_point = getCollisionPoint();

    if (collision_point) {
      return StopObstacle{
        obstacle.uuid,
        obstacle.stamp,
        obstacle.classification,
        obstacle.pose,
        obstacle.shape,
        obstacle.longitudinal_velocity,
        obstacle.approach_velocity,
        collision_point->first,
        collision_point->second};
    }
    return std::nullopt;
  }

  // Obstacle inside the trajectory
  if (!isInsideStopObstacle(obstacle.classification.label)) {
    return std::nullopt;
  }

  // calculate collision points with trajectory with lateral stop margin
  const auto traj_polys_with_lat_margin =
    createOneStepPolygons(traj_points, vehicle_info_, odometry.pose.pose, max_lat_margin_for_stop);

  const auto collision_point = polygon_utils::getCollisionPoint(
    traj_points, traj_polys_with_lat_margin, obstacle, is_driving_forward_, vehicle_info_);
  if (!collision_point) {
    return std::nullopt;
  }

  // check transient obstacle or not
  const double abs_ego_offset =
    min_behavior_stop_margin_ + (is_driving_forward_
                                   ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                   : std::abs(vehicle_info_.min_longitudinal_offset_m));

  const double time_to_reach_stop_point =
    calcTimeToReachCollisionPoint(odometry, collision_point->first, traj_points, abs_ego_offset);
  const bool is_transient_obstacle = [&]() {
    if (time_to_reach_stop_point <= p.collision_time_margin) {
      return false;
    }
    // get the predicted position of the obstacle when ego reaches the collision point
    const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths(
      obstacle.predicted_paths, p.prediction_resampling_time_interval,
      p.prediction_resampling_time_horizon, 1);
    if (resampled_predicted_paths.empty() || resampled_predicted_paths.front().path.empty()) {
      return false;
    }
    const auto future_obj_pose = autoware::object_recognition_utils::calcInterpolatedPose(
      resampled_predicted_paths.front(), time_to_reach_stop_point - p.collision_time_margin);

    Obstacle tmp_future_obs = obstacle;
    tmp_future_obs.pose =
      future_obj_pose ? future_obj_pose.value() : resampled_predicted_paths.front().path.back();
    const auto future_collision_point = polygon_utils::getCollisionPoint(
      traj_points, traj_polys_with_lat_margin, tmp_future_obs, is_driving_forward_, vehicle_info_);

    return !future_collision_point;
  }();

  if (is_transient_obstacle) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[Stop] Ignore inside obstacle (%s) since it is transient obstacle.", object_id.c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
    return std::nullopt;
  }

  return StopObstacle{
    obstacle.uuid,
    obstacle.stamp,
    obstacle.classification,
    obstacle.pose,
    obstacle.shape,
    obstacle.longitudinal_velocity,
    obstacle.approach_velocity,
    collision_point->first,
    collision_point->second};
}

std::optional<StopObstacle> ObstacleCruisePlannerNode::createStopObstacleForPointCloud(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
  const double precise_lat_dist) const
{
  const auto & p = behavior_determination_param_;

  if (!use_pointcloud_for_stop_) {
    return std::nullopt;
  }

  if (!obstacle.stop_collision_point) {
    return std::nullopt;
  }

  const double max_lat_margin_for_stop =
    (obstacle.classification.label == ObjectClassification::UNKNOWN)
      ? p.max_lat_margin_for_stop_against_unknown
      : p.max_lat_margin_for_stop;

  if (precise_lat_dist > std::max(max_lat_margin_for_stop, 1e-3)) {
    return std::nullopt;
  }

  const auto dist_to_collide_on_traj =
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, *obstacle.stop_collision_point);

  return StopObstacle{
    obstacle.uuid,
    obstacle.stamp,
    obstacle.classification,
    obstacle.pose,
    obstacle.shape,
    {},
    {},
    *obstacle.stop_collision_point,
    dist_to_collide_on_traj};
}

std::optional<SlowDownObstacle> ObstacleCruisePlannerNode::createSlowDownObstacleForPredictedObject(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const Obstacle & obstacle, const double precise_lat_dist)
{
  const auto & object_id = obstacle.uuid.substr(0, 4);
  const auto & p = behavior_determination_param_;
  slow_down_condition_counter_.addCurrentUuid(obstacle.uuid);

  const bool is_prev_obstacle_slow_down =
    getObstacleFromUuid(prev_slow_down_object_obstacles_, obstacle.uuid).has_value();

  if (!enable_slow_down_planning_ || !isSlowDownObstacle(obstacle.classification.label)) {
    return std::nullopt;
  }

  // check lateral distance considering hysteresis
  const bool is_lat_dist_low = isLowerConsideringHysteresis(
    precise_lat_dist, is_prev_obstacle_slow_down,
    p.max_lat_margin_for_slow_down + p.lat_hysteresis_margin_for_slow_down / 2.0,
    p.max_lat_margin_for_slow_down - p.lat_hysteresis_margin_for_slow_down / 2.0);

  const bool is_slow_down_required = [&]() {
    if (is_prev_obstacle_slow_down) {
      // check if exiting slow down
      if (!is_lat_dist_low) {
        const int count = slow_down_condition_counter_.decreaseCounter(obstacle.uuid);
        if (count <= -p.successive_num_to_exit_slow_down_condition) {
          slow_down_condition_counter_.reset(obstacle.uuid);
          return false;
        }
      }
      return true;
    }
    // check if entering slow down
    if (is_lat_dist_low) {
      const int count = slow_down_condition_counter_.increaseCounter(obstacle.uuid);
      if (p.successive_num_to_entry_slow_down_condition <= count) {
        slow_down_condition_counter_.reset(obstacle.uuid);
        return true;
      }
    }
    return false;
  }();
  if (!is_slow_down_required) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[SlowDown] Ignore obstacle (%s) since it's far from trajectory. (%f [m])", object_id.c_str(),
      precise_lat_dist);
    return std::nullopt;
  }

  const auto obstacle_poly = autoware::universe_utils::toPolygon2d(obstacle.pose, obstacle.shape);

  // calculate collision points with trajectory with lateral stop margin
  // NOTE: For additional margin, hysteresis is not divided by two.
  const auto traj_polys_with_lat_margin = createOneStepPolygons(
    traj_points, vehicle_info_, odometry.pose.pose,
    p.max_lat_margin_for_slow_down + p.lat_hysteresis_margin_for_slow_down);

  std::vector<Polygon2d> front_collision_polygons;
  size_t front_seg_idx = 0;
  std::vector<Polygon2d> back_collision_polygons;
  size_t back_seg_idx = 0;
  for (size_t i = 0; i < traj_polys_with_lat_margin.size(); ++i) {
    std::vector<Polygon2d> collision_polygons;
    bg::intersection(traj_polys_with_lat_margin.at(i), obstacle_poly, collision_polygons);

    if (!collision_polygons.empty()) {
      if (front_collision_polygons.empty()) {
        front_collision_polygons = collision_polygons;
        front_seg_idx = i == 0 ? i : i - 1;
      }
      back_collision_polygons = collision_polygons;
      back_seg_idx = i == 0 ? i : i - 1;
    } else {
      if (!back_collision_polygons.empty()) {
        break;  // for efficient calculation
      }
    }
  }

  if (front_collision_polygons.empty() || back_collision_polygons.empty()) {
    RCLCPP_INFO_EXPRESSION(
      get_logger(), enable_debug_info_,
      "[SlowDown] Ignore obstacle (%s) since there is no collision point", object_id.c_str());
    return std::nullopt;
  }

  // calculate front collision point
  double front_min_dist = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point front_collision_point;
  for (const auto & collision_poly : front_collision_polygons) {
    for (const auto & collision_point : collision_poly.outer()) {
      const auto collision_geom_point = toGeomPoint(collision_point);
      const double dist = autoware::motion_utils::calcLongitudinalOffsetToSegment(
        traj_points, front_seg_idx, collision_geom_point);
      if (dist < front_min_dist) {
        front_min_dist = dist;
        front_collision_point = collision_geom_point;
      }
    }
  }

  // calculate back collision point
  double back_max_dist = -std::numeric_limits<double>::max();
  geometry_msgs::msg::Point back_collision_point = front_collision_point;
  for (const auto & collision_poly : back_collision_polygons) {
    for (const auto & collision_point : collision_poly.outer()) {
      const auto collision_geom_point = toGeomPoint(collision_point);
      const double dist = autoware::motion_utils::calcLongitudinalOffsetToSegment(
        traj_points, back_seg_idx, collision_geom_point);
      if (back_max_dist < dist) {
        back_max_dist = dist;
        back_collision_point = collision_geom_point;
      }
    }
  }

  return SlowDownObstacle{
    obstacle.uuid,
    obstacle.stamp,
    obstacle.classification,
    obstacle.pose,
    obstacle.longitudinal_velocity,
    obstacle.approach_velocity,
    precise_lat_dist,
    front_collision_point,
    back_collision_point};
}

std::optional<SlowDownObstacle> ObstacleCruisePlannerNode::createSlowDownObstacleForPointCloud(
  const Obstacle & obstacle, const double precise_lat_dist)
{
  if (!enable_slow_down_planning_ || !use_pointcloud_for_slow_down_) {
    return std::nullopt;
  }

  if (!obstacle.slow_down_front_collision_point) {
    return std::nullopt;
  }

  auto front_collision_point = *obstacle.slow_down_front_collision_point;
  auto back_collision_point =
    obstacle.slow_down_back_collision_point.value_or(front_collision_point);

  return SlowDownObstacle{
    obstacle.uuid,    obstacle.stamp,        obstacle.classification, obstacle.pose, {}, {},
    precise_lat_dist, front_collision_point, back_collision_point};
}

void ObstacleCruisePlannerNode::checkConsistency(
  const rclcpp::Time & current_time, const PredictedObjects & predicted_objects,
  std::vector<StopObstacle> & stop_obstacles)
{
  for (const auto & prev_closest_stop_obstacle : prev_closest_stop_object_obstacles_) {
    const auto predicted_object_itr = std::find_if(
      predicted_objects.objects.begin(), predicted_objects.objects.end(),
      [&prev_closest_stop_obstacle](const PredictedObject & po) {
        return autoware::universe_utils::toHexString(po.object_id) ==
               prev_closest_stop_obstacle.uuid;
      });
    // If previous closest obstacle disappear from the perception result, do nothing anymore.
    if (predicted_object_itr == predicted_objects.objects.end()) {
      continue;
    }

    const auto is_disappeared_from_stop_obstacle = std::none_of(
      stop_obstacles.begin(), stop_obstacles.end(),
      [&prev_closest_stop_obstacle](const StopObstacle & so) {
        return so.uuid == prev_closest_stop_obstacle.uuid;
      });
    if (is_disappeared_from_stop_obstacle) {
      // re-evaluate as a stop candidate, and overwrite the current decision if "maintain stop"
      // condition is satisfied
      const double elapsed_time = (current_time - prev_closest_stop_obstacle.stamp).seconds();
      if (
        predicted_object_itr->kinematics.initial_twist_with_covariance.twist.linear.x <
          behavior_determination_param_.obstacle_velocity_threshold_from_stop_to_cruise &&
        elapsed_time < behavior_determination_param_.stop_obstacle_hold_time_threshold) {
        stop_obstacles.push_back(prev_closest_stop_obstacle);
      }
    }
  }

  prev_closest_stop_object_obstacles_ =
    obstacle_cruise_utils::getClosestStopObstacles(stop_obstacles);
}

bool ObstacleCruisePlannerNode::isObstacleCrossing(
  const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle) const
{
  const double diff_angle = calcDiffAngleAgainstTrajectory(traj_points, obstacle.pose);

  // NOTE: Currently predicted objects does not have orientation availability even
  // though sometimes orientation is not available.
  const bool is_obstacle_crossing_trajectory =
    behavior_determination_param_.crossing_obstacle_traj_angle_threshold < std::abs(diff_angle) &&
    behavior_determination_param_.crossing_obstacle_traj_angle_threshold <
      M_PI - std::abs(diff_angle);
  if (!is_obstacle_crossing_trajectory) {
    return false;
  }

  // Only obstacles crossing the ego's trajectory with high speed are considered.
  return true;
}

double ObstacleCruisePlannerNode::calcCollisionTimeMargin(
  const Odometry & odometry, const std::vector<PointWithStamp> & collision_points,
  const std::vector<TrajectoryPoint> & traj_points, const bool is_driving_forward) const
{
  const auto & p = behavior_determination_param_;
  const double abs_ego_offset =
    min_behavior_stop_margin_ + (is_driving_forward
                                   ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                   : std::abs(vehicle_info_.min_longitudinal_offset_m));
  const double time_to_reach_stop_point = calcTimeToReachCollisionPoint(
    odometry, collision_points.front().point, traj_points, abs_ego_offset);

  const double time_to_leave_collision_point =
    time_to_reach_stop_point +
    abs_ego_offset /
      std::max(p.min_velocity_to_reach_collision_point, odometry.twist.twist.linear.x);

  const double time_to_start_cross =
    (rclcpp::Time(collision_points.front().stamp) - now()).seconds();
  const double time_to_end_cross = (rclcpp::Time(collision_points.back().stamp) - now()).seconds();

  if (time_to_leave_collision_point < time_to_start_cross) {  // Ego goes first.
    return time_to_start_cross - time_to_reach_stop_point;
  }
  if (time_to_end_cross < time_to_reach_stop_point) {  // Obstacle goes first.
    return time_to_reach_stop_point - time_to_end_cross;
  }
  return 0.0;  // Ego and obstacle will collide.
}

PlannerData ObstacleCruisePlannerNode::createPlannerData(
  const Odometry & odometry, const AccelWithCovarianceStamped & acc,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  PlannerData planner_data;
  planner_data.current_time = now();
  planner_data.traj_points = traj_points;
  planner_data.ego_pose = odometry.pose.pose;
  planner_data.ego_vel = odometry.twist.twist.linear.x;
  planner_data.ego_acc = acc.accel.accel.linear.x;
  planner_data.is_driving_forward = is_driving_forward_;
  return planner_data;
}

void ObstacleCruisePlannerNode::publishVelocityLimit(
  const std::optional<VelocityLimit> & vel_limit, const std::string & module_name)
{
  if (vel_limit) {
    vel_limit_pub_->publish(*vel_limit);
    need_to_clear_vel_limit_.at(module_name) = true;
    return;
  }

  if (!need_to_clear_vel_limit_.at(module_name)) {
    return;
  }

  // clear velocity limit
  const auto clear_vel_limit_msg = createVelocityLimitClearCommandMessage(now(), module_name);
  clear_vel_limit_pub_->publish(clear_vel_limit_msg);
  need_to_clear_vel_limit_.at(module_name) = false;
}

void ObstacleCruisePlannerNode::publishDebugMarker() const
{
  stop_watch_.tic(__func__);

  // 1. publish debug marker
  MarkerArray debug_marker;

  // obstacles to cruise
  std::vector<geometry_msgs::msg::Point> stop_collision_points;
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_cruise.size(); ++i) {
    // obstacle
    const auto obstacle_marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->obstacles_to_cruise.at(i).pose, i, "obstacles_to_cruise", 1.0, 0.6, 0.1);
    debug_marker.markers.push_back(obstacle_marker);

    // collision points
    for (size_t j = 0; j < debug_data_ptr_->obstacles_to_cruise.at(i).collision_points.size();
         ++j) {
      stop_collision_points.push_back(
        debug_data_ptr_->obstacles_to_cruise.at(i).collision_points.at(j).point);
    }
  }
  for (size_t i = 0; i < stop_collision_points.size(); ++i) {
    auto collision_point_marker = autoware::universe_utils::createDefaultMarker(
      "map", now(), "cruise_collision_points", i, Marker::SPHERE,
      autoware::universe_utils::createMarkerScale(0.25, 0.25, 0.25),
      autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
    collision_point_marker.pose.position = stop_collision_points.at(i);
    debug_marker.markers.push_back(collision_point_marker);
  }

  // obstacles to stop
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_stop.size(); ++i) {
    // obstacle
    const auto obstacle_marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->obstacles_to_stop.at(i).pose, i, "obstacles_to_stop", 1.0, 0.0, 0.0);
    debug_marker.markers.push_back(obstacle_marker);

    // collision point
    auto collision_point_marker = autoware::universe_utils::createDefaultMarker(
      "map", now(), "stop_collision_points", 0, Marker::SPHERE,
      autoware::universe_utils::createMarkerScale(0.25, 0.25, 0.25),
      autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
    collision_point_marker.pose.position = debug_data_ptr_->obstacles_to_stop.at(i).collision_point;
    debug_marker.markers.push_back(collision_point_marker);
  }

  // obstacles to slow down
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_slow_down.size(); ++i) {
    // obstacle
    const auto obstacle_marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->obstacles_to_slow_down.at(i).pose, i, "obstacles_to_slow_down", 0.7, 0.7,
      0.0);
    debug_marker.markers.push_back(obstacle_marker);

    // collision points
    auto front_collision_point_marker = autoware::universe_utils::createDefaultMarker(
      "map", now(), "slow_down_collision_points", i * 2 + 0, Marker::SPHERE,
      autoware::universe_utils::createMarkerScale(0.25, 0.25, 0.25),
      autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
    front_collision_point_marker.pose.position =
      debug_data_ptr_->obstacles_to_slow_down.at(i).front_collision_point;
    auto back_collision_point_marker = autoware::universe_utils::createDefaultMarker(
      "map", now(), "slow_down_collision_points", i * 2 + 1, Marker::SPHERE,
      autoware::universe_utils::createMarkerScale(0.25, 0.25, 0.25),
      autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
    back_collision_point_marker.pose.position =
      debug_data_ptr_->obstacles_to_slow_down.at(i).back_collision_point;

    debug_marker.markers.push_back(front_collision_point_marker);
    debug_marker.markers.push_back(back_collision_point_marker);
  }

  // intentionally ignored obstacles to cruise or stop
  for (size_t i = 0; i < debug_data_ptr_->intentionally_ignored_obstacles.size(); ++i) {
    const auto marker = obstacle_cruise_utils::getObjectMarker(
      debug_data_ptr_->intentionally_ignored_obstacles.at(i).pose, i,
      "intentionally_ignored_obstacles", 0.0, 1.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  {  // footprint polygons
    auto marker = autoware::universe_utils::createDefaultMarker(
      "map", now(), "detection_polygons", 0, Marker::LINE_LIST,
      autoware::universe_utils::createMarkerScale(0.01, 0.0, 0.0),
      autoware::universe_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (const auto & detection_polygon : debug_data_ptr_->detection_polygons) {
      for (size_t dp_idx = 0; dp_idx < detection_polygon.outer().size(); ++dp_idx) {
        const auto & current_point = detection_polygon.outer().at(dp_idx);
        const auto & next_point =
          detection_polygon.outer().at((dp_idx + 1) % detection_polygon.outer().size());

        marker.points.push_back(
          autoware::universe_utils::createPoint(current_point.x(), current_point.y(), 0.0));
        marker.points.push_back(
          autoware::universe_utils::createPoint(next_point.x(), next_point.y(), 0.0));
      }
    }
    debug_marker.markers.push_back(marker);
  }

  // slow down debug wall marker
  autoware::universe_utils::appendMarkerArray(
    debug_data_ptr_->slow_down_debug_wall_marker, &debug_marker);

  debug_marker_pub_->publish(debug_marker);

  // 2. publish virtual wall for cruise and stop
  debug_cruise_wall_marker_pub_->publish(debug_data_ptr_->cruise_wall_marker);
  debug_stop_wall_marker_pub_->publish(debug_data_ptr_->stop_wall_marker);
  debug_slow_down_wall_marker_pub_->publish(debug_data_ptr_->slow_down_wall_marker);

  // 3. print calculation time
  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner"), enable_calculation_time_info_, "  %s := %f [ms]",
    __func__, calculation_time);
}

void ObstacleCruisePlannerNode::publishDebugInfo() const
{
  // stop
  const auto stop_debug_msg = planner_ptr_->getStopPlanningDebugMessage(now());
  debug_stop_planning_info_pub_->publish(stop_debug_msg);

  // cruise
  const auto cruise_debug_msg = planner_ptr_->getCruisePlanningDebugMessage(now());
  debug_cruise_planning_info_pub_->publish(cruise_debug_msg);

  // slow_down
  const auto slow_down_debug_msg = planner_ptr_->getSlowDownPlanningDebugMessage(now());
  debug_slow_down_planning_info_pub_->publish(slow_down_debug_msg);
}

void ObstacleCruisePlannerNode::publishCalculationTime(const double calculation_time) const
{
  Float64Stamped calculation_time_msg;
  calculation_time_msg.stamp = now();
  calculation_time_msg.data = calculation_time;
  debug_calculation_time_pub_->publish(calculation_time_msg);
}

double ObstacleCruisePlannerNode::calcTimeToReachCollisionPoint(
  const Odometry & odometry, const geometry_msgs::msg::Point & collision_point,
  const std::vector<TrajectoryPoint> & traj_points, const double abs_ego_offset) const
{
  const auto & p = behavior_determination_param_;
  const double dist_from_ego_to_obstacle =
    std::abs(autoware::motion_utils::calcSignedArcLength(
      traj_points, odometry.pose.pose.position, collision_point)) -
    abs_ego_offset;
  return dist_from_ego_to_obstacle /
         std::max(p.min_velocity_to_reach_collision_point, std::abs(odometry.twist.twist.linear.x));
}
}  // namespace autoware::motion_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion_planning::ObstacleCruisePlannerNode)
