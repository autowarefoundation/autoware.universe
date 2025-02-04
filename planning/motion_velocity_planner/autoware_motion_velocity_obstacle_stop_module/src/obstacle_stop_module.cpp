// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "obstacle_stop_module.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware::universe_utils::getOrDeclareParameter;

namespace
{
template <typename T>
bool is_in_vector(const T variable, const std::vector<T> & vec)
{
  return std::find(vec.begin(), vec.end(), variable) != vec.end();
}

double calc_minimum_distance_to_stop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }

  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

autoware::universe_utils::Point2d convert_point(const geometry_msgs::msg::Point & p)
{
  return autoware::universe_utils::Point2d{p.x, p.y};
}

std::vector<TrajectoryPoint> resample_trajectory_points(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(traj, interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

std::vector<PredictedPath> resample_highest_confidence_predicted_paths(
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

template <class T>
std::vector<T> concat_vectors(
  const std::vector<T> & first_vector, const std::vector<T> & second_vector)
{
  std::vector<T> concatenated_vector;
  concatenated_vector.insert(concatenated_vector.end(), first_vector.begin(), first_vector.end());
  concatenated_vector.insert(concatenated_vector.end(), second_vector.begin(), second_vector.end());
  return concatenated_vector;
}

double calc_dist_to_bumper(const bool is_driving_forward, const VehicleInfo & vehicle_info)
{
  if (is_driving_forward) {
    return std::abs(vehicle_info.max_longitudinal_offset_m);
  }
  return std::abs(vehicle_info.min_longitudinal_offset_m);
}

Float64Stamped create_float64_stamped(const rclcpp::Time & now, const float & data)
{
  Float64Stamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}

double calc_time_to_reach_collision_point(
  const Odometry & odometry, const geometry_msgs::msg::Point & collision_point,
  const std::vector<TrajectoryPoint> & traj_points, const double dist_to_bumper,
  const double min_velocity_to_reach_collision_point)
{
  const double dist_from_ego_to_obstacle =
    std::abs(autoware::motion_utils::calcSignedArcLength(
      traj_points, odometry.pose.pose.position, collision_point)) -
    dist_to_bumper;
  return dist_from_ego_to_obstacle /
         std::max(min_velocity_to_reach_collision_point, std::abs(odometry.twist.twist.linear.x));
}
}  // namespace

void ObstacleStopModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  clock_ = node.get_clock();
  logger_ = node.get_logger();

  // ros parameters
  ignore_crossing_obstacle_ =
    getOrDeclareParameter<bool>(node, "obstacle_stop.option.ignore_crossing_obstacle");
  suppress_sudden_stop_ =
    getOrDeclareParameter<bool>(node, "obstacle_stop.option.suppress_sudden_stop");

  common_param_ = CommonParam(node);
  stop_planning_param_ = StopPlanningParam(node, common_param_);
  obstacle_filtering_param_ = ObstacleFilteringParam(node);

  // common publisher
  processing_time_publisher_ =
    node.create_publisher<Float64Stamped>("~/debug/obstacle_stop/processing_time_ms", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle_stop/virtual_walls", 1);
  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle_stop/debug_markers", 1);

  // module publisher
  debug_stop_planning_info_pub_ =
    node.create_publisher<Float32MultiArrayStamped>("~/debug/stop_planning_info", 1);
  metrics_pub_ = node.create_publisher<MetricArray>("~/stop/metrics", 10);
  processing_time_detail_pub_ = node.create_publisher<universe_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/obstacle_stop", 1);
  // interface publisher
  objects_of_interest_marker_interface_ = std::make_unique<
    autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
    &node, "obstacle_stop");
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "obstacle_stop");

  // time keeper
  time_keeper_ = std::make_shared<universe_utils::TimeKeeper>(processing_time_detail_pub_);
}

void ObstacleStopModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using universe_utils::updateParam;

  updateParam(
    parameters, "obstacle_stop.option.ignore_crossing_obstacle", ignore_crossing_obstacle_);
  updateParam(parameters, "obstacle_stop.option.suppress_sudden_stop", suppress_sudden_stop_);
}

VelocityPlanningResult ObstacleStopModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw_trajectory_points,
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &
    smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. init variables
  stop_watch_.tic();
  debug_data_ptr_ = std::make_shared<DebugData>();
  metrics_manager_.init();
  const double dist_to_bumper =
    calc_dist_to_bumper(planner_data->is_driving_forward, planner_data->vehicle_info_);
  stop_planning_debug_info_.reset();
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::EGO_VELOCITY, planner_data->current_odometry.twist.twist.linear.x);
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::EGO_ACCELERATION,
    planner_data->current_acceleration.accel.accel.linear.x);
  trajectory_polygon_for_inside_map_.clear();
  trajectory_polygon_for_outside_ = std::nullopt;
  decimated_traj_polys_ = std::nullopt;

  // 2. pre-process
  const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
    raw_trajectory_points, planner_data->current_odometry.pose.pose,
    planner_data->ego_nearest_dist_threshold, planner_data->ego_nearest_yaw_threshold,
    planner_data->trajectory_polygon_collision_check.decimate_trajectory_step_length,
    stop_planning_param_.stop_margin);

  // 3. filter obstacles of predicted objects
  const auto stop_obstacles_for_predicted_object = filter_stop_obstacle_for_predicted_object(
    planner_data->current_odometry, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold,
    rclcpp::Time(planner_data->predicted_objects_header.stamp), raw_trajectory_points,
    decimated_traj_points, planner_data->objects, planner_data->is_driving_forward,
    planner_data->vehicle_info_, dist_to_bumper, planner_data->trajectory_polygon_collision_check);

  // 4. filter obstacles of point cloud
  const auto stop_obstacles_for_point_cloud =
    std::vector<StopObstacle>{};  // filter_stop_obstacle_for_point_cloud();

  // 5. concat stop obstacles by predicted objects and point cloud
  const auto stop_obstacles = stop_obstacles_for_predicted_object;
  concat_vectors(stop_obstacles_for_predicted_object, stop_obstacles_for_point_cloud);

  // 6. plan stop
  const auto stop_point =
    plan_stop(planner_data, raw_trajectory_points, stop_obstacles, dist_to_bumper);

  // 7. publish messages for debugging
  publish_debug_info();

  // 8. generate VelocityPlanningResult
  VelocityPlanningResult result;
  if (stop_point) {
    result.stop_points.push_back(*stop_point);
  }

  return result;
}

std::vector<StopObstacle> ObstacleStopModule::filter_stop_obstacle_for_predicted_object(
  const Odometry & odometry, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold, const rclcpp::Time & predicted_objects_stamp,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects, const bool is_driving_forward,
  const VehicleInfo & vehicle_info, const double dist_to_bumper,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & current_pose = odometry.pose.pose;

  std::vector<StopObstacle> stop_obstacles;
  for (const auto & object : objects) {
    autoware::universe_utils::ScopedTimeTrack st_for_each_object("for_each_object", *time_keeper_);

    // 1. rough filtering
    // 1.1. Check if the obstacle is in front of the ego.
    const double lon_dist_from_ego_to_obj =
      object->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
    if (lon_dist_from_ego_to_obj < 0.0) {
      continue;
    }

    // 1.2. Check if the rough lateral distance is smaller than the threshold.
    // TODO(murooka) outside obstacle stop was removed.
    const double min_lat_dist_to_traj_poly =
      utils::calc_possible_min_dist_from_obj_to_traj_poly(object, traj_points, vehicle_info);
    const uint8_t obj_label = object->predicted_object.classification.at(0).label;
    if (get_max_lat_margin(obj_label) < min_lat_dist_to_traj_poly) {
      continue;
    }

    // 2. precise filtering
    const auto & decimated_traj_polys = [&]() {
      autoware::universe_utils::ScopedTimeTrack st_get_decimated_traj_polys(
        "get_decimated_traj_polys", *time_keeper_);
      return get_decimated_traj_polys(
        traj_points, current_pose, vehicle_info, ego_nearest_dist_threshold,
        ego_nearest_yaw_threshold, trajectory_polygon_collision_check);
    }();
    const double dist_from_obj_to_traj_poly = [&]() {
      autoware::universe_utils::ScopedTimeTrack st_get_dist_to_traj_poly(
        "get_dist_to_traj_poly", *time_keeper_);
      return object->get_dist_to_traj_poly(decimated_traj_polys);
    }();

    // 2.1. filter target object inside trajectory
    const auto inside_stop_obstacle = filter_inside_stop_obstacle_for_predicted_object(
      odometry, traj_points, decimated_traj_points, object, predicted_objects_stamp,
      dist_from_obj_to_traj_poly, vehicle_info, dist_to_bumper, trajectory_polygon_collision_check);
    if (inside_stop_obstacle) {
      stop_obstacles.push_back(*inside_stop_obstacle);
      continue;
    }

    // 2.2 filter target object outside trajectory
    const auto outside_stop_obstacle = filter_outside_stop_obstacle_for_predicted_object(
      odometry, traj_points, decimated_traj_points, predicted_objects_stamp, object,
      dist_from_obj_to_traj_poly, is_driving_forward, vehicle_info, dist_to_bumper,
      trajectory_polygon_collision_check);
    if (outside_stop_obstacle) {
      stop_obstacles.push_back(*outside_stop_obstacle);
    }
  }

  // Check target obstacles' consistency
  check_consistency(predicted_objects_stamp, objects, stop_obstacles);

  prev_stop_obstacles_ = stop_obstacles;

  return stop_obstacles;
}

std::optional<StopObstacle> ObstacleStopModule::filter_inside_stop_obstacle_for_predicted_object(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const double dist_from_obj_poly_to_traj_poly, const VehicleInfo & vehicle_info,
  const double dist_to_bumper,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & predicted_object = object->predicted_object;
  const auto & obj_pose = object->get_predicted_pose(clock_->now(), predicted_objects_stamp);

  // 1. filter by label
  const uint8_t obj_label = predicted_object.classification.at(0).label;
  if (!is_in_vector(obj_label, obstacle_filtering_param_.inside_stop_object_types)) {
    return std::nullopt;
  }

  // 2. filter by lateral distance
  const double max_lat_margin = get_max_lat_margin(obj_label);
  // NOTE: max_lat_margin can be negative, so apply std::max with 1e-3.
  if (std::max(max_lat_margin, 1e-3) <= dist_from_obj_poly_to_traj_poly) {
    return std::nullopt;
  }

  // 3. filter by velocity
  if (!is_inside_stop_obstacle_velocity(object, traj_points)) {
    return std::nullopt;
  }

  // calculate collision points with trajectory with lateral stop margin
  const auto & p = trajectory_polygon_collision_check;
  const auto decimated_traj_polys_with_lat_margin = get_trajectory_polygon_for_inside(
    decimated_traj_points, vehicle_info, odometry.pose.pose, max_lat_margin,
    p.enable_to_consider_current_pose, p.time_to_convergence, p.decimate_trajectory_step_length);
  debug_data_ptr_->decimated_traj_polys = decimated_traj_polys_with_lat_margin;

  // 4. check if the obstacle really collides with the trajectory
  const auto collision_point = polygon_utils::get_collision_point(
    decimated_traj_points, decimated_traj_polys_with_lat_margin, obj_pose, clock_->now(),
    predicted_object.shape, dist_to_bumper);
  if (!collision_point) {
    return std::nullopt;
  }

  // 5. filter if the obstacle will cross and go out of trajectory soon
  if (
    ignore_crossing_obstacle_ &&
    is_crossing_transient_obstacle(
      odometry, traj_points, decimated_traj_points, object, dist_to_bumper,
      decimated_traj_polys_with_lat_margin, collision_point)) {
    return std::nullopt;
  }

  return StopObstacle{
    autoware::universe_utils::toHexString(predicted_object.object_id),
    predicted_objects_stamp,
    predicted_object.classification.at(0),
    obj_pose,
    predicted_object.shape,
    object->get_lon_vel_relative_to_traj(traj_points),
    collision_point->first,
    collision_point->second};
}

bool ObstacleStopModule::is_inside_stop_obstacle_velocity(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  const bool is_prev_object_stop =
    utils::get_obstacle_from_uuid(
      prev_stop_obstacles_,
      autoware::universe_utils::toHexString(object->predicted_object.object_id))
      .has_value();

  if (is_prev_object_stop) {
    if (
      obstacle_filtering_param_.obstacle_velocity_threshold_from_stop <
      object->get_lon_vel_relative_to_traj(traj_points)) {
      return false;
    }
    return true;
  }
  if (
    object->get_lon_vel_relative_to_traj(traj_points) <
    obstacle_filtering_param_.obstacle_velocity_threshold_to_stop) {
    return true;
  }
  return false;
}

bool ObstacleStopModule::is_crossing_transient_obstacle(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::shared_ptr<PlannerData::Object> object, const double dist_to_bumper,
  const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
  const std::optional<std::pair<geometry_msgs::msg::Point, double>> & collision_point) const
{
  // calculate the time to reach the collision point
  const double time_to_reach_stop_point = calc_time_to_reach_collision_point(
    odometry, collision_point->first, traj_points,
    stop_planning_param_.min_behavior_stop_margin + dist_to_bumper,
    obstacle_filtering_param_.min_velocity_to_reach_collision_point);
  if (
    time_to_reach_stop_point <= obstacle_filtering_param_.crossing_obstacle_collision_time_margin) {
    return false;
  }

  // get the highest confident predicted paths
  std::vector<PredictedPath> predicted_paths;
  for (const auto & path : object->predicted_object.kinematics.predicted_paths) {
    predicted_paths.push_back(path);
  }
  constexpr double prediction_resampling_time_interval = 0.1;
  constexpr double prediction_resampling_time_horizon = 10.0;
  const auto resampled_predicted_paths = resample_highest_confidence_predicted_paths(
    predicted_paths, prediction_resampling_time_interval, prediction_resampling_time_horizon, 1);
  if (resampled_predicted_paths.empty() || resampled_predicted_paths.front().path.empty()) {
    return false;
  }

  // predict object pose when the ego reaches the collision point
  const auto future_obj_pose = [&]() {
    const auto opt_future_obj_pose = autoware::object_recognition_utils::calcInterpolatedPose(
      resampled_predicted_paths.front(),
      time_to_reach_stop_point - obstacle_filtering_param_.crossing_obstacle_collision_time_margin);
    if (opt_future_obj_pose) {
      return *opt_future_obj_pose;
    }
    return resampled_predicted_paths.front().path.back();
  }();

  // check if the ego will collide with the obstacle
  auto future_predicted_object = object->predicted_object;
  future_predicted_object.kinematics.initial_pose_with_covariance.pose = future_obj_pose;
  const auto future_collision_point = polygon_utils::get_collision_point(
    decimated_traj_points, decimated_traj_polys_with_lat_margin,
    future_predicted_object.kinematics.initial_pose_with_covariance.pose, clock_->now(),
    future_predicted_object.shape, dist_to_bumper);
  const bool no_collision = !future_collision_point;

  return no_collision;
}

std::optional<StopObstacle> ObstacleStopModule::filter_outside_stop_obstacle_for_predicted_object(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const rclcpp::Time & predicted_objects_stamp, const std::shared_ptr<PlannerData::Object> object,
  const double dist_from_obj_poly_to_traj_poly, const bool is_driving_forward,
  const VehicleInfo & vehicle_info, const double dist_to_bumper,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & object_id =
    autoware::universe_utils::toHexString(object->predicted_object.object_id);

  // 1. filter by label
  const uint8_t obj_label = object->predicted_object.classification.at(0).label;
  if (!is_in_vector(obj_label, obstacle_filtering_param_.outside_stop_object_types)) {
    return std::nullopt;
  }

  // 2. filter by lateral distance
  const double max_lat_margin = obj_label == ObjectClassification::UNKNOWN
                                  ? obstacle_filtering_param_.max_lat_margin_against_unknown
                                  : obstacle_filtering_param_.max_lat_margin;
  if (dist_from_obj_poly_to_traj_poly < std::max(max_lat_margin, 1e-3)) {
    // Obstacle that is not inside of trajectory
    return std::nullopt;
  }

  const auto time_to_traj = dist_from_obj_poly_to_traj_poly /
                            std::max(1e-6, object->get_lat_vel_relative_to_traj(traj_points));
  if (time_to_traj > obstacle_filtering_param_.outside_max_lat_time_margin) {
    RCLCPP_DEBUG(
      logger_, "[Stop] Ignore outside obstacle (%s) since it's far from trajectory.",
      object_id.substr(0, 4).c_str());
    return std::nullopt;
  }

  // brkay54: For the pedestrians and bicycles, we need to check the collision point by thinking
  // they will stop with a predefined deceleration rate to avoid unnecessary stops.
  double resample_time_horizon = 10.0;
  if (obj_label == ObjectClassification::PEDESTRIAN) {
    resample_time_horizon =
      std::sqrt(
        std::pow(
          object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x, 2) +
        std::pow(
          object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y, 2)) /
      (2.0 * obstacle_filtering_param_.outside_pedestrian_deceleration_rate);
  } else if (obj_label == ObjectClassification::BICYCLE) {
    resample_time_horizon =
      std::sqrt(
        std::pow(
          object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x, 2) +
        std::pow(
          object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y, 2)) /
      (2.0 * obstacle_filtering_param_.outside_bicycle_deceleration_rate);
  }

  // Get the highest confidence predicted path
  std::vector<PredictedPath> predicted_paths;
  for (const auto & path : object->predicted_object.kinematics.predicted_paths) {
    predicted_paths.push_back(path);
  }
  constexpr double prediction_resampling_time_interval = 0.1;
  const auto resampled_predicted_paths = resample_highest_confidence_predicted_paths(
    predicted_paths, prediction_resampling_time_interval, resample_time_horizon,
    obstacle_filtering_param_.outside_num_of_predicted_paths);
  if (resampled_predicted_paths.empty()) {
    return std::nullopt;
  }

  const auto & p = trajectory_polygon_collision_check;
  const auto decimated_traj_polys_with_lat_margin = get_trajectory_polygon_for_outside(
    decimated_traj_points, vehicle_info, odometry.pose.pose, 0.0, p.enable_to_consider_current_pose,
    p.time_to_convergence, p.decimate_trajectory_step_length);

  const auto get_collision_point =
    [&]() -> std::optional<std::pair<geometry_msgs::msg::Point, double>> {
    for (const auto & predicted_path : resampled_predicted_paths) {
      const auto collision_point = create_collision_point_for_outside_stop_obstacle(
        odometry, traj_points, decimated_traj_points, decimated_traj_polys_with_lat_margin, object,
        predicted_objects_stamp, predicted_path, max_lat_margin, is_driving_forward, vehicle_info,
        dist_to_bumper, trajectory_polygon_collision_check.decimate_trajectory_step_length);
      if (collision_point) {
        return collision_point;
      }
    }
    return std::nullopt;
  };

  const auto collision_point = get_collision_point();

  if (collision_point) {
    return StopObstacle{
      object_id,
      predicted_objects_stamp,
      object->predicted_object.classification.at(0),
      object->get_predicted_pose(clock_->now(), predicted_objects_stamp),
      object->predicted_object.shape,
      object->get_lon_vel_relative_to_traj(traj_points),
      collision_point->first,
      collision_point->second};
  }
  return std::nullopt;
}

std::optional<std::pair<geometry_msgs::msg::Point, double>>
ObstacleStopModule::create_collision_point_for_outside_stop_obstacle(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const PredictedPath & resampled_predicted_path, double max_lat_margin,
  const bool is_driving_forward, const VehicleInfo & vehicle_info, const double dist_to_bumper,
  const double decimate_trajectory_step_length) const
{
  const auto & object_id =
    autoware::universe_utils::toHexString(object->predicted_object.object_id);

  std::vector<size_t> collision_index;
  const auto collision_points = polygon_utils::get_collision_points(
    decimated_traj_points, decimated_traj_polys, predicted_objects_stamp, resampled_predicted_path,
    object->predicted_object.shape, clock_->now(), is_driving_forward, collision_index,
    utils::calc_object_possible_max_dist_from_center(object->predicted_object.shape) +
      decimate_trajectory_step_length +
      std::hypot(
        vehicle_info.vehicle_length_m, vehicle_info.vehicle_width_m * 0.5 + max_lat_margin));
  if (collision_points.empty()) {
    RCLCPP_DEBUG(
      logger_,
      "[Stop] Ignore outside obstacle (%s) since there is no collision point between the "
      "predicted path "
      "and the ego.",
      object_id.substr(0, 4).c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(object);
    return std::nullopt;
  }

  const double collision_time_margin =
    calc_collision_time_margin(odometry, collision_points, traj_points, dist_to_bumper);
  if (obstacle_filtering_param_.crossing_obstacle_collision_time_margin < collision_time_margin) {
    RCLCPP_DEBUG(
      logger_, "[Stop] Ignore outside obstacle (%s) since it will not collide with the ego.",
      object_id.substr(0, 4).c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(object);
    return std::nullopt;
  }

  return polygon_utils::get_collision_point(
    decimated_traj_points, collision_index.front(), collision_points, dist_to_bumper);
}

std::optional<geometry_msgs::msg::Point> ObstacleStopModule::plan_stop(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<StopObstacle> & stop_obstacles, const double dist_to_bumper)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (stop_obstacles.empty()) {
    const auto markers =
      autoware::motion_utils::createDeletedStopVirtualWallMarker(clock_->now(), 0);
    autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);

    prev_stop_distance_info_ = std::nullopt;
    return std::nullopt;
  }

  std::optional<StopObstacle> determined_stop_obstacle{};
  std::optional<double> determined_zero_vel_dist{};
  std::optional<double> determined_desired_stop_margin{};

  const auto closest_stop_obstacles = get_closest_stop_obstacles(stop_obstacles);
  for (const auto & stop_obstacle : closest_stop_obstacles) {
    const auto ego_segment_idx =
      planner_data->find_segment_index(traj_points, planner_data->current_odometry.pose.pose);

    // calculate dist to collide
    const double dist_to_collide_on_ref_traj =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, ego_segment_idx) +
      stop_obstacle.dist_to_collide_on_decimated_traj;

    // calculate desired stop margin
    const double desired_stop_margin = calc_desired_stop_margin(
      planner_data, traj_points, stop_obstacle, dist_to_bumper, ego_segment_idx,
      dist_to_collide_on_ref_traj);

    // calculate stop point against the obstacle
    const auto candidate_zero_vel_dist = calc_candidate_zero_vel_dist(
      planner_data, traj_points, stop_obstacle, dist_to_collide_on_ref_traj, desired_stop_margin);
    if (!candidate_zero_vel_dist) {
      continue;
    }

    if (determined_stop_obstacle) {
      const bool is_same_param_types =
        (stop_obstacle.classification.label == determined_stop_obstacle->classification.label);
      if (
        (is_same_param_types && stop_obstacle.dist_to_collide_on_decimated_traj >
                                  determined_stop_obstacle->dist_to_collide_on_decimated_traj) ||
        (!is_same_param_types && *candidate_zero_vel_dist > determined_zero_vel_dist)) {
        continue;
      }
    }
    determined_zero_vel_dist = *candidate_zero_vel_dist;
    determined_stop_obstacle = stop_obstacle;
    determined_desired_stop_margin = desired_stop_margin;
  }

  if (!determined_zero_vel_dist) {
    // delete marker
    const auto markers =
      autoware::motion_utils::createDeletedStopVirtualWallMarker(clock_->now(), 0);
    autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);

    prev_stop_distance_info_ = std::nullopt;
    return std::nullopt;
  }

  // Hold previous stop distance if necessary
  hold_previous_stop_if_necessary(planner_data, traj_points, determined_zero_vel_dist);

  // Insert stop point
  const auto stop_point = calc_stop_point(
    planner_data, traj_points, dist_to_bumper, determined_stop_obstacle, determined_zero_vel_dist);

  // set stop_planning_debug_info
  set_stop_planning_debug_info(determined_stop_obstacle, determined_desired_stop_margin);

  return stop_point;
}

double ObstacleStopModule::calc_desired_stop_margin(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  const double dist_to_bumper, const size_t ego_segment_idx,
  const double dist_to_collide_on_ref_traj)
{
  // calculate default stop margin
  const double default_stop_margin = [&]() {
    const auto ref_traj_length =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, traj_points.size() - 1);
    if (dist_to_collide_on_ref_traj > ref_traj_length) {
      // Use terminal margin (terminal_stop_margin) for obstacle stop
      return stop_planning_param_.terminal_stop_margin;
    }
    return stop_planning_param_.stop_margin;
  }();

  // calculate stop margin on curve
  const double stop_margin_on_curve = calc_margin_from_obstacle_on_curve(
    planner_data, traj_points, stop_obstacle, dist_to_bumper, default_stop_margin);

  // calculate stop margin considering behavior's stop point
  // NOTE: If behavior stop point is ahead of the closest_obstacle_stop point within a certain
  //       margin we set closest_obstacle_stop_distance to closest_behavior_stop_distance
  const auto closest_behavior_stop_idx =
    autoware::motion_utils::searchZeroVelocityIndex(traj_points, ego_segment_idx + 1);
  if (closest_behavior_stop_idx) {
    const double closest_behavior_stop_dist_on_ref_traj =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, *closest_behavior_stop_idx);
    const double stop_dist_diff =
      closest_behavior_stop_dist_on_ref_traj - (dist_to_collide_on_ref_traj - stop_margin_on_curve);
    if (0.0 < stop_dist_diff && stop_dist_diff < stop_margin_on_curve) {
      return stop_planning_param_.min_behavior_stop_margin;
    }
  }
  return stop_margin_on_curve;
}

std::optional<double> ObstacleStopModule::calc_candidate_zero_vel_dist(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  const double dist_to_collide_on_ref_traj, const double desired_stop_margin)
{
  double candidate_zero_vel_dist = std::max(0.0, dist_to_collide_on_ref_traj - desired_stop_margin);
  if (suppress_sudden_stop_) {
    const auto acceptable_stop_acc = [&]() -> std::optional<double> {
      if (stop_planning_param_.get_param_type(stop_obstacle.classification) == "default") {
        return common_param_.limit_min_accel;
      }
      const double distance_to_judge_suddenness = std::min(
        calc_minimum_distance_to_stop(
          planner_data->current_odometry.twist.twist.linear.x, common_param_.limit_max_accel,
          stop_planning_param_.get_param(stop_obstacle.classification).sudden_object_acc_threshold),
        stop_planning_param_.get_param(stop_obstacle.classification).sudden_object_dist_threshold);
      if (candidate_zero_vel_dist > distance_to_judge_suddenness) {
        return common_param_.limit_min_accel;
      }
      if (stop_planning_param_.get_param(stop_obstacle.classification).abandon_to_stop) {
        RCLCPP_WARN(
          rclcpp::get_logger("ObstacleCruisePlanner::StopPlanner"),
          "[Cruise] abandon to stop against %s object",
          stop_planning_param_.object_types_maps.at(stop_obstacle.classification.label).c_str());
        return std::nullopt;
      } else {
        return stop_planning_param_.get_param(stop_obstacle.classification).limit_min_acc;
      }
    }();
    if (!acceptable_stop_acc) {
      return std::nullopt;
    }

    const double acceptable_stop_pos =
      autoware::motion_utils::calcSignedArcLength(
        traj_points, 0, planner_data->current_odometry.pose.pose.position) +
      calc_minimum_distance_to_stop(
        planner_data->current_odometry.twist.twist.linear.x, common_param_.limit_max_accel,
        acceptable_stop_acc.value());
    if (acceptable_stop_pos > candidate_zero_vel_dist) {
      candidate_zero_vel_dist = acceptable_stop_pos;
    }
  }
  return candidate_zero_vel_dist;
}

void ObstacleStopModule::hold_previous_stop_if_necessary(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  std::optional<double> & determined_zero_vel_dist)
{
  if (
    std::abs(planner_data->current_odometry.twist.twist.linear.x) <
      stop_planning_param_.hold_stop_velocity_threshold &&
    prev_stop_distance_info_) {
    // NOTE: We assume that the current trajectory's front point is ahead of the previous
    // trajectory's front point.
    const size_t traj_front_point_prev_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        prev_stop_distance_info_->first, traj_points.front().pose);
    const double diff_dist_front_points = autoware::motion_utils::calcSignedArcLength(
      prev_stop_distance_info_->first, 0, traj_points.front().pose.position,
      traj_front_point_prev_seg_idx);

    const double prev_zero_vel_dist = prev_stop_distance_info_->second - diff_dist_front_points;
    if (
      std::abs(prev_zero_vel_dist - determined_zero_vel_dist.value()) <
      stop_planning_param_.hold_stop_distance_threshold) {
      determined_zero_vel_dist.value() = prev_zero_vel_dist;
    }
  }
}

std::optional<geometry_msgs::msg::Point> ObstacleStopModule::calc_stop_point(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const double dist_to_bumper,
  const std::optional<StopObstacle> & determined_stop_obstacle,
  const std::optional<double> & determined_zero_vel_dist)
{
  auto output_traj_points = traj_points;

  // insert stop point
  const auto zero_vel_idx =
    autoware::motion_utils::insertStopPoint(0, *determined_zero_vel_dist, output_traj_points);
  if (!zero_vel_idx) {
    return std::nullopt;
  }

  // virtual wall marker for stop obstacle
  const auto markers = autoware::motion_utils::createStopVirtualWallMarker(
    output_traj_points.at(*zero_vel_idx).pose, "obstacle stop", clock_->now(), 0, dist_to_bumper,
    "", planner_data->is_driving_forward);
  autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);
  debug_data_ptr_->obstacles_to_stop.push_back(*determined_stop_obstacle);

  // update planning factor
  const auto stop_pose = output_traj_points.at(*zero_vel_idx).pose;
  planning_factor_interface_->add(
    output_traj_points, planner_data->current_odometry.pose.pose, stop_pose, PlanningFactor::STOP,
    SafetyFactorArray{});

  // Store stop reason debug data
  metrics_manager_.calculate_metrics(
    "PlannerInterface", "stop", planner_data, traj_points, stop_pose, *determined_stop_obstacle);

  prev_stop_distance_info_ = std::make_pair(output_traj_points, determined_zero_vel_dist.value());

  return stop_pose.position;
}

void ObstacleStopModule::set_stop_planning_debug_info(
  const std::optional<StopObstacle> & determined_stop_obstacle,
  const std::optional<double> & determined_desired_stop_margin) const
{
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_DISTANCE,
    determined_stop_obstacle->dist_to_collide_on_decimated_traj);
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_VELOCITY,
    determined_stop_obstacle->velocity);
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_TARGET_OBSTACLE_DISTANCE,
    determined_desired_stop_margin.value());
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_VELOCITY, 0.0);
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_ACCELERATION, 0.0);
}

void ObstacleStopModule::publish_debug_info()
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. debug marker
  MarkerArray debug_marker;

  // 1.1. obstacles
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_stop.size(); ++i) {
    // obstacle
    const auto obstacle_marker = utils::get_object_marker(
      debug_data_ptr_->obstacles_to_stop.at(i).pose, i, "obstacles", 1.0, 0.0, 0.0);
    debug_marker.markers.push_back(obstacle_marker);

    // collision point
    auto collision_point_marker = autoware::universe_utils::createDefaultMarker(
      "map", clock_->now(), "collision_points", 0, Marker::SPHERE,
      autoware::universe_utils::createMarkerScale(0.25, 0.25, 0.25),
      autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
    collision_point_marker.pose.position = debug_data_ptr_->obstacles_to_stop.at(i).collision_point;
    debug_marker.markers.push_back(collision_point_marker);
  }

  // 1.2. intentionally ignored obstacles
  for (size_t i = 0; i < debug_data_ptr_->intentionally_ignored_obstacles.size(); ++i) {
    const auto marker = utils::get_object_marker(
      debug_data_ptr_->intentionally_ignored_obstacles.at(i)
        ->predicted_object.kinematics.initial_pose_with_covariance.pose,
      i, "intentionally_ignored_obstacles", 0.0, 1.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  // 1.3. detection area
  auto decimated_traj_polys_marker = autoware::universe_utils::createDefaultMarker(
    "map", clock_->now(), "detection_area", 0, Marker::LINE_LIST,
    autoware::universe_utils::createMarkerScale(0.01, 0.0, 0.0),
    autoware::universe_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999));
  for (const auto & decimated_traj_poly : debug_data_ptr_->decimated_traj_polys) {
    for (size_t dp_idx = 0; dp_idx < decimated_traj_poly.outer().size(); ++dp_idx) {
      const auto & current_point = decimated_traj_poly.outer().at(dp_idx);
      const auto & next_point =
        decimated_traj_poly.outer().at((dp_idx + 1) % decimated_traj_poly.outer().size());

      decimated_traj_polys_marker.points.push_back(
        autoware::universe_utils::createPoint(current_point.x(), current_point.y(), 0.0));
      decimated_traj_polys_marker.points.push_back(
        autoware::universe_utils::createPoint(next_point.x(), next_point.y(), 0.0));
    }
  }
  debug_marker.markers.push_back(decimated_traj_polys_marker);

  debug_publisher_->publish(debug_marker);

  // 2. virtual wall
  virtual_wall_publisher_->publish(debug_data_ptr_->stop_wall_marker);

  // 3. stop planning info
  const auto stop_debug_msg = stop_planning_debug_info_.convert_to_message(clock_->now());
  debug_stop_planning_info_pub_->publish(stop_debug_msg);

  // 4. objects of interest
  objects_of_interest_marker_interface_->publishMarkerArray();

  // 5. metrics
  const auto metrics_msg = metrics_manager_.create_metric_array(clock_->now());
  metrics_pub_->publish(metrics_msg);

  // 6. processing time
  processing_time_publisher_->publish(create_float64_stamped(clock_->now(), stop_watch_.toc()));

  // 7. planning factor
  planning_factor_interface_->publish();
}

double ObstacleStopModule::calc_collision_time_margin(
  const Odometry & odometry, const std::vector<polygon_utils::PointWithStamp> & collision_points,
  const std::vector<TrajectoryPoint> & traj_points, const double dist_to_bumper) const
{
  const double time_to_reach_stop_point = calc_time_to_reach_collision_point(
    odometry, collision_points.front().point, traj_points,
    stop_planning_param_.min_behavior_stop_margin + dist_to_bumper,
    obstacle_filtering_param_.min_velocity_to_reach_collision_point);

  const double time_to_leave_collision_point =
    time_to_reach_stop_point +
    dist_to_bumper / std::max(
                       obstacle_filtering_param_.min_velocity_to_reach_collision_point,
                       odometry.twist.twist.linear.x);

  const double time_to_start_cross =
    (rclcpp::Time(collision_points.front().stamp) - clock_->now()).seconds();
  const double time_to_end_cross =
    (rclcpp::Time(collision_points.back().stamp) - clock_->now()).seconds();

  if (time_to_leave_collision_point < time_to_start_cross) {  // Ego goes first.
    return time_to_start_cross - time_to_reach_stop_point;
  }
  if (time_to_end_cross < time_to_reach_stop_point) {  // Obstacle goes first.
    return time_to_reach_stop_point - time_to_end_cross;
  }
  return 0.0;  // Ego and obstacle will collide.
}

std::vector<Polygon2d> ObstacleStopModule::get_trajectory_polygon_for_inside(
  const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length) const
{
  if (trajectory_polygon_for_inside_map_.count(lat_margin) == 0) {
    const auto traj_polys = polygon_utils::create_one_step_polygons(
      decimated_traj_points, vehicle_info, current_ego_pose, lat_margin,
      enable_to_consider_current_pose, time_to_convergence, decimate_trajectory_step_length);
    trajectory_polygon_for_inside_map_.emplace(lat_margin, traj_polys);
  }
  return trajectory_polygon_for_inside_map_.at(lat_margin);
}

std::vector<Polygon2d> ObstacleStopModule::get_trajectory_polygon_for_outside(
  const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length) const
{
  if (!trajectory_polygon_for_outside_) {
    trajectory_polygon_for_outside_ = polygon_utils::create_one_step_polygons(
      decimated_traj_points, vehicle_info, current_ego_pose, lat_margin,
      enable_to_consider_current_pose, time_to_convergence, decimate_trajectory_step_length);
  }
  return *trajectory_polygon_for_outside_;
}

void ObstacleStopModule::check_consistency(
  const rclcpp::Time & current_time,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  std::vector<StopObstacle> & stop_obstacles)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & prev_closest_stop_obstacle : prev_closest_stop_obstacles_) {
    const auto object_itr = std::find_if(
      objects.begin(), objects.end(),
      [&prev_closest_stop_obstacle](const std::shared_ptr<PlannerData::Object> & o) {
        return autoware::universe_utils::toHexString(o->predicted_object.object_id) ==
               prev_closest_stop_obstacle.uuid;
      });
    // If previous closest obstacle disappear from the perception result, do nothing anymore.
    if (object_itr == objects.end()) {
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
        (*object_itr)->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x <
          obstacle_filtering_param_.obstacle_velocity_threshold_from_stop &&
        elapsed_time < obstacle_filtering_param_.stop_obstacle_hold_time_threshold) {
        stop_obstacles.push_back(prev_closest_stop_obstacle);
      }
    }
  }

  prev_closest_stop_obstacles_ = get_closest_stop_obstacles(stop_obstacles);
}

double ObstacleStopModule::calc_margin_from_obstacle_on_curve(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  const double dist_to_bumper, const double default_stop_margin) const
{
  if (
    !stop_planning_param_.enable_approaching_on_curve || obstacle_filtering_param_.use_pointcloud) {
    return default_stop_margin;
  }

  // calculate short trajectory points towards obstacle
  const size_t obj_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, stop_obstacle.collision_point);
  std::vector<TrajectoryPoint> short_traj_points{traj_points.at(obj_segment_idx + 1)};
  double sum_short_traj_length{0.0};
  for (int i = obj_segment_idx; 0 <= i; --i) {
    short_traj_points.push_back(traj_points.at(i));

    if (
      1 < short_traj_points.size() &&
      stop_planning_param_.stop_margin + dist_to_bumper < sum_short_traj_length) {
      break;
    }
    sum_short_traj_length +=
      autoware::universe_utils::calcDistance2d(traj_points.at(i), traj_points.at(i + 1));
  }
  std::reverse(short_traj_points.begin(), short_traj_points.end());
  if (short_traj_points.size() < 2) {
    return default_stop_margin;
  }

  // calculate collision index between straight line from ego pose and object
  const auto calculate_distance_from_straight_ego_path =
    [&](const auto & ego_pose, const auto & object_polygon) {
      const auto forward_ego_pose = autoware::universe_utils::calcOffsetPose(
        ego_pose, stop_planning_param_.stop_margin + 3.0, 0.0, 0.0);
      const auto ego_straight_segment = autoware::universe_utils::Segment2d{
        convert_point(ego_pose.position), convert_point(forward_ego_pose.position)};
      return boost::geometry::distance(ego_straight_segment, object_polygon);
    };
  const auto resampled_short_traj_points = resample_trajectory_points(short_traj_points, 0.5);
  const auto object_polygon =
    autoware::universe_utils::toPolygon2d(stop_obstacle.pose, stop_obstacle.shape);
  const auto collision_idx = [&]() -> std::optional<size_t> {
    for (size_t i = 0; i < resampled_short_traj_points.size(); ++i) {
      const double dist_to_obj = calculate_distance_from_straight_ego_path(
        resampled_short_traj_points.at(i).pose, object_polygon);
      if (dist_to_obj < planner_data->vehicle_info_.vehicle_width_m / 2.0) {
        return i;
      }
    }
    return std::nullopt;
  }();
  if (!collision_idx) {
    return stop_planning_param_.min_stop_margin_on_curve;
  }
  if (*collision_idx == 0) {
    return default_stop_margin;
  }

  // calculate margin from obstacle
  const double partial_segment_length = [&]() {
    const double collision_segment_length = autoware::universe_utils::calcDistance2d(
      resampled_short_traj_points.at(*collision_idx - 1),
      resampled_short_traj_points.at(*collision_idx));
    const double prev_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(*collision_idx - 1).pose, object_polygon);
    const double next_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(*collision_idx).pose, object_polygon);
    return (next_dist - planner_data->vehicle_info_.vehicle_width_m / 2.0) /
           (next_dist - prev_dist) * collision_segment_length;
  }();

  const double short_margin_from_obstacle =
    partial_segment_length +
    autoware::motion_utils::calcSignedArcLength(
      resampled_short_traj_points, *collision_idx, stop_obstacle.collision_point) -
    dist_to_bumper + stop_planning_param_.additional_stop_margin_on_curve;

  return std::min(
    default_stop_margin,
    std::max(stop_planning_param_.min_stop_margin_on_curve, short_margin_from_obstacle));
}

std::vector<StopObstacle> ObstacleStopModule::get_closest_stop_obstacles(
  const std::vector<StopObstacle> & stop_obstacles)
{
  std::vector<StopObstacle> candidates{};
  for (const auto & stop_obstacle : stop_obstacles) {
    const auto itr =
      std::find_if(candidates.begin(), candidates.end(), [&stop_obstacle](const StopObstacle & co) {
        return co.classification.label == stop_obstacle.classification.label;
      });
    if (itr == candidates.end()) {
      candidates.emplace_back(stop_obstacle);
    } else if (
      stop_obstacle.dist_to_collide_on_decimated_traj < itr->dist_to_collide_on_decimated_traj) {
      *itr = stop_obstacle;
    }
  }
  return candidates;
}

double ObstacleStopModule::get_max_lat_margin(const uint8_t obj_label) const
{
  if (obj_label == ObjectClassification::UNKNOWN) {
    return obstacle_filtering_param_.max_lat_margin_against_unknown;
  }
  return obstacle_filtering_param_.max_lat_margin;
}

std::vector<Polygon2d> ObstacleStopModule::get_decimated_traj_polys(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  if (!decimated_traj_polys_) {
    const auto & p = trajectory_polygon_collision_check;
    const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
      traj_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold,
      p.decimate_trajectory_step_length, p.goal_extended_trajectory_length);
    decimated_traj_polys_ = polygon_utils::create_one_step_polygons(
      decimated_traj_points, vehicle_info, current_pose, 0.0, p.enable_to_consider_current_pose,
      p.time_to_convergence, p.decimate_trajectory_step_length);
  }
  return *decimated_traj_polys_;
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::ObstacleStopModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
