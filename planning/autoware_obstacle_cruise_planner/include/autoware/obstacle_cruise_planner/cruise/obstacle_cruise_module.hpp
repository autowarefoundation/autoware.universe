// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__CRUISE__OBSTACLE_CRUISE_MODULE_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__CRUISE__OBSTACLE_CRUISE_MODULE_HPP_

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/cruise/type_alias.hpp"
#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"
#include "autoware/obstacle_cruise_planner/stop/obstacle_stop_module.hpp"
#include "autoware/obstacle_cruise_planner/stop/stop_planning_debug_info.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>

#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
std::vector<PredictedPath> resampleHighestConfidencePredictedPaths1(
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

VelocityLimitClearCommand createVelocityLimitClearCommandMessage(
  const rclcpp::Time & current_time, const std::string & module_name)
{
  VelocityLimitClearCommand msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_cruise_planner." + module_name;
  msg.command = true;
  return msg;
}

}  // namespace

namespace autoware::motion_planning
{
// cruise
struct CruiseBehaviorDeterminationParam
{
  CruiseBehaviorDeterminationParam() = default;

  explicit CruiseBehaviorDeterminationParam(rclcpp::Node & node)
  {  // behavior determination
    outside_obstacle_min_velocity_threshold = node.declare_parameter<double>(
      "cruise.behavior_determination.outside_obstacle.obstacle_velocity_threshold");
    ego_obstacle_overlap_time_threshold = node.declare_parameter<double>(
      "cruise.behavior_determination.outside_obstacle.ego_obstacle_overlap_time_threshold");
    max_prediction_time_for_collision_check = node.declare_parameter<double>(
      "cruise.behavior_determination.outside_obstacle.max_prediction_time_for_collision_check");
    max_lat_margin_for_cruise =
      node.declare_parameter<double>("cruise.behavior_determination.max_lat_margin");
    enable_yield = node.declare_parameter<bool>("cruise.behavior_determination.yield.enable_yield");
    yield_lat_distance_threshold =
      node.declare_parameter<double>("cruise.behavior_determination.yield.lat_distance_threshold");
    max_lat_dist_between_obstacles = node.declare_parameter<double>(
      "cruise.behavior_determination.yield.max_lat_dist_between_obstacles");
    stopped_obstacle_velocity_threshold = node.declare_parameter<double>(
      "cruise.behavior_determination.yield.stopped_obstacle_velocity_threshold");
    max_obstacles_collision_time = node.declare_parameter<double>(
      "cruise.behavior_determination.yield.max_obstacles_collision_time");
    max_lat_time_margin_for_cruise = node.declare_parameter<double>(
      "cruise.behavior_determination.outside_obstacle.max_lateral_time_margin");
    num_of_predicted_paths_for_outside_cruise_obstacle = node.declare_parameter<int>(
      "cruise.behavior_determination.outside_obstacle.num_of_predicted_paths");

    obstacle_velocity_threshold_from_cruise_to_stop = node.declare_parameter<double>(
      "cruise.behavior_determination.obstacle_velocity_threshold_from_cruise_to_stop");
    obstacle_velocity_threshold_from_stop_to_cruise = node.declare_parameter<double>(
      "cruise.behavior_determination.obstacle_velocity_threshold_from_stop_to_cruise");
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    autoware::universe_utils::updateParam<double>(
      parameters, "cruise.behavior_determination.outside_obstacle.obstacle_velocity_threshold",
      outside_obstacle_min_velocity_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters,
      "cruise.behavior_determination.outside_obstacle.ego_obstacle_overlap_time_threshold",
      ego_obstacle_overlap_time_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters,
      "cruise.behavior_determination.outside_obstacle.max_prediction_time_for_collision_check",
      max_prediction_time_for_collision_check);
    autoware::universe_utils::updateParam<double>(
      parameters, "cruise.behavior_determination.max_lat_margin", max_lat_margin_for_cruise);
    autoware::universe_utils::updateParam<bool>(
      parameters, "cruise.behavior_determination.yield.enable_yield", enable_yield);
    autoware::universe_utils::updateParam<double>(
      parameters, "cruise.behavior_determination.yield.lat_distance_threshold",
      yield_lat_distance_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters, "cruise.behavior_determination.yield.max_lat_dist_between_obstacles",
      max_lat_dist_between_obstacles);
    autoware::universe_utils::updateParam<double>(
      parameters, "cruise.behavior_determination.yield.stopped_obstacle_velocity_threshold",
      stopped_obstacle_velocity_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters, "cruise.behavior_determination.yield.max_obstacles_collision_time",
      max_obstacles_collision_time);
    autoware::universe_utils::updateParam<double>(
      parameters, "cruise.behavior_determination.outside_obstacle.max_lateral_time_margin",
      max_lat_time_margin_for_cruise);
    autoware::universe_utils::updateParam<int>(
      parameters, "cruise.behavior_determination.outside_obstacle.num_of_predicted_paths",
      num_of_predicted_paths_for_outside_cruise_obstacle);
  }

  double outside_obstacle_min_velocity_threshold;
  double ego_obstacle_overlap_time_threshold;
  double max_prediction_time_for_collision_check;
  double max_lat_margin_for_cruise;
  bool enable_yield;
  double yield_lat_distance_threshold;
  double max_lat_dist_between_obstacles;
  double stopped_obstacle_velocity_threshold;
  double max_obstacles_collision_time;
  double max_lat_time_margin_for_cruise;
  int num_of_predicted_paths_for_outside_cruise_obstacle;
  double obstacle_velocity_threshold_from_cruise_to_stop;
  double obstacle_velocity_threshold_from_stop_to_cruise;
};

class ObstacleCruiseModule
{
public:
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

      idling_time = node.declare_parameter<double>("cruise.idling_time");
      min_ego_accel_for_rss = node.declare_parameter<double>("cruise.min_ego_accel_for_rss");
      min_object_accel_for_rss = node.declare_parameter<double>("cruise.min_object_accel_for_rss");

      safe_distance_margin = node.declare_parameter<double>("cruise.safe_distance_margin");
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

      autoware::universe_utils::updateParam<double>(parameters, "cruise.idling_time", idling_time);
      autoware::universe_utils::updateParam<double>(
        parameters, "cruise.min_ego_accel_for_rss", min_ego_accel_for_rss);
      autoware::universe_utils::updateParam<double>(
        parameters, "cruise.min_object_accel_for_rss", min_object_accel_for_rss);

      autoware::universe_utils::updateParam<double>(
        parameters, "cruise.safe_distance_margin", safe_distance_margin);
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
  };

  struct CruiseObstacle
  {
    CruiseObstacle(
      const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
      const geometry_msgs::msg::Pose & arg_pose, const double arg_lon_velocity,
      const double arg_lat_velocity, const std::vector<PointWithStamp> & arg_collision_points,
      bool arg_is_yield_obstacle = false)
    : uuid(arg_uuid),
      stamp(arg_stamp),
      pose(arg_pose),
      velocity(arg_lon_velocity),
      lat_velocity(arg_lat_velocity),
      collision_points(arg_collision_points),
      is_yield_obstacle(arg_is_yield_obstacle)
    {
    }
    std::string uuid;
    rclcpp::Time stamp;
    geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
    double velocity;                // longitudinal velocity against ego's trajectory
    double lat_velocity;            // lateral velocity against ego's trajectory

    std::vector<PointWithStamp> collision_points;  // time-series collision points
    bool is_yield_obstacle;
  };

  explicit ObstacleCruiseModule(rclcpp::Node & node)
  : clock_(node.get_clock()), longitudinal_info_(node)
  {
    enable_debug_info_ = node.declare_parameter<bool>("cruise.common.enable_debug_info");
    enable_calculation_time_info_ =
      node.declare_parameter<bool>("cruise.common.enable_calculation_time_info");

    inside_cruise_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(node, "cruise.obstacle_type.inside.");
    outside_cruise_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(node, "cruise.obstacle_type.outside.");

    objects_of_interest_marker_interface_ = std::make_unique<
      autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
      &node, "obstacle_cruise_planner");

    debug_cruise_wall_marker_pub_ = node.create_publisher<MarkerArray>("~/virtual_wall/cruise", 1);
    debug_marker_pub_ = node.create_publisher<MarkerArray>("~/debug/marker", 1);

    debug_cruise_planning_info_pub_ =
      node.create_publisher<Float32MultiArrayStamped>("~/debug/cruise_planning_info", 1);

    vel_limit_pub_ = node.create_publisher<VelocityLimit>(
      "~/output/velocity_limit", rclcpp::QoS{1}.transient_local());
    clear_vel_limit_pub_ = node.create_publisher<VelocityLimitClearCommand>(
      "~/output/clear_velocity_limit", rclcpp::QoS{1}.transient_local());

    cruise_behavior_determination_param_ = CruiseBehaviorDeterminationParam(node);
  }

  std::vector<TrajectoryPoint> plan(
    const std::vector<TrajectoryPoint> & base_traj_points, const std::vector<Obstacle> & obstacles,
    const CommonBehaviorDeterminationParam & common_behavior_determination_param,
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points)
  {
    common_behavior_determination_param_ = common_behavior_determination_param;

    const auto decimated_traj_points = obstacle_cruise_utils::decimateTrajectoryPoints(
      planner_data.current_odometry, base_traj_points, planner_data,
      common_behavior_determination_param_.decimate_trajectory_step_length, 0.0);
    const auto decimated_traj_polys = obstacle_cruise_utils::createOneStepPolygons(
      decimated_traj_points, planner_data.vehicle_info, planner_data.current_odometry.pose.pose,
      0.0, common_behavior_determination_param_);

    const auto cruise_obstacles = determineEgoBehaviorAgainstPredictedObjectObstacles(
      planner_data.current_odometry, decimated_traj_points, decimated_traj_polys, obstacles,
      planner_data.is_driving_forward, planner_data.vehicle_info);
    std::optional<VelocityLimit> cruise_vel_limit;
    const auto cruise_traj_points =
      generateCruiseTrajectory(planner_data, stop_traj_points, cruise_obstacles, cruise_vel_limit);
    publishVelocityLimit(cruise_vel_limit);
    publishMetrics(clock_->now());
    postprocess();
    publishDebugMarker();

    return cruise_traj_points;
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    updateCommonParam(parameters);
    updateCruiseParam(parameters);
  }

protected:
  std::vector<CruiseObstacle> prev_cruise_object_obstacles_;
  std::vector<int> inside_cruise_obstacle_types_;
  std::vector<int> outside_cruise_obstacle_types_;

  void postprocess() { objects_of_interest_marker_interface_->publishMarkerArray(); }

  std::vector<CruiseObstacle> determineEgoBehaviorAgainstPredictedObjectObstacles(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys, const std::vector<Obstacle> & obstacles,
    const bool is_driving_forward, const VehicleInfo & vehicle_info)
  {
    // cruise
    std::vector<CruiseObstacle> cruise_obstacles;
    for (const auto & obstacle : obstacles) {
      const auto cruise_obstacle = createCruiseObstacle(
        odometry, decimated_traj_points, decimated_traj_polys, obstacle, obstacle.precise_lat_dist,
        is_driving_forward, vehicle_info);
      if (cruise_obstacle) {
        cruise_obstacles.push_back(*cruise_obstacle);
      }
    }
    const auto & p = cruise_behavior_determination_param_;
    if (p.enable_yield) {
      const auto yield_obstacles =
        findYieldCruiseObstacles(obstacles, decimated_traj_points, vehicle_info);
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
    prev_cruise_object_obstacles_ = cruise_obstacles;

    return cruise_obstacles;
  }

  double calcDistanceToCollisionPoint(
    const PlannerData & planner_data, const geometry_msgs::msg::Point & collision_point)
  {
    const double offset = planner_data.is_driving_forward
                            ? std::abs(planner_data.vehicle_info.max_longitudinal_offset_m)
                            : std::abs(planner_data.vehicle_info.min_longitudinal_offset_m);

    const size_t ego_segment_idx = planner_data.findSegmentIndex(
      planner_data.traj_points, planner_data.current_odometry.pose.pose);

    const size_t collision_segment_idx =
      autoware::motion_utils::findNearestSegmentIndex(planner_data.traj_points, collision_point);

    const auto dist_to_collision_point = autoware::motion_utils::calcSignedArcLength(
      planner_data.traj_points, planner_data.current_odometry.pose.pose.position, ego_segment_idx,
      collision_point, collision_segment_idx);

    return dist_to_collision_point - offset;
  }

  virtual Float32MultiArrayStamped getCruisePlanningDebugMessage(
    [[maybe_unused]] const rclcpp::Time & current_time) const
  {
    return Float32MultiArrayStamped{};
  }

  virtual std::vector<TrajectoryPoint> generateCruiseTrajectory(
    [[maybe_unused]] const PlannerData & planner_data,
    [[maybe_unused]] const std::vector<TrajectoryPoint> & stop_traj_points,
    [[maybe_unused]] const std::vector<CruiseObstacle> & cruise_obstacles,
    [[maybe_unused]] std::optional<VelocityLimit> & vel_limit)
  {
    return std::vector<TrajectoryPoint>{};
  }

  std::vector<Metric> makeMetrics(
    const std::string & module_name, const std::string & reason,
    const std::optional<PlannerData> & planner_data = std::nullopt,
    const std::optional<geometry_msgs::msg::Pose> & stop_pose = std::nullopt)
  {
    auto metrics = std::vector<Metric>();

    // Create status
    {
      // Decision
      Metric decision_metric;
      decision_metric.name = module_name + "/decision";
      decision_metric.unit = "string";
      decision_metric.value = reason;
      metrics.push_back(decision_metric);
    }

    if (stop_pose.has_value() && planner_data.has_value()) {  // Stop info
      Metric stop_position_metric;
      stop_position_metric.name = module_name + "/stop_position";
      stop_position_metric.unit = "string";
      const auto & p = stop_pose.value().position;
      stop_position_metric.value =
        "{" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + "}";
      metrics.push_back(stop_position_metric);

      Metric stop_orientation_metric;
      stop_orientation_metric.name = module_name + "/stop_orientation";
      stop_orientation_metric.unit = "string";
      const auto & o = stop_pose.value().orientation;
      stop_orientation_metric.value = "{" + std::to_string(o.w) + ", " + std::to_string(o.x) +
                                      ", " + std::to_string(o.y) + ", " + std::to_string(o.z) + "}";
      metrics.push_back(stop_orientation_metric);

      const auto dist_to_stop_pose = autoware::motion_utils::calcSignedArcLength(
        planner_data->traj_points, planner_data->current_odometry.pose.pose.position,
        stop_pose.value().position);

      Metric dist_to_stop_pose_metric;
      dist_to_stop_pose_metric.name = module_name + "/distance_to_stop_pose";
      dist_to_stop_pose_metric.unit = "double";
      dist_to_stop_pose_metric.value = std::to_string(dist_to_stop_pose);
      metrics.push_back(dist_to_stop_pose_metric);
    }

    return metrics;
  }

  void publishMetrics(const rclcpp::Time & current_time)
  {
    // create array
    MetricArray metrics_msg;
    metrics_msg.stamp = current_time;

    auto addMetrics = [&metrics_msg](std::optional<std::vector<Metric>> & opt_metrics) {
      if (opt_metrics) {
        metrics_msg.metric_array.insert(
          metrics_msg.metric_array.end(), opt_metrics->begin(), opt_metrics->end());
      }
    };
    addMetrics(debug_data_ptr_->cruise_metrics);
    metrics_pub_->publish(metrics_msg);
    clearMetrics();
  }

  void clearMetrics() { debug_data_ptr_->cruise_metrics = std::nullopt; }

  void publishDebugMarker()
  {
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
        "map", clock_->now(), "cruise_collision_points", i, Marker::SPHERE,
        autoware::universe_utils::createMarkerScale(0.25, 0.25, 0.25),
        autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
      collision_point_marker.pose.position = stop_collision_points.at(i);
      debug_marker.markers.push_back(collision_point_marker);
    }

    debug_cruise_wall_marker_pub_->publish(debug_data_ptr_->cruise_wall_marker);
    debug_marker_pub_->publish(debug_marker);

    // cruise
    const auto cruise_debug_msg = getCruisePlanningDebugMessage(clock_->now());
    debug_cruise_planning_info_pub_->publish(cruise_debug_msg);
  }

  void publishVelocityLimit(const std::optional<VelocityLimit> & vel_limit)
  {
    const std::string module_name = "cruise";

    if (vel_limit) {
      vel_limit_pub_->publish(*vel_limit);
      need_to_clear_vel_limit_.at(module_name) = true;
      return;
    }

    if (!need_to_clear_vel_limit_.at(module_name)) {
      return;
    }

    // clear velocity limit
    const auto clear_vel_limit_msg =
      createVelocityLimitClearCommandMessage(clock_->now(), module_name);
    clear_vel_limit_pub_->publish(clear_vel_limit_msg);
    need_to_clear_vel_limit_.at(module_name) = false;
  }

  struct DebugData
  {
    DebugData() = default;
    std::vector<Obstacle> intentionally_ignored_obstacles;
    std::vector<CruiseObstacle> obstacles_to_cruise;
    MarkerArray cruise_wall_marker;
    std::optional<std::vector<Metric>> cruise_metrics{std::nullopt};
  };

  std::optional<CruiseObstacle> createCruiseObstacle(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Polygon2d> & traj_polys, const Obstacle & obstacle,
    const double precise_lat_dist, const bool is_driving_forward, const VehicleInfo & vehicle_info)
  {
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & p = cruise_behavior_determination_param_;

    // NOTE: When driving backward, Stop will be planned instead of cruise.
    //       When the obstacle is crossing the ego's trajectory, cruise can be ignored.
    if (!isCruiseObstacle(obstacle.classification.label) || !is_driving_forward) {
      return std::nullopt;
    }

    if (obstacle.longitudinal_velocity < 0.0) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore obstacle (%s) since it's driving in opposite direction.",
      //   object_id.c_str());
      return std::nullopt;
    }

    if (p.max_lat_margin_for_cruise < precise_lat_dist) {
      const auto time_to_traj = precise_lat_dist / std::max(1e-6, obstacle.approach_velocity);
      if (time_to_traj > p.max_lat_time_margin_for_cruise) {
        // RCLCPP_INFO_EXPRESSION(
        //   get_logger(), enable_debug_info_,
        //   "[Cruise] Ignore obstacle (%s) since it's far from trajectory.", object_id.c_str());
        return std::nullopt;
      }
    }

    if (isObstacleCrossing(traj_points, obstacle)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore obstacle (%s) since it's crossing the ego's trajectory..",
      //   object_id.c_str());
      return std::nullopt;
    }

    const auto collision_points = [&]() -> std::optional<std::vector<PointWithStamp>> {
      constexpr double epsilon = 1e-6;
      if (precise_lat_dist < epsilon) {
        // obstacle is inside the trajectory
        return createCollisionPointsForInsideCruiseObstacle(
          traj_points, traj_polys, obstacle, is_driving_forward, vehicle_info);
      }
      // obstacle is outside the trajectory
      // If the ego is stopping, do not plan cruise for outside obstacles. Stop will be planned.
      if (odometry.twist.twist.linear.x < 0.1) {
        return std::nullopt;
      }
      return createCollisionPointsForOutsideCruiseObstacle(
        traj_points, traj_polys, obstacle, is_driving_forward, vehicle_info);
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

  std::optional<std::vector<CruiseObstacle>> findYieldCruiseObstacles(
    const std::vector<Obstacle> & obstacles, const std::vector<TrajectoryPoint> & traj_points,
    const VehicleInfo & vehicle_info)
  {
    if (obstacles.empty() || traj_points.empty()) return std::nullopt;
    const auto & p = cruise_behavior_determination_param_;

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

    std::sort(
      moving_obstacles.begin(), moving_obstacles.end(), [](const auto & o1, const auto & o2) {
        return o1.ego_to_obstacle_distance < o2.ego_to_obstacle_distance;
      });

    std::vector<CruiseObstacle> yield_obstacles;
    for (const auto & moving_obstacle : moving_obstacles) {
      for (const auto & stopped_obstacle : stopped_obstacles) {
        const bool is_moving_obs_behind_of_stopped_obs =
          moving_obstacle.ego_to_obstacle_distance < stopped_obstacle.ego_to_obstacle_distance;
        const bool is_moving_obs_ahead_of_ego_front =
          moving_obstacle.ego_to_obstacle_distance > vehicle_info.vehicle_length_m;

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
            objects_of_interest_marker_interface_->insertObjectData(
              stopped_obstacle.pose, stopped_obstacle.shape, ColorName::RED);
          }
        }
      }
    }
    if (yield_obstacles.empty()) return std::nullopt;
    return yield_obstacles;
  }

  std::optional<std::vector<PointWithStamp>> createCollisionPointsForInsideCruiseObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
    const Obstacle & obstacle, const bool is_driving_forward,
    const VehicleInfo & vehicle_info) const
  {
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & crp = cruise_behavior_determination_param_;
    const auto & cp = common_behavior_determination_param_;

    // check label
    if (!isInsideCruiseObstacle(obstacle.classification.label)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore inside obstacle (%s) since its type is not designated.",
      //   object_id.c_str());
      return std::nullopt;
    }

    {  // consider hysteresis
      // const bool is_prev_obstacle_stop = getObstacleFromUuid(prev_stop_obstacles_,
      // obstacle.uuid).has_value();
      const bool is_prev_obstacle_cruise =
        obstacle_cruise_utils::getObstacleFromUuid(prev_cruise_object_obstacles_, obstacle.uuid)
          .has_value();

      if (is_prev_obstacle_cruise) {
        if (obstacle.longitudinal_velocity < crp.obstacle_velocity_threshold_from_cruise_to_stop) {
          return std::nullopt;
        }
        // NOTE: else is keeping cruise
      } else {  // if (is_prev_obstacle_stop) {
        // TODO(murooka) consider hysteresis for slow down
        // If previous obstacle is stop or does not exist.
        if (obstacle.longitudinal_velocity < crp.obstacle_velocity_threshold_from_stop_to_cruise) {
          return std::nullopt;
        }
        // NOTE: else is cruise from stop
      }
    }

    // Get highest confidence predicted path
    const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths1(
      obstacle.predicted_paths, cp.prediction_resampling_time_interval,
      cp.prediction_resampling_time_horizon, 1);

    if (resampled_predicted_paths.empty()) {
      return std::nullopt;
    }

    // calculate nearest collision point
    std::vector<size_t> collision_index;
    const auto collision_points = polygon_utils::getCollisionPoints(
      traj_points, traj_polys, obstacle.stamp, resampled_predicted_paths.front(), obstacle.shape,
      clock_->now(), is_driving_forward, collision_index,
      obstacle_cruise_utils::calcObstacleMaxLength(obstacle.shape) +
        cp.decimate_trajectory_step_length +
        std::hypot(
          vehicle_info.vehicle_length_m,
          vehicle_info.vehicle_width_m * 0.5 + crp.max_lat_margin_for_cruise));
    return collision_points;
  }

  std::optional<std::vector<PointWithStamp>> createCollisionPointsForOutsideCruiseObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
    const Obstacle & obstacle, const bool is_driving_forward,
    const VehicleInfo & vehicle_info) const
  {
    const auto & cp = common_behavior_determination_param_;
    const auto & crp = cruise_behavior_determination_param_;
    const auto & object_id = obstacle.uuid.substr(0, 4);

    // check label
    if (!isOutsideCruiseObstacle(obstacle.classification.label)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore outside obstacle (%s) since its type is not designated.",
      //   object_id.c_str());
      return std::nullopt;
    }

    if (
      std::hypot(obstacle.twist.linear.x, obstacle.twist.linear.y) <
      crp.outside_obstacle_min_velocity_threshold) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore outside obstacle (%s) since the obstacle velocity is low.",
      //   object_id.c_str());
      return std::nullopt;
    }

    // Get the highest confidence predicted paths
    const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths(
      obstacle.predicted_paths, cp.prediction_resampling_time_interval,
      cp.prediction_resampling_time_horizon,
      crp.num_of_predicted_paths_for_outside_cruise_obstacle);

    // calculate collision condition for cruise
    std::vector<size_t> collision_index;
    const auto getCollisionPoints = [&]() -> std::vector<PointWithStamp> {
      for (const auto & predicted_path : resampled_predicted_paths) {
        const auto collision_points = polygon_utils::getCollisionPoints(
          traj_points, traj_polys, obstacle.stamp, predicted_path, obstacle.shape, clock_->now(),
          is_driving_forward, collision_index,
          obstacle_cruise_utils::calcObstacleMaxLength(obstacle.shape) +
            cp.decimate_trajectory_step_length +
            std::hypot(
              vehicle_info.vehicle_length_m,
              vehicle_info.vehicle_width_m * 0.5 + crp.max_lat_margin_for_cruise),
          crp.max_prediction_time_for_collision_check);
        if (!collision_points.empty()) {
          return collision_points;
        }
      }
      return {};
    };

    const auto collision_points = getCollisionPoints();

    if (collision_points.empty()) {
      // Ignore vehicle obstacles outside the trajectory without collision
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore outside obstacle (%s) since there are no collision points.",
      //   object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }

    const double overlap_time =
      (rclcpp::Time(collision_points.back().stamp) - rclcpp::Time(collision_points.front().stamp))
        .seconds();
    if (overlap_time < crp.ego_obstacle_overlap_time_threshold) {
      // Ignore vehicle obstacles outside the trajectory, whose predicted path
      // overlaps the ego trajectory in a certain time.
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore outside obstacle (%s) since it will not collide with the ego.",
      //   object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
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

  std::optional<CruiseObstacle> createYieldCruiseObstacle(
    const Obstacle & obstacle, const std::vector<TrajectoryPoint> & traj_points)
  {
    if (traj_points.empty()) return std::nullopt;
    // check label
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & p = cruise_behavior_determination_param_;

    if (!isOutsideCruiseObstacle(obstacle.classification.label)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore yield obstacle (%s) since its type is not designated.",
      //   object_id.c_str());
      return std::nullopt;
    }

    if (
      std::hypot(obstacle.twist.linear.x, obstacle.twist.linear.y) <
      p.outside_obstacle_min_velocity_threshold) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore yield obstacle (%s) since the obstacle velocity is low.",
      //   object_id.c_str());
      return std::nullopt;
    }

    if (isObstacleCrossing(traj_points, obstacle)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore yield obstacle (%s) since it's crossing the ego's trajectory..",
      //   object_id.c_str());
      return std::nullopt;
    }

    const auto collision_points = [&]() -> std::optional<std::vector<PointWithStamp>> {
      const auto obstacle_idx =
        autoware::motion_utils::findNearestIndex(traj_points, obstacle.pose);
      if (!obstacle_idx) return std::nullopt;
      const auto collision_traj_point = traj_points.at(obstacle_idx.value());
      const auto object_time = clock_->now() + traj_points.at(obstacle_idx.value()).time_from_start;

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

  bool isInsideCruiseObstacle(const uint8_t label) const
  {
    const auto & types = inside_cruise_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  bool isOutsideCruiseObstacle(const uint8_t label) const
  {
    const auto & types = outside_cruise_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  bool isCruiseObstacle(const uint8_t label) const
  {
    return isInsideCruiseObstacle(label) || isOutsideCruiseObstacle(label);
  }

  bool isFrontCollideObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
    const size_t first_collision_idx) const
  {
    const auto obstacle_idx =
      autoware::motion_utils::findNearestIndex(traj_points, obstacle.pose.position);

    const double obstacle_to_col_points_distance =
      autoware::motion_utils::calcSignedArcLength(traj_points, obstacle_idx, first_collision_idx);
    const double obstacle_max_length = obstacle_cruise_utils::calcObstacleMaxLength(obstacle.shape);

    // If the obstacle is far in front of the collision point, the obstacle is behind the ego.
    return obstacle_to_col_points_distance > -obstacle_max_length;
  }

  bool isObstacleCrossing(
    const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle) const
  {
    const double diff_angle = calcDiffAngleAgainstTrajectory(traj_points, obstacle.pose);

    // NOTE: Currently predicted objects does not have orientation availability even
    // though sometimes orientation is not available.
    const bool is_obstacle_crossing_trajectory =
      common_behavior_determination_param_.crossing_obstacle_traj_angle_threshold <
        std::abs(diff_angle) &&
      common_behavior_determination_param_.crossing_obstacle_traj_angle_threshold <
        M_PI - std::abs(diff_angle);
    if (!is_obstacle_crossing_trajectory) {
      return false;
    }

    // Only obstacles crossing the ego's trajectory with high speed are considered.
    return true;
  }

  double calcRSSDistance(
    const double ego_vel, const double obstacle_vel, const double margin = 0.0) const
  {
    const auto & i = longitudinal_info_;
    const double rss_dist_with_margin =
      ego_vel * i.idling_time + std::pow(ego_vel, 2) * 0.5 / std::abs(i.min_ego_accel_for_rss) -
      std::pow(obstacle_vel, 2) * 0.5 / std::abs(i.min_object_accel_for_rss) + margin;
    return rss_dist_with_margin;
  }

  virtual void updateCruiseParam([[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
  {
  }

  // stop watch
  autoware::universe_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  rclcpp::Publisher<MetricArray>::SharedPtr metrics_pub_;
  rclcpp::Publisher<VelocityFactorArray>::SharedPtr velocity_factors_pub_;

  // Parameters
  bool enable_debug_info_{false};
  bool enable_calculation_time_info_{false};
  bool use_pointcloud_{false};

  void updateCommonParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    longitudinal_info_.onParam(parameters);
  }

  rclcpp::Clock::SharedPtr clock_;
  LongitudinalInfo longitudinal_info_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;

  std::unique_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>
    objects_of_interest_marker_interface_;
  CommonBehaviorDeterminationParam common_behavior_determination_param_;
  CruiseBehaviorDeterminationParam cruise_behavior_determination_param_;

  rclcpp::Publisher<MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_cruise_wall_marker_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_cruise_planning_info_pub_;

  rclcpp::Publisher<VelocityLimit>::SharedPtr vel_limit_pub_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr clear_vel_limit_pub_;

  std::unordered_map<std::string, bool> need_to_clear_vel_limit_{{"cruise", false}};
};
}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__CRUISE__OBSTACLE_CRUISE_MODULE_HPP_
