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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_STOP_MODULE_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_STOP_MODULE_HPP_

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"
#include "autoware/obstacle_cruise_planner/stop_planning_debug_info.hpp"
#include "autoware/obstacle_cruise_planner/type_alias.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
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

StopSpeedExceeded createStopSpeedExceededMsg(
  const rclcpp::Time & current_time, const bool stop_flag)
{
  StopSpeedExceeded msg{};
  msg.stamp = current_time;
  msg.stop_speed_exceeded = stop_flag;
  return msg;
}

double calcMinimumDistanceToStop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }

  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

autoware::universe_utils::Point2d convertPoint(const geometry_msgs::msg::Point & p)
{
  return autoware::universe_utils::Point2d{p.x, p.y};
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(traj, interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
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
}  // namespace

namespace autoware::motion_planning
{
class ObstacleStopModule
{
public:
  struct StopObstacle
  {
    StopObstacle(
      const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
      const ObjectClassification & object_classification, const geometry_msgs::msg::Pose & arg_pose,
      const Shape & arg_shape, const double arg_lon_velocity, const double arg_lat_velocity,
      const geometry_msgs::msg::Point arg_collision_point,
      const double arg_dist_to_collide_on_decimated_traj)
    : uuid(arg_uuid),
      stamp(arg_stamp),
      pose(arg_pose),
      velocity(arg_lon_velocity),
      lat_velocity(arg_lat_velocity),
      shape(arg_shape),
      collision_point(arg_collision_point),
      dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
      classification(object_classification)
    {
    }
    std::string uuid;
    rclcpp::Time stamp;
    geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
    double velocity;                // longitudinal velocity against ego's trajectory
    double lat_velocity;            // lateral velocity against ego's trajectory

    Shape shape;
    geometry_msgs::msg::Point
      collision_point;  // TODO(yuki_takagi): this member variable still used in
                        // calculateMarginFromObstacleOnCurve() and  should be removed as it can be
                        // replaced by ”dist_to_collide_on_decimated_traj”
    double dist_to_collide_on_decimated_traj;
    ObjectClassification classification;
  };
  ObstacleStopModule(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const VehicleInfo & vehicle_info, const EgoNearestParam & ego_nearest_param)
  : clock_(node.get_clock()),
    longitudinal_info_(longitudinal_info),
    vehicle_info_(vehicle_info),
    ego_nearest_param_(ego_nearest_param),
    stop_param_(StopParam(node, longitudinal_info))
  {
    inside_stop_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(node, "common.stop_obstacle_type.inside.");
    outside_stop_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(node, "common.stop_obstacle_type.outside.");
    use_pointcloud_for_stop_ = node.declare_parameter<bool>("common.stop_obstacle_type.pointcloud");

    stop_speed_exceeded_pub_ =
      node.create_publisher<StopSpeedExceeded>("~/output/stop_speed_exceeded", 1);

    velocity_factors_pub_ =
      node.create_publisher<VelocityFactorArray>("/planning/velocity_factors/obstacle_cruise", 1);
    metrics_pub_ = node.create_publisher<MetricArray>("~/metrics", 10);

    objects_of_interest_marker_interface_ = std::make_unique<
      autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
      &node, "obstacle_cruise_planner");

    debug_stop_planning_info_pub_ =
      node.create_publisher<Float32MultiArrayStamped>("~/debug/stop_planning_info", 1);
    debug_stop_wall_marker_pub_ = node.create_publisher<MarkerArray>("~/virtual_wall/stop", 1);
    debug_marker_pub_ = node.create_publisher<MarkerArray>("~/debug/marker", 1);
  }

  void postprocess() { objects_of_interest_marker_interface_->publishMarkerArray(); }

  std::vector<StopObstacle> determineEgoBehaviorAgainstPredictedObjectObstacles(
    const Odometry & odometry, const PredictedObjects & objects,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys, const std::vector<Obstacle> & obstacles,
    const bool is_driving_forward, const BehaviorDeterminationParam & behavior_determination_param,
    const double min_behavior_stop_margin)
  {
    is_driving_forward_ = is_driving_forward;
    behavior_determination_param_ = behavior_determination_param;
    min_behavior_stop_margin_ = min_behavior_stop_margin;

    // stop
    std::vector<StopObstacle> stop_obstacles;
    for (const auto & obstacle : obstacles) {
      const auto stop_obstacle = createStopObstacleForPredictedObject(
        odometry, decimated_traj_points, decimated_traj_polys, obstacle, obstacle.precise_lat_dist);
      if (stop_obstacle) {
        stop_obstacles.push_back(*stop_obstacle);
      }
    }
    // Check target obstacles' consistency
    checkConsistency(objects.header.stamp, objects, stop_obstacles);
    prev_stop_object_obstacles_ = stop_obstacles;

    return stop_obstacles;
  }

  std::vector<TrajectoryPoint> generateStopTrajectory(
    const PlannerData & planner_data, const std::vector<StopObstacle> & stop_obstacles)
  {
    *debug_data_ptr_ = DebugData();

    stop_watch_.tic(__func__);

    stop_planning_debug_info_.reset();
    stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::EGO_VELOCITY, planner_data.ego_vel);
    stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::EGO_VELOCITY, planner_data.ego_acc);

    const double abs_ego_offset = planner_data.is_driving_forward
                                    ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                    : std::abs(vehicle_info_.min_longitudinal_offset_m);

    if (stop_obstacles.empty()) {
      velocity_factors_pub_->publish(
        obstacle_cruise_utils::makeVelocityFactorArray(planner_data.current_time));
      // delete marker
      const auto markers =
        autoware::motion_utils::createDeletedStopVirtualWallMarker(planner_data.current_time, 0);
      autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);

      prev_stop_distance_info_ = std::nullopt;
      return planner_data.traj_points;
    }

    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), enable_debug_info_,
      "stop planning");

    std::optional<StopObstacle> determined_stop_obstacle{};
    std::optional<double> determined_zero_vel_dist{};
    std::optional<double> determined_desired_margin{};

    const auto closest_stop_obstacles = getClosestStopObstacles(stop_obstacles);
    for (const auto & stop_obstacle : closest_stop_obstacles) {
      const auto ego_segment_idx =
        ego_nearest_param_.findSegmentIndex(planner_data.traj_points, planner_data.ego_pose);
      const double dist_to_collide_on_ref_traj =
        autoware::motion_utils::calcSignedArcLength(planner_data.traj_points, 0, ego_segment_idx) +
        stop_obstacle.dist_to_collide_on_decimated_traj;

      const double desired_margin = [&]() {
        const double margin_from_obstacle =
          calculateMarginFromObstacleOnCurve(planner_data, stop_obstacle);
        // Use terminal margin (terminal_safe_distance_margin) for obstacle stop
        const auto ref_traj_length = autoware::motion_utils::calcSignedArcLength(
          planner_data.traj_points, 0, planner_data.traj_points.size() - 1);
        if (dist_to_collide_on_ref_traj > ref_traj_length) {
          return longitudinal_info_.terminal_safe_distance_margin;
        }

        // If behavior stop point is ahead of the closest_obstacle_stop point within a certain
        // margin we set closest_obstacle_stop_distance to closest_behavior_stop_distance
        const auto closest_behavior_stop_idx = autoware::motion_utils::searchZeroVelocityIndex(
          planner_data.traj_points, ego_segment_idx + 1);
        if (closest_behavior_stop_idx) {
          const double closest_behavior_stop_dist_on_ref_traj =
            autoware::motion_utils::calcSignedArcLength(
              planner_data.traj_points, 0, *closest_behavior_stop_idx);
          const double stop_dist_diff = closest_behavior_stop_dist_on_ref_traj -
                                        (dist_to_collide_on_ref_traj - margin_from_obstacle);
          if (0.0 < stop_dist_diff && stop_dist_diff < margin_from_obstacle) {
            return min_behavior_stop_margin_;
          }
        }
        return margin_from_obstacle;
      }();

      // calc stop point against the obstacle
      double candidate_zero_vel_dist = std::max(0.0, dist_to_collide_on_ref_traj - desired_margin);
      if (suppress_sudden_obstacle_stop_) {
        const auto acceptable_stop_acc = [&]() -> std::optional<double> {
          if (stop_param_.getParamType(stop_obstacle.classification) == "default") {
            return longitudinal_info_.limit_min_accel;
          }
          const double distance_to_judge_suddenness = std::min(
            calcMinimumDistanceToStop(
              planner_data.ego_vel, longitudinal_info_.limit_max_accel,
              stop_param_.getParam(stop_obstacle.classification).sudden_object_acc_threshold),
            stop_param_.getParam(stop_obstacle.classification).sudden_object_dist_threshold);
          if (candidate_zero_vel_dist > distance_to_judge_suddenness) {
            return longitudinal_info_.limit_min_accel;
          }
          if (stop_param_.getParam(stop_obstacle.classification).abandon_to_stop) {
            RCLCPP_WARN(
              rclcpp::get_logger("ObstacleCruisePlanner::StopPlanner"),
              "[Cruise] abandon to stop against %s object",
              stop_param_.types_maps.at(stop_obstacle.classification.label).c_str());
            return std::nullopt;
          } else {
            return stop_param_.getParam(stop_obstacle.classification).limit_min_acc;
          }
        }();
        if (!acceptable_stop_acc) {
          continue;
        }

        const double acceptable_stop_pos =
          autoware::motion_utils::calcSignedArcLength(
            planner_data.traj_points, 0, planner_data.ego_pose.position) +
          calcMinimumDistanceToStop(
            planner_data.ego_vel, longitudinal_info_.limit_max_accel, acceptable_stop_acc.value());
        if (acceptable_stop_pos > candidate_zero_vel_dist) {
          candidate_zero_vel_dist = acceptable_stop_pos;
        }
      }

      if (determined_stop_obstacle) {
        const bool is_same_param_types =
          (stop_obstacle.classification.label == determined_stop_obstacle->classification.label);
        if (
          (is_same_param_types && stop_obstacle.dist_to_collide_on_decimated_traj >
                                    determined_stop_obstacle->dist_to_collide_on_decimated_traj) ||
          (!is_same_param_types && candidate_zero_vel_dist > determined_zero_vel_dist)) {
          continue;
        }
      }
      determined_zero_vel_dist = candidate_zero_vel_dist;
      determined_stop_obstacle = stop_obstacle;
      determined_desired_margin = desired_margin;
    }

    if (!determined_zero_vel_dist) {
      // delete marker
      const auto markers =
        autoware::motion_utils::createDeletedStopVirtualWallMarker(planner_data.current_time, 0);
      autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);

      prev_stop_distance_info_ = std::nullopt;
      return planner_data.traj_points;
    }

    // Hold previous stop distance if necessary
    if (
      std::abs(planner_data.ego_vel) < longitudinal_info_.hold_stop_velocity_threshold &&
      prev_stop_distance_info_) {
      // NOTE: We assume that the current trajectory's front point is ahead of the previous
      // trajectory's front point.
      const size_t traj_front_point_prev_seg_idx =
        autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
          prev_stop_distance_info_->first, planner_data.traj_points.front().pose);
      const double diff_dist_front_points = autoware::motion_utils::calcSignedArcLength(
        prev_stop_distance_info_->first, 0, planner_data.traj_points.front().pose.position,
        traj_front_point_prev_seg_idx);

      const double prev_zero_vel_dist = prev_stop_distance_info_->second - diff_dist_front_points;
      if (
        std::abs(prev_zero_vel_dist - determined_zero_vel_dist.value()) <
        longitudinal_info_.hold_stop_distance_threshold) {
        determined_zero_vel_dist.value() = prev_zero_vel_dist;
      }
    }

    // Insert stop point
    auto output_traj_points = planner_data.traj_points;
    const auto zero_vel_idx =
      autoware::motion_utils::insertStopPoint(0, *determined_zero_vel_dist, output_traj_points);
    if (zero_vel_idx) {
      // virtual wall marker for stop obstacle
      const auto markers = autoware::motion_utils::createStopVirtualWallMarker(
        output_traj_points.at(*zero_vel_idx).pose, "obstacle stop", planner_data.current_time, 0,
        abs_ego_offset, "", planner_data.is_driving_forward);
      autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);
      debug_data_ptr_->obstacles_to_stop.push_back(*determined_stop_obstacle);

      // Publish Stop Reason
      const auto stop_pose = output_traj_points.at(*zero_vel_idx).pose;
      velocity_factors_pub_->publish(obstacle_cruise_utils::makeVelocityFactorArray(
        planner_data.current_time, PlanningBehavior::ROUTE_OBSTACLE, stop_pose));
      // Store stop reason debug data
      debug_data_ptr_->stop_metrics =
        makeMetrics("PlannerInterface", "stop", planner_data, stop_pose, *determined_stop_obstacle);
      // Publish if ego vehicle will over run against the stop point with a limit acceleration

      const bool will_over_run = determined_zero_vel_dist.value() >
                                 autoware::motion_utils::calcSignedArcLength(
                                   planner_data.traj_points, 0, planner_data.ego_pose.position) +
                                   determined_stop_obstacle->dist_to_collide_on_decimated_traj +
                                   determined_desired_margin.value() + 0.1;
      const auto stop_speed_exceeded_msg =
        createStopSpeedExceededMsg(planner_data.current_time, will_over_run);
      stop_speed_exceeded_pub_->publish(stop_speed_exceeded_msg);

      prev_stop_distance_info_ =
        std::make_pair(output_traj_points, determined_zero_vel_dist.value());
    }

    stop_planning_debug_info_.set(
      StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_DISTANCE,
      determined_stop_obstacle->dist_to_collide_on_decimated_traj);
    stop_planning_debug_info_.set(
      StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_VELOCITY,
      determined_stop_obstacle->velocity);
    stop_planning_debug_info_.set(
      StopPlanningDebugInfo::TYPE::STOP_TARGET_OBSTACLE_DISTANCE,
      determined_desired_margin.value());
    stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_VELOCITY, 0.0);
    stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_ACCELERATION, 0.0);

    const double calculation_time = stop_watch_.toc(__func__);
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("ObstacleCruisePlanner::SlowDownPlanner"), enable_calculation_time_info_,
      "  %s := %f [ms]", __func__, calculation_time);

    return output_traj_points;
  }

  double calcTimeToReachCollisionPoint(
    const Odometry & odometry, const geometry_msgs::msg::Point & collision_point,
    const std::vector<TrajectoryPoint> & traj_points, const double abs_ego_offset) const
  {
    const auto & p = behavior_determination_param_;
    const double dist_from_ego_to_obstacle =
      std::abs(autoware::motion_utils::calcSignedArcLength(
        traj_points, odometry.pose.pose.position, collision_point)) -
      abs_ego_offset;
    return dist_from_ego_to_obstacle /
           std::max(
             p.min_velocity_to_reach_collision_point, std::abs(odometry.twist.twist.linear.x));
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    updateCommonParam(parameters);
    stop_param_.onParam(parameters, longitudinal_info_);
  }

  Float32MultiArrayStamped getStopPlanningDebugMessage(const rclcpp::Time & current_time) const
  {
    return stop_planning_debug_info_.convertToMessage(current_time);
  }

  std::vector<Metric> makeMetrics(
    const std::string & module_name, const std::string & reason,
    const std::optional<PlannerData> & planner_data = std::nullopt,
    const std::optional<geometry_msgs::msg::Pose> & stop_pose = std::nullopt,
    const std::optional<StopObstacle> & stop_obstacle = std::nullopt)
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
        planner_data.value().traj_points, planner_data.value().ego_pose.position,
        stop_pose.value().position);

      Metric dist_to_stop_pose_metric;
      dist_to_stop_pose_metric.name = module_name + "/distance_to_stop_pose";
      dist_to_stop_pose_metric.unit = "double";
      dist_to_stop_pose_metric.value = std::to_string(dist_to_stop_pose);
      metrics.push_back(dist_to_stop_pose_metric);
    }

    if (stop_obstacle.has_value()) {
      // Obstacle info
      Metric collision_point_metric;
      const auto & p = stop_obstacle.value().collision_point;
      collision_point_metric.name = module_name + "/collision_point";
      collision_point_metric.unit = "string";
      collision_point_metric.value =
        "{" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + "}";
      metrics.push_back(collision_point_metric);
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
    addMetrics(debug_data_ptr_->stop_metrics);
    metrics_pub_->publish(metrics_msg);
    clearMetrics();
  }

  void clearMetrics() { debug_data_ptr_->stop_metrics = std::nullopt; }

  double getSafeDistanceMargin() const { return longitudinal_info_.safe_distance_margin; }

  void setParam(
    const bool enable_debug_info, const bool enable_calculation_time_info,
    const bool use_pointcloud, const double min_behavior_stop_margin,
    const double enable_approaching_on_curve, const double additional_safe_distance_margin_on_curve,
    const double min_safe_distance_margin_on_curve, const bool suppress_sudden_obstacle_stop)
  {
    enable_debug_info_ = enable_debug_info;
    enable_calculation_time_info_ = enable_calculation_time_info;
    use_pointcloud_ = use_pointcloud;
    min_behavior_stop_margin_ = min_behavior_stop_margin;
    enable_approaching_on_curve_ = enable_approaching_on_curve;
    additional_safe_distance_margin_on_curve_ = additional_safe_distance_margin_on_curve;
    min_safe_distance_margin_on_curve_ = min_safe_distance_margin_on_curve;
    suppress_sudden_obstacle_stop_ = suppress_sudden_obstacle_stop;
  }

  void publishDebugMarker()
  {
    // 1. publish debug marker
    MarkerArray debug_marker;

    // obstacles to stop
    for (size_t i = 0; i < debug_data_ptr_->obstacles_to_stop.size(); ++i) {
      // obstacle
      const auto obstacle_marker = obstacle_cruise_utils::getObjectMarker(
        debug_data_ptr_->obstacles_to_stop.at(i).pose, i, "obstacles_to_stop", 1.0, 0.0, 0.0);
      debug_marker.markers.push_back(obstacle_marker);

      // collision point
      auto collision_point_marker = autoware::universe_utils::createDefaultMarker(
        "map", clock_->now(), "stop_collision_points", 0, Marker::SPHERE,
        autoware::universe_utils::createMarkerScale(0.25, 0.25, 0.25),
        autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
      collision_point_marker.pose.position =
        debug_data_ptr_->obstacles_to_stop.at(i).collision_point;
      debug_marker.markers.push_back(collision_point_marker);
    }

    // intentionally ignored obstacles to cruise or stop
    for (size_t i = 0; i < debug_data_ptr_->intentionally_ignored_obstacles.size(); ++i) {
      const auto marker = obstacle_cruise_utils::getObjectMarker(
        debug_data_ptr_->intentionally_ignored_obstacles.at(i).pose, i,
        "intentionally_ignored_obstacles", 0.0, 1.0, 0.0);
      debug_marker.markers.push_back(marker);
    }

    debug_marker_pub_->publish(debug_marker);

    // 2. publish virtual wall for cruise and stop
    debug_stop_wall_marker_pub_->publish(debug_data_ptr_->stop_wall_marker);

    // stop
    const auto stop_debug_msg = getStopPlanningDebugMessage(clock_->now());
    debug_stop_planning_info_pub_->publish(stop_debug_msg);
  }

private:
  std::vector<StopObstacle> prev_closest_stop_object_obstacles_{};
  bool use_pointcloud_for_stop_;
  bool is_driving_forward_;
  std::vector<StopObstacle> prev_stop_object_obstacles_;
  std::vector<int> inside_stop_obstacle_types_;
  std::vector<int> outside_stop_obstacle_types_;

  rclcpp::Publisher<StopSpeedExceeded>::SharedPtr stop_speed_exceeded_pub_;

  struct DebugData
  {
    DebugData() = default;
    std::vector<Obstacle> intentionally_ignored_obstacles;
    std::vector<StopObstacle> obstacles_to_stop;
    MarkerArray stop_wall_marker;
    std::vector<autoware::universe_utils::Polygon2d> detection_polygons;
    std::optional<std::vector<Metric>> stop_metrics{std::nullopt};
  };

  struct StopParam
  {
    struct ObstacleSpecificParams
    {
      double limit_min_acc;
      double sudden_object_acc_threshold;
      double sudden_object_dist_threshold;
      bool abandon_to_stop;
    };
    const std::unordered_map<uint8_t, std::string> types_maps = {
      {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
      {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
      {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
      {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};
    std::unordered_map<std::string, ObstacleSpecificParams> type_specified_param_list;
    explicit StopParam(rclcpp::Node & node, const LongitudinalInfo & longitudinal_info)
    {
      const std::string param_prefix = "stop.type_specified_params.";
      std::vector<std::string> obstacle_labels{"default"};
      obstacle_labels =
        node.declare_parameter<std::vector<std::string>>(param_prefix + "labels", obstacle_labels);

      for (const auto & type_str : obstacle_labels) {
        if (type_str != "default") {
          ObstacleSpecificParams param{
            node.declare_parameter<double>(param_prefix + type_str + ".limit_min_acc"),
            node.declare_parameter<double>(
              param_prefix + type_str + ".sudden_object_acc_threshold"),
            node.declare_parameter<double>(
              param_prefix + type_str + ".sudden_object_dist_threshold"),
            node.declare_parameter<bool>(param_prefix + type_str + ".abandon_to_stop")};

          param.sudden_object_acc_threshold =
            std::min(param.sudden_object_acc_threshold, longitudinal_info.limit_min_accel);
          param.limit_min_acc = std::min(param.limit_min_acc, param.sudden_object_acc_threshold);

          type_specified_param_list.emplace(type_str, param);
        }
      }
    }
    void onParam(
      const std::vector<rclcpp::Parameter> & parameters, const LongitudinalInfo & longitudinal_info)
    {
      const std::string param_prefix = "stop.type_specified_params.";
      for (auto & [type_str, param] : type_specified_param_list) {
        if (type_str == "default") {
          continue;
        }
        autoware::universe_utils::updateParam<double>(
          parameters, param_prefix + type_str + ".limit_min_acc", param.limit_min_acc);
        autoware::universe_utils::updateParam<double>(
          parameters, param_prefix + type_str + ".sudden_object_acc_threshold",
          param.sudden_object_acc_threshold);
        autoware::universe_utils::updateParam<double>(
          parameters, param_prefix + type_str + ".sudden_object_dist_threshold",
          param.sudden_object_dist_threshold);
        autoware::universe_utils::updateParam<bool>(
          parameters, param_prefix + type_str + ".abandon_to_stop", param.abandon_to_stop);

        param.sudden_object_acc_threshold =
          std::min(param.sudden_object_acc_threshold, longitudinal_info.limit_min_accel);
        param.limit_min_acc = std::min(param.limit_min_acc, param.sudden_object_acc_threshold);
      }
    }
    std::string getParamType(const ObjectClassification label)
    {
      const auto type_str = types_maps.at(label.label);
      if (type_specified_param_list.count(type_str) == 0) {
        return "default";
      }
      return type_str;
    }
    ObstacleSpecificParams getParam(const ObjectClassification label)
    {
      return type_specified_param_list.at(getParamType(label));
    }
  };

  std::optional<StopObstacle> createStopObstacleForPredictedObject(
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
        // RCLCPP_INFO_EXPRESSION(
        //   get_logger(), enable_debug_info_,
        //   "[Stop] Ignore outside obstacle (%s) since it's far from trajectory.",
        //   object_id.c_str());
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
    const auto traj_polys_with_lat_margin = obstacle_cruise_utils::createOneStepPolygons(
      traj_points, vehicle_info_, odometry.pose.pose, max_lat_margin_for_stop,
      behavior_determination_param_);

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
        traj_points, traj_polys_with_lat_margin, tmp_future_obs, is_driving_forward_,
        vehicle_info_);

      return !future_collision_point;
    }();

    if (is_transient_obstacle) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Stop] Ignore inside obstacle (%s) since it is transient obstacle.", object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
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

  bool isInsideStopObstacle(const uint8_t label) const
  {
    const auto & types = inside_stop_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  bool isOutsideStopObstacle(const uint8_t label) const
  {
    const auto & types = outside_stop_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  bool isStopObstacle(const uint8_t label) const
  {
    return isInsideStopObstacle(label) || isOutsideStopObstacle(label);
  }

  void checkConsistency(
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

    prev_closest_stop_object_obstacles_ = getClosestStopObstacles(stop_obstacles);
  }

  std::optional<std::pair<geometry_msgs::msg::Point, double>>
  createCollisionPointForOutsideStopObstacle(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Polygon2d> & traj_polys, const Obstacle & obstacle,
    const PredictedPath & resampled_predicted_path, double max_lat_margin_for_stop) const
  {
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & p = behavior_determination_param_;

    std::vector<size_t> collision_index;
    const auto collision_points = polygon_utils::getCollisionPoints(
      traj_points, traj_polys, obstacle.stamp, resampled_predicted_path, obstacle.shape,
      clock_->now(), is_driving_forward_, collision_index,
      obstacle_cruise_utils::calcObstacleMaxLength(obstacle.shape) +
        p.decimate_trajectory_step_length +
        std::hypot(
          vehicle_info_.vehicle_length_m,
          vehicle_info_.vehicle_width_m * 0.5 + max_lat_margin_for_stop));
    if (collision_points.empty()) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Stop] Ignore outside obstacle (%s) since there is no collision point between the "
      //   "predicted path "
      //   "and the ego.",
      //   object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }

    const double collision_time_margin =
      calcCollisionTimeMargin(odometry, collision_points, traj_points, is_driving_forward_);
    if (p.collision_time_margin < collision_time_margin) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Stop] Ignore outside obstacle (%s) since it will not collide with the ego.",
      //   object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }

    return polygon_utils::getCollisionPoint(
      traj_points, collision_index.front(), collision_points, is_driving_forward_, vehicle_info_);
  }
  double calcCollisionTimeMargin(
    const Odometry & odometry, const std::vector<PointWithStamp> & collision_points,
    const std::vector<TrajectoryPoint> & traj_points, const bool is_driving_forward) const
  {
    const auto & p = behavior_determination_param_;
    const double abs_ego_offset =
      min_behavior_stop_margin_ + (is_driving_forward
                                     ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                     : std::abs(vehicle_info_.min_longitudinal_offset_m));
    const double time_to_reach_stop_point = ObstacleStopModule::calcTimeToReachCollisionPoint(
      odometry, collision_points.front().point, traj_points, abs_ego_offset);

    const double time_to_leave_collision_point =
      time_to_reach_stop_point +
      abs_ego_offset /
        std::max(p.min_velocity_to_reach_collision_point, odometry.twist.twist.linear.x);

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

  double calculateMarginFromObstacleOnCurve(
    const PlannerData & planner_data, const StopObstacle & stop_obstacle) const
  {
    if (!enable_approaching_on_curve_ || use_pointcloud_) {
      return longitudinal_info_.safe_distance_margin;
    }

    const double abs_ego_offset = planner_data.is_driving_forward
                                    ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                    : std::abs(vehicle_info_.min_longitudinal_offset_m);

    // calculate short trajectory points towards obstacle
    const size_t obj_segment_idx = autoware::motion_utils::findNearestSegmentIndex(
      planner_data.traj_points, stop_obstacle.collision_point);
    std::vector<TrajectoryPoint> short_traj_points{
      planner_data.traj_points.at(obj_segment_idx + 1)};
    double sum_short_traj_length{0.0};
    for (int i = obj_segment_idx; 0 <= i; --i) {
      short_traj_points.push_back(planner_data.traj_points.at(i));

      if (
        1 < short_traj_points.size() &&
        longitudinal_info_.safe_distance_margin + abs_ego_offset < sum_short_traj_length) {
        break;
      }
      sum_short_traj_length += autoware::universe_utils::calcDistance2d(
        planner_data.traj_points.at(i), planner_data.traj_points.at(i + 1));
    }
    std::reverse(short_traj_points.begin(), short_traj_points.end());
    if (short_traj_points.size() < 2) {
      return longitudinal_info_.safe_distance_margin;
    }

    // calculate collision index between straight line from ego pose and object
    const auto calculate_distance_from_straight_ego_path =
      [&](const auto & ego_pose, const auto & object_polygon) {
        const auto forward_ego_pose = autoware::universe_utils::calcOffsetPose(
          ego_pose, longitudinal_info_.safe_distance_margin + 3.0, 0.0, 0.0);
        const auto ego_straight_segment = autoware::universe_utils::Segment2d{
          convertPoint(ego_pose.position), convertPoint(forward_ego_pose.position)};
        return boost::geometry::distance(ego_straight_segment, object_polygon);
      };
    const auto resampled_short_traj_points = resampleTrajectoryPoints(short_traj_points, 0.5);
    const auto object_polygon =
      autoware::universe_utils::toPolygon2d(stop_obstacle.pose, stop_obstacle.shape);
    const auto collision_idx = [&]() -> std::optional<size_t> {
      for (size_t i = 0; i < resampled_short_traj_points.size(); ++i) {
        const double dist_to_obj = calculate_distance_from_straight_ego_path(
          resampled_short_traj_points.at(i).pose, object_polygon);
        if (dist_to_obj < vehicle_info_.vehicle_width_m / 2.0) {
          return i;
        }
      }
      return std::nullopt;
    }();
    if (!collision_idx) {
      return min_safe_distance_margin_on_curve_;
    }
    if (*collision_idx == 0) {
      return longitudinal_info_.safe_distance_margin;
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
      return (next_dist - vehicle_info_.vehicle_width_m / 2.0) / (next_dist - prev_dist) *
             collision_segment_length;
    }();

    const double short_margin_from_obstacle =
      partial_segment_length +
      autoware::motion_utils::calcSignedArcLength(
        resampled_short_traj_points, *collision_idx, stop_obstacle.collision_point) -
      abs_ego_offset + additional_safe_distance_margin_on_curve_;

    return std::min(
      longitudinal_info_.safe_distance_margin,
      std::max(min_safe_distance_margin_on_curve_, short_margin_from_obstacle));
  }

  // previous trajectory and distance to stop
  // NOTE: Previous trajectory is memorized to deal with nearest index search for overlapping or
  // crossing lanes.
  std::optional<std::pair<std::vector<TrajectoryPoint>, double>> prev_stop_distance_info_{
    std::nullopt};

  StopPlanningDebugInfo stop_planning_debug_info_;

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
  double min_behavior_stop_margin_;
  bool enable_approaching_on_curve_;
  double additional_safe_distance_margin_on_curve_;
  double min_safe_distance_margin_on_curve_;
  bool suppress_sudden_obstacle_stop_;

  void updateCommonParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    longitudinal_info_.onParam(parameters);
  }

  rclcpp::Clock::SharedPtr clock_;
  LongitudinalInfo longitudinal_info_;
  VehicleInfo vehicle_info_;
  EgoNearestParam ego_nearest_param_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;
  StopParam stop_param_;
  std::unique_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>
    objects_of_interest_marker_interface_;
  BehaviorDeterminationParam behavior_determination_param_;

  rclcpp::Publisher<MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_stop_wall_marker_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_stop_planning_info_pub_;

  std::vector<StopObstacle> getClosestStopObstacles(
    const std::vector<StopObstacle> & stop_obstacles)
  {
    std::vector<StopObstacle> candidates{};
    for (const auto & stop_obstacle : stop_obstacles) {
      const auto itr = std::find_if(
        candidates.begin(), candidates.end(), [&stop_obstacle](const StopObstacle & co) {
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
};
}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_STOP_MODULE_HPP_
