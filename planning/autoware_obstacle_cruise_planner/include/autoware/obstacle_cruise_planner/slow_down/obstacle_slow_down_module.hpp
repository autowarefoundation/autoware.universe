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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__SLOW_DOWN__OBSTACLE_SLOW_DOWN_MODULE_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__SLOW_DOWN__OBSTACLE_SLOW_DOWN_MODULE_HPP_

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/slow_down/type_alias.hpp"
#include "autoware/obstacle_cruise_planner/stop/stop_planning_debug_info.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"
#include "autoware/signal_processing/lowpass_filter_1d.hpp"
#include "autoware/universe_utils/ros/parameter.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"

#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_planning
{
using autoware::universe_utils::getOrDeclareParameter;

namespace
{
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

geometry_msgs::msg::Point toGeomPoint(const autoware::universe_utils::Point2d & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  return geom_point;
}

template <typename T>
std::optional<T> getObjectFromUuid(const std::vector<T> & objects, const std::string & target_uuid)
{
  const auto itr = std::find_if(objects.begin(), objects.end(), [&](const auto & object) {
    return object.uuid == target_uuid;
  });

  if (itr == objects.end()) {
    return std::nullopt;
  }
  return *itr;
}

// TODO(murooka) following two functions are copied from behavior_velocity_planner.
// These should be refactored.
double findReachTime(
  const double jerk, const double accel, const double velocity, const double distance,
  const double t_min, const double t_max)
{
  const double j = jerk;
  const double a = accel;
  const double v = velocity;
  const double d = distance;
  const double min = t_min;
  const double max = t_max;
  auto f = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  if (f(min, j, a, v, d) > 0 || f(max, j, a, v, d) < 0) {
    throw std::logic_error("[obstacle_cruise_planner](findReachTime): search range is invalid");
  }
  const double eps = 1e-5;
  const int warn_iter = 100;
  double lower = min;
  double upper = max;
  double t;
  int iter = 0;
  for (int i = 0;; i++) {
    t = 0.5 * (lower + upper);
    const double fx = f(t, j, a, v, d);
    // std::cout<<"fx: "<<fx<<" up: "<<upper<<" lo: "<<lower<<" t: "<<t<<std::endl;
    if (std::abs(fx) < eps) {
      break;
    } else if (fx > 0.0) {
      upper = t;
    } else {
      lower = t;
    }
    iter++;
    if (iter > warn_iter)
      std::cerr << "[obstacle_cruise_planner](findReachTime): current iter is over warning"
                << std::endl;
  }
  return t;
}

double calcDecelerationVelocityFromDistanceToTarget(
  const double max_slowdown_jerk, const double max_slowdown_accel, const double current_accel,
  const double current_velocity, const double distance_to_target)
{
  if (max_slowdown_jerk > 0 || max_slowdown_accel > 0) {
    throw std::logic_error("max_slowdown_jerk and max_slowdown_accel should be negative");
  }
  // case0: distance to target is behind ego
  if (distance_to_target <= 0) return current_velocity;
  auto ft = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  auto vt = [](const double t, const double j, const double a, const double v) {
    return j * t * t / 2.0 + a * t + v;
  };
  const double j_max = max_slowdown_jerk;
  const double a0 = current_accel;
  const double a_max = max_slowdown_accel;
  const double v0 = current_velocity;
  const double l = distance_to_target;
  const double t_const_jerk = (a_max - a0) / j_max;
  const double d_const_jerk_stop = ft(t_const_jerk, j_max, a0, v0, 0.0);
  const double d_const_acc_stop = l - d_const_jerk_stop;

  if (d_const_acc_stop < 0) {
    // case0: distance to target is within constant jerk deceleration
    // use binary search instead of solving cubic equation
    const double t_jerk = findReachTime(j_max, a0, v0, l, 0, t_const_jerk);
    const double velocity = vt(t_jerk, j_max, a0, v0);
    return velocity;
  } else {
    const double v1 = vt(t_const_jerk, j_max, a0, v0);
    const double discriminant_of_stop = 2.0 * a_max * d_const_acc_stop + v1 * v1;
    // case3: distance to target is farther than distance to stop
    if (discriminant_of_stop <= 0) {
      return 0.0;
    }
    // case2: distance to target is within constant accel deceleration
    // solve d = 0.5*a^2+v*t by t
    const double t_acc = (-v1 + std::sqrt(discriminant_of_stop)) / a_max;
    return vt(t_acc, 0.0, a_max, v1);
  }
  return current_velocity;
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

class ObstacleSlowDownModule
{
public:
  struct LongitudinalInfo
  {
    explicit LongitudinalInfo(rclcpp::Node & node)
    {
      max_accel = getOrDeclareParameter<double>(node, "normal.max_acc");
      min_accel = getOrDeclareParameter<double>(node, "normal.min_acc");
      max_jerk = getOrDeclareParameter<double>(node, "normal.max_jerk");
      min_jerk = getOrDeclareParameter<double>(node, "normal.min_jerk");
      limit_max_accel = getOrDeclareParameter<double>(node, "limit.max_acc");
      limit_min_accel = getOrDeclareParameter<double>(node, "limit.min_acc");
      limit_max_jerk = getOrDeclareParameter<double>(node, "limit.max_jerk");
      limit_min_jerk = getOrDeclareParameter<double>(node, "limit.min_jerk");
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
  };

  struct SlowDownObstacle
  {
    SlowDownObstacle(
      const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
      const ObjectClassification & object_classification, const geometry_msgs::msg::Pose & arg_pose,
      const double arg_lon_velocity, const double arg_lat_velocity,
      const double arg_precise_lat_dist,
      const geometry_msgs::msg::Point & arg_front_collision_point,
      const geometry_msgs::msg::Point & arg_back_collision_point)
    : uuid(arg_uuid),
      stamp(arg_stamp),
      pose(arg_pose),
      velocity(arg_lon_velocity),
      lat_velocity(arg_lat_velocity),
      precise_lat_dist(arg_precise_lat_dist),
      front_collision_point(arg_front_collision_point),
      back_collision_point(arg_back_collision_point),
      classification(object_classification)
    {
    }
    std::string uuid;
    rclcpp::Time stamp;
    geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
    double velocity;                // longitudinal velocity against ego's trajectory
    double lat_velocity;            // lateral velocity against ego's trajectory

    double precise_lat_dist;  // for efficient calculation
    geometry_msgs::msg::Point front_collision_point;
    geometry_msgs::msg::Point back_collision_point;
    ObjectClassification classification;
  };

  explicit ObstacleSlowDownModule(rclcpp::Node & node)
  : clock_(node.get_clock()), slow_down_param_(SlowDownParam(node)), longitudinal_info_(node)
  {
    enable_debug_info_ = getOrDeclareParameter<bool>(node, "slow_down.common.enable_debug_info");

    enable_slow_down_planning_ =
      getOrDeclareParameter<bool>(node, "slow_down.common.enable_slow_down_planning");
    slow_down_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(node, "slow_down.obstacle_type.");
    use_pointcloud_for_slow_down_ =
      getOrDeclareParameter<bool>(node, "slow_down.obstacle_type.pointcloud");

    moving_object_speed_threshold =
      getOrDeclareParameter<double>(node, "slow_down.moving_object_speed_threshold");
    moving_object_hysteresis_range =
      getOrDeclareParameter<double>(node, "slow_down.moving_object_hysteresis_range");

    objects_of_interest_marker_interface_ = std::make_unique<
      autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
      &node, "obstacle_cruise_planner");

    debug_slow_down_wall_marker_pub_ =
      node.create_publisher<MarkerArray>("~/virtual_wall/slow_down", 1);
    debug_marker_pub_ = node.create_publisher<MarkerArray>("~/debug/marker", 1);

    metrics_pub_ = node.create_publisher<MetricArray>("~/slow_down/metrics", 10);

    debug_slow_down_planning_info_pub_ =
      node.create_publisher<Float32MultiArrayStamped>("~/debug/slow_down_planning_info", 1);

    velocity_factors_pub_ = node.create_publisher<VelocityFactorArray>(
      "/planning/velocity_factors/obstacle_cruise/stop", 1);

    vel_limit_pub_ = node.create_publisher<VelocityLimit>(
      "~/output/velocity_limit", rclcpp::QoS{1}.transient_local());
    clear_vel_limit_pub_ = node.create_publisher<VelocityLimitClearCommand>(
      "~/output/clear_velocity_limit", rclcpp::QoS{1}.transient_local());
  }

  std::vector<TrajectoryPoint> plan(
    const std::vector<TrajectoryPoint> & base_traj_points,
    const CommonBehaviorDeterminationParam & common_behavior_determination_param,
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & cruise_traj_points)
  {
    debug_data_ptr_ = std::make_shared<DebugData>();
    common_behavior_determination_param_ = common_behavior_determination_param;

    const auto decimated_traj_points = obstacle_cruise_utils::decimateTrajectoryPoints(
      planner_data.current_odometry, base_traj_points, planner_data,
      common_behavior_determination_param_.decimate_trajectory_step_length, 0.0);

    const auto slow_down_obstacles = determineEgoBehaviorAgainstPredictedObjectObstacles(
      planner_data.current_odometry, decimated_traj_points, planner_data.obstacles,
      planner_data.vehicle_info);
    std::optional<VelocityLimit> slow_down_vel_limit;
    const auto slow_down_traj_points = generateSlowDownTrajectory(
      planner_data, cruise_traj_points, slow_down_obstacles, slow_down_vel_limit,
      planner_data.vehicle_info);
    publishVelocityLimit(slow_down_vel_limit);
    publishMetrics(clock_->now());
    postprocess();
    publishDebugMarker();

    return slow_down_traj_points;
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    updateCommonParam(parameters);
    slow_down_param_.onParam(parameters);
  }

private:
  std::vector<SlowDownObstacle> determineEgoBehaviorAgainstPredictedObjectObstacles(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<PredictedObjectBasedObstacle> & obstacles, const VehicleInfo & vehicle_info)
  {
    // slow down
    slow_down_condition_counter_.resetCurrentUuids();
    std::vector<SlowDownObstacle> slow_down_obstacles;
    for (const auto & obstacle : obstacles) {
      const auto slow_down_obstacle = createSlowDownObstacleForPredictedObject(
        odometry, decimated_traj_points, obstacle, obstacle.precise_lat_dist, vehicle_info);
      if (slow_down_obstacle) {
        slow_down_obstacles.push_back(*slow_down_obstacle);
        continue;
      }
    }
    slow_down_condition_counter_.removeCounterUnlessUpdated();
    prev_slow_down_object_obstacles_ = slow_down_obstacles;

    return slow_down_obstacles;
  }

  std::vector<TrajectoryPoint> generateSlowDownTrajectory(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & cruise_traj_points,
    const std::vector<SlowDownObstacle> & obstacles,
    [[maybe_unused]] std::optional<VelocityLimit> & vel_limit, const VehicleInfo & vehicle_info)
  {
    auto slow_down_traj_points = cruise_traj_points;
    slow_down_debug_multi_array_ = Float32MultiArrayStamped();

    const double dist_to_ego = [&]() {
      const size_t ego_seg_idx = planner_data.findSegmentIndex(
        slow_down_traj_points, planner_data.current_odometry.pose.pose);
      return autoware::motion_utils::calcSignedArcLength(
        slow_down_traj_points, 0, planner_data.current_odometry.pose.pose.position, ego_seg_idx);
    }();
    const double abs_ego_offset = planner_data.is_driving_forward
                                    ? std::abs(vehicle_info.max_longitudinal_offset_m)
                                    : std::abs(vehicle_info.min_longitudinal_offset_m);

    // define function to insert slow down velocity to trajectory
    const auto insert_point_in_trajectory = [&](const double lon_dist) -> std::optional<size_t> {
      const auto inserted_idx =
        autoware::motion_utils::insertTargetPoint(0, lon_dist, slow_down_traj_points);
      if (inserted_idx) {
        if (inserted_idx.value() + 1 <= slow_down_traj_points.size() - 1) {
          // zero-order hold for velocity interpolation
          slow_down_traj_points.at(inserted_idx.value()).longitudinal_velocity_mps =
            slow_down_traj_points.at(inserted_idx.value() + 1).longitudinal_velocity_mps;
        }
        return inserted_idx.value();
      }
      return std::nullopt;
    };

    std::vector<SlowDownOutput> new_prev_slow_down_output;
    for (size_t i = 0; i < obstacles.size(); ++i) {
      const auto & obstacle = obstacles.at(i);
      const auto prev_output = getObjectFromUuid(prev_slow_down_output_, obstacle.uuid);

      const bool is_obstacle_moving = [&]() -> bool {
        const auto object_vel_norm = std::hypot(obstacle.velocity, obstacle.lat_velocity);
        if (!prev_output) return object_vel_norm > moving_object_speed_threshold;
        if (prev_output->is_moving)
          return object_vel_norm > moving_object_speed_threshold - moving_object_hysteresis_range;
        return object_vel_norm > moving_object_speed_threshold + moving_object_hysteresis_range;
      }();

      // calculate slow down start distance, and insert slow down velocity
      const auto dist_vec_to_slow_down = calculateDistanceToSlowDownWithConstraints(
        planner_data, slow_down_traj_points, obstacle, prev_output, dist_to_ego, is_obstacle_moving,
        vehicle_info);
      if (!dist_vec_to_slow_down) {
        RCLCPP_INFO_EXPRESSION(
          rclcpp::get_logger("ObstacleCruisePlanner::PlannerInterface"), enable_debug_info_,
          "[SlowDown] Ignore obstacle (%s) since distance to slow down is not valid",
          obstacle.uuid.c_str());
        continue;
      }
      const auto dist_to_slow_down_start = std::get<0>(*dist_vec_to_slow_down);
      const auto dist_to_slow_down_end = std::get<1>(*dist_vec_to_slow_down);
      const auto feasible_slow_down_vel = std::get<2>(*dist_vec_to_slow_down);

      // calculate slow down end distance, and insert slow down velocity
      // NOTE: slow_down_start_idx will not be wrong since inserted back point is after inserted
      // front point.
      const auto slow_down_start_idx = insert_point_in_trajectory(dist_to_slow_down_start);
      const auto slow_down_end_idx = dist_to_slow_down_start < dist_to_slow_down_end
                                       ? insert_point_in_trajectory(dist_to_slow_down_end)
                                       : std::nullopt;
      if (!slow_down_end_idx) {
        continue;
      }

      // calculate slow down velocity
      const double stable_slow_down_vel = [&]() {
        if (prev_output) {
          return autoware::signal_processing::lowpassFilter(
            feasible_slow_down_vel, prev_output->target_vel,
            slow_down_param_.lpf_gain_slow_down_vel);
        }
        return feasible_slow_down_vel;
      }();

      // insert slow down velocity between slow start and end
      for (size_t j = (slow_down_start_idx ? *slow_down_start_idx : 0); j <= *slow_down_end_idx;
           ++j) {
        auto & traj_point = slow_down_traj_points.at(j);
        traj_point.longitudinal_velocity_mps =
          std::min(traj_point.longitudinal_velocity_mps, static_cast<float>(stable_slow_down_vel));
      }

      // add debug data
      slow_down_debug_multi_array_.data.push_back(obstacle.precise_lat_dist);
      slow_down_debug_multi_array_.data.push_back(dist_to_slow_down_start);
      slow_down_debug_multi_array_.data.push_back(dist_to_slow_down_end);
      slow_down_debug_multi_array_.data.push_back(feasible_slow_down_vel);
      slow_down_debug_multi_array_.data.push_back(stable_slow_down_vel);
      slow_down_debug_multi_array_.data.push_back(
        slow_down_start_idx ? *slow_down_start_idx : -1.0);
      slow_down_debug_multi_array_.data.push_back(*slow_down_end_idx);

      // add virtual wall
      if (slow_down_start_idx && slow_down_end_idx) {
        const size_t ego_idx =
          planner_data.findIndex(slow_down_traj_points, planner_data.current_odometry.pose.pose);
        const size_t slow_down_wall_idx = [&]() {
          if (ego_idx < *slow_down_start_idx) return *slow_down_start_idx;
          if (ego_idx < *slow_down_end_idx) return ego_idx;
          return *slow_down_end_idx;
        }();

        const auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
          slow_down_traj_points.at(slow_down_wall_idx).pose, "obstacle slow down",
          planner_data.current_time, i, abs_ego_offset, "", planner_data.is_driving_forward);
        velocity_factors_pub_->publish(obstacle_cruise_utils::makeVelocityFactorArray(
          planner_data.current_time, PlanningBehavior::ROUTE_OBSTACLE,
          slow_down_traj_points.at(slow_down_wall_idx).pose));
        autoware::universe_utils::appendMarkerArray(
          markers, &debug_data_ptr_->slow_down_wall_marker);
      }

      // add debug virtual wall
      if (slow_down_start_idx) {
        const auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
          slow_down_traj_points.at(*slow_down_start_idx).pose, "obstacle slow down start",
          planner_data.current_time, i * 2, abs_ego_offset, "", planner_data.is_driving_forward);
        autoware::universe_utils::appendMarkerArray(
          markers, &debug_data_ptr_->slow_down_debug_wall_marker);
      }
      if (slow_down_end_idx) {
        const auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
          slow_down_traj_points.at(*slow_down_end_idx).pose, "obstacle slow down end",
          planner_data.current_time, i * 2 + 1, abs_ego_offset, "",
          planner_data.is_driving_forward);
        autoware::universe_utils::appendMarkerArray(
          markers, &debug_data_ptr_->slow_down_debug_wall_marker);
      }

      // Add debug data
      debug_data_ptr_->obstacles_to_slow_down.push_back(obstacle);
      // if (!debug_data_ptr_->stop_metrics.has_value()) {
      //   debug_data_ptr_->slow_down_metrics =
      //     makeMetrics("PlannerInterface", "slow_down", planner_data);
      // }

      // update prev_slow_down_output_
      new_prev_slow_down_output.push_back(SlowDownOutput{
        obstacle.uuid, slow_down_traj_points, slow_down_start_idx, slow_down_end_idx,
        stable_slow_down_vel, feasible_slow_down_vel, obstacle.precise_lat_dist,
        is_obstacle_moving});
    }

    // update prev_slow_down_output_
    prev_slow_down_output_ = new_prev_slow_down_output;

    return slow_down_traj_points;
  }

  void postprocess() { objects_of_interest_marker_interface_->publishMarkerArray(); }

  Float32MultiArrayStamped getSlowDownPlanningDebugMessage(const rclcpp::Time & current_time)
  {
    slow_down_debug_multi_array_.stamp = current_time;
    return slow_down_debug_multi_array_;
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
    addMetrics(debug_data_ptr_->slow_down_metrics);
    metrics_pub_->publish(metrics_msg);
    clearMetrics();
  }

  void clearMetrics() { debug_data_ptr_->slow_down_metrics = std::nullopt; }

  void publishDebugMarker()
  {
    // 1. publish debug marker
    MarkerArray debug_marker;

    // obstacles to slow down
    for (size_t i = 0; i < debug_data_ptr_->obstacles_to_slow_down.size(); ++i) {
      // obstacle
      const auto obstacle_marker = obstacle_cruise_utils::getObjectMarker(
        debug_data_ptr_->obstacles_to_slow_down.at(i).pose, i, "obstacles_to_slow_down", 0.7, 0.7,
        0.0);
      debug_marker.markers.push_back(obstacle_marker);

      // collision points
      auto front_collision_point_marker = autoware::universe_utils::createDefaultMarker(
        "map", clock_->now(), "slow_down_collision_points", i * 2 + 0, Marker::SPHERE,
        autoware::universe_utils::createMarkerScale(0.25, 0.25, 0.25),
        autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
      front_collision_point_marker.pose.position =
        debug_data_ptr_->obstacles_to_slow_down.at(i).front_collision_point;
      auto back_collision_point_marker = autoware::universe_utils::createDefaultMarker(
        "map", clock_->now(), "slow_down_collision_points", i * 2 + 1, Marker::SPHERE,
        autoware::universe_utils::createMarkerScale(0.25, 0.25, 0.25),
        autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
      back_collision_point_marker.pose.position =
        debug_data_ptr_->obstacles_to_slow_down.at(i).back_collision_point;

      debug_marker.markers.push_back(front_collision_point_marker);
      debug_marker.markers.push_back(back_collision_point_marker);
    }

    // slow down debug wall marker
    autoware::universe_utils::appendMarkerArray(
      debug_data_ptr_->slow_down_debug_wall_marker, &debug_marker);

    debug_slow_down_wall_marker_pub_->publish(debug_data_ptr_->slow_down_wall_marker);
    debug_marker_pub_->publish(debug_marker);

    // slow_down
    const auto slow_down_debug_msg = getSlowDownPlanningDebugMessage(clock_->now());
    debug_slow_down_planning_info_pub_->publish(slow_down_debug_msg);
  }

  void publishVelocityLimit(const std::optional<VelocityLimit> & vel_limit)
  {
    const std::string module_name = "slow_down";

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

  bool use_pointcloud_for_slow_down_;
  bool enable_slow_down_planning_{false};
  std::vector<SlowDownObstacle> prev_slow_down_object_obstacles_;
  std::vector<int> slow_down_obstacle_types_;

  struct DebugData
  {
    DebugData() = default;
    std::vector<SlowDownObstacle> obstacles_to_slow_down;
    MarkerArray slow_down_debug_wall_marker;
    MarkerArray slow_down_wall_marker;
    std::optional<std::vector<Metric>> slow_down_metrics{std::nullopt};
  };

  struct SlowDownOutput
  {
    SlowDownOutput() = default;
    SlowDownOutput(
      const std::string & arg_uuid, const std::vector<TrajectoryPoint> & traj_points,
      const std::optional<size_t> & start_idx, const std::optional<size_t> & end_idx,
      const double arg_target_vel, const double arg_feasible_target_vel,
      const double arg_precise_lat_dist, const bool is_moving)
    : uuid(arg_uuid),
      target_vel(arg_target_vel),
      feasible_target_vel(arg_feasible_target_vel),
      precise_lat_dist(arg_precise_lat_dist),
      is_moving(is_moving)
    {
      if (start_idx) {
        start_point = traj_points.at(*start_idx).pose;
      }
      if (end_idx) {
        end_point = traj_points.at(*end_idx).pose;
      }
    }

    std::string uuid;
    double target_vel;
    double feasible_target_vel;
    double precise_lat_dist;
    std::optional<geometry_msgs::msg::Pose> start_point{std::nullopt};
    std::optional<geometry_msgs::msg::Pose> end_point{std::nullopt};
    bool is_moving;
  };
  std::vector<SlowDownOutput> prev_slow_down_output_;

  struct SlowDownConditionCounter
  {
    void resetCurrentUuids() { current_uuids_.clear(); }
    void addCurrentUuid(const std::string & uuid) { current_uuids_.push_back(uuid); }
    void removeCounterUnlessUpdated()
    {
      std::vector<std::string> obsolete_uuids;
      for (const auto & key_and_value : counter_) {
        if (
          std::find(current_uuids_.begin(), current_uuids_.end(), key_and_value.first) ==
          current_uuids_.end()) {
          obsolete_uuids.push_back(key_and_value.first);
        }
      }

      for (const auto & obsolete_uuid : obsolete_uuids) {
        counter_.erase(obsolete_uuid);
      }
    }

    int increaseCounter(const std::string & uuid)
    {
      if (counter_.count(uuid) != 0) {
        counter_.at(uuid) = std::max(1, counter_.at(uuid) + 1);
      } else {
        counter_.emplace(uuid, 1);
      }
      return counter_.at(uuid);
    }
    int decreaseCounter(const std::string & uuid)
    {
      if (counter_.count(uuid) != 0) {
        counter_.at(uuid) = std::min(-1, counter_.at(uuid) - 1);
      } else {
        counter_.emplace(uuid, -1);
      }
      return counter_.at(uuid);
    }
    void reset(const std::string & uuid) { counter_.emplace(uuid, 0); }

    // NOTE: positive is for meeting entering condition, and negative is for exiting.
    std::unordered_map<std::string, int> counter_;
    std::vector<std::string> current_uuids_;
  };
  SlowDownConditionCounter slow_down_condition_counter_;

  struct SlowDownParam
  {
    double slow_down_min_jerk;
    double slow_down_min_accel;
    double max_lat_margin_for_slow_down;
    double lat_hysteresis_margin_for_slow_down;
    int successive_num_to_entry_slow_down_condition;
    int successive_num_to_exit_slow_down_condition;

    std::vector<std::string> obstacle_labels{"default"};
    std::vector<std::string> obstacle_moving_classification{"static", "moving"};
    std::unordered_map<uint8_t, std::string> types_map;
    struct ObstacleSpecificParams
    {
      double max_lat_margin;
      double min_lat_margin;
      double max_ego_velocity;
      double min_ego_velocity;
    };

    explicit SlowDownParam(rclcpp::Node & node)
    {
      slow_down_min_accel = getOrDeclareParameter<double>(node, "slow_down.slow_down_min_acc");
      slow_down_min_jerk = getOrDeclareParameter<double>(node, "slow_down.slow_down_min_jerk");

      // behavior determination
      max_lat_margin_for_slow_down =
        getOrDeclareParameter<double>(node, "slow_down.behavior_determination.max_lat_margin");
      lat_hysteresis_margin_for_slow_down = getOrDeclareParameter<double>(
        node, "slow_down.behavior_determination.lat_hysteresis_margin");
      successive_num_to_entry_slow_down_condition = getOrDeclareParameter<int>(
        node, "slow_down.behavior_determination.successive_num_to_entry_slow_down_condition");
      successive_num_to_exit_slow_down_condition = getOrDeclareParameter<int>(
        node, "slow_down.behavior_determination.successive_num_to_exit_slow_down_condition");

      obstacle_labels = getOrDeclareParameter<std::vector<std::string>>(node, "slow_down.labels");
      // obstacle label dependant parameters
      for (const auto & label : obstacle_labels) {
        for (const auto & movement_postfix : obstacle_moving_classification) {
          ObstacleSpecificParams params;
          params.max_lat_margin = getOrDeclareParameter<double>(
            node, "slow_down." + label + "." + movement_postfix + ".max_lat_margin");
          params.min_lat_margin = getOrDeclareParameter<double>(
            node, "slow_down." + label + "." + movement_postfix + ".min_lat_margin");
          params.max_ego_velocity = getOrDeclareParameter<double>(
            node, "slow_down." + label + "." + movement_postfix + ".max_ego_velocity");
          params.min_ego_velocity = getOrDeclareParameter<double>(
            node, "slow_down." + label + "." + movement_postfix + ".min_ego_velocity");
          obstacle_to_param_struct_map.emplace(
            std::make_pair(label + "." + movement_postfix, params));
        }
      }

      // common parameters
      time_margin_on_target_velocity =
        getOrDeclareParameter<double>(node, "slow_down.time_margin_on_target_velocity");
      lpf_gain_slow_down_vel =
        getOrDeclareParameter<double>(node, "slow_down.lpf_gain_slow_down_vel");
      lpf_gain_lat_dist = getOrDeclareParameter<double>(node, "slow_down.lpf_gain_lat_dist");
      lpf_gain_dist_to_slow_down =
        getOrDeclareParameter<double>(node, "slow_down.lpf_gain_dist_to_slow_down");

      types_map = {{ObjectClassification::UNKNOWN, "unknown"},
                   {ObjectClassification::CAR, "car"},
                   {ObjectClassification::TRUCK, "truck"},
                   {ObjectClassification::BUS, "bus"},
                   {ObjectClassification::TRAILER, "trailer"},
                   {ObjectClassification::MOTORCYCLE, "motorcycle"},
                   {ObjectClassification::BICYCLE, "bicycle"},
                   {ObjectClassification::PEDESTRIAN, "pedestrian"}};
    }

    ObstacleSpecificParams getObstacleParamByLabel(
      const ObjectClassification & label_id, const bool is_obstacle_moving) const
    {
      const std::string label =
        (types_map.count(label_id.label) > 0) ? types_map.at(label_id.label) : "default";
      const std::string movement_postfix = (is_obstacle_moving) ? "moving" : "static";
      return (obstacle_to_param_struct_map.count(label + "." + movement_postfix) > 0)
               ? obstacle_to_param_struct_map.at(label + "." + movement_postfix)
               : obstacle_to_param_struct_map.at("default." + movement_postfix);
    }

    void onParam(const std::vector<rclcpp::Parameter> & parameters)
    {
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.slow_down_min_accel", slow_down_min_accel);
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.slow_down_min_jerk", slow_down_min_jerk);

      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.behavior_determination.max_lat_margin",
        max_lat_margin_for_slow_down);
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.behavior_determination.lat_hysteresis_margin",
        lat_hysteresis_margin_for_slow_down);
      autoware::universe_utils::updateParam<int>(
        parameters, "slow_down.behavior_determination.successive_num_to_entry_slow_down_condition",
        successive_num_to_entry_slow_down_condition);
      autoware::universe_utils::updateParam<int>(
        parameters, "slow_down.behavior_determination.successive_num_to_exit_slow_down_condition",
        successive_num_to_exit_slow_down_condition);

      // obstacle type dependant parameters
      for (const auto & label : obstacle_labels) {
        for (const auto & movement_postfix : obstacle_moving_classification) {
          if (obstacle_to_param_struct_map.count(label + "." + movement_postfix) < 1) continue;
          auto & param_by_obstacle_label =
            obstacle_to_param_struct_map.at(label + "." + movement_postfix);
          autoware::universe_utils::updateParam<double>(
            parameters, "slow_down." + label + "." + movement_postfix + ".max_lat_margin",
            param_by_obstacle_label.max_lat_margin);
          autoware::universe_utils::updateParam<double>(
            parameters, "slow_down." + label + "." + movement_postfix + ".min_lat_margin",
            param_by_obstacle_label.min_lat_margin);
          autoware::universe_utils::updateParam<double>(
            parameters, "slow_down." + label + "." + movement_postfix + ".max_ego_velocity",
            param_by_obstacle_label.max_ego_velocity);
          autoware::universe_utils::updateParam<double>(
            parameters, "slow_down." + label + "." + movement_postfix + ".min_ego_velocity",
            param_by_obstacle_label.min_ego_velocity);
        }
      }

      // common parameters
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.time_margin_on_target_velocity", time_margin_on_target_velocity);
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.lpf_gain_slow_down_vel", lpf_gain_slow_down_vel);
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.lpf_gain_lat_dist", lpf_gain_lat_dist);
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.lpf_gain_dist_to_slow_down", lpf_gain_dist_to_slow_down);
    }

    std::unordered_map<std::string, ObstacleSpecificParams> obstacle_to_param_struct_map;

    double time_margin_on_target_velocity;
    double lpf_gain_slow_down_vel;
    double lpf_gain_lat_dist;
    double lpf_gain_dist_to_slow_down;
  };

  double moving_object_speed_threshold;
  double moving_object_hysteresis_range;

  bool isSlowDownObstacle(const uint8_t label) const
  {
    const auto & types = slow_down_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  std::optional<SlowDownObstacle> createSlowDownObstacleForPredictedObject(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const PredictedObjectBasedObstacle & obstacle, const double precise_lat_dist,
    const VehicleInfo & vehicle_info)
  {
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & p = slow_down_param_;
    slow_down_condition_counter_.addCurrentUuid(obstacle.uuid);

    const bool is_prev_obstacle_slow_down =
      obstacle_cruise_utils::getObstacleFromUuid(prev_slow_down_object_obstacles_, obstacle.uuid)
        .has_value();

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
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[SlowDown] Ignore obstacle (%s) since it's far from trajectory. (%f [m])",
      //   object_id.c_str(), precise_lat_dist);
      return std::nullopt;
    }

    const auto obstacle_poly = autoware::universe_utils::toPolygon2d(obstacle.pose, obstacle.shape);

    // calculate collision points with trajectory with lateral stop margin
    // NOTE: For additional margin, hysteresis is not divided by two.
    const auto traj_polys_with_lat_margin = obstacle_cruise_utils::createOneStepPolygons(
      traj_points, vehicle_info, odometry.pose.pose,
      p.max_lat_margin_for_slow_down + p.lat_hysteresis_margin_for_slow_down,
      common_behavior_determination_param_);

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
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[SlowDown] Ignore obstacle (%s) since there is no collision point", object_id.c_str());
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

  std::optional<SlowDownObstacle> createSlowDownObstacleForPointCloud(
    const PredictedObjectBasedObstacle & obstacle, const double precise_lat_dist)
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

  std::optional<std::tuple<double, double, double>> calculateDistanceToSlowDownWithConstraints(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points,
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
    const double dist_to_ego, const bool is_obstacle_moving, const VehicleInfo & vehicle_info) const
  {
    const double abs_ego_offset = planner_data.is_driving_forward
                                    ? std::abs(vehicle_info.max_longitudinal_offset_m)
                                    : std::abs(vehicle_info.min_longitudinal_offset_m);
    const double obstacle_vel = obstacle.velocity;
    // calculate slow down velocity
    const double slow_down_vel =
      calculateSlowDownVelocity(obstacle, prev_output, is_obstacle_moving);

    // calculate distance to collision points
    const double dist_to_front_collision =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, obstacle.front_collision_point);
    const double dist_to_back_collision =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, obstacle.back_collision_point);

    // calculate offset distance to first collision considering relative velocity
    const double relative_vel =
      planner_data.current_odometry.twist.twist.linear.x - obstacle.velocity;
    const double offset_dist_to_collision = [&]() {
      if (dist_to_front_collision < dist_to_ego + abs_ego_offset) {
        return 0.0;
      }

      // NOTE: This min_relative_vel forces the relative velocity positive if the ego velocity is
      // lower than the obstacle velocity. Without this, the slow down feature will flicker where
      // the ego velocity is very close to the obstacle velocity.
      constexpr double min_relative_vel = 1.0;
      const double time_to_collision = (dist_to_front_collision - dist_to_ego - abs_ego_offset) /
                                       std::max(min_relative_vel, relative_vel);

      constexpr double time_to_collision_margin = 1.0;
      const double cropped_time_to_collision =
        std::max(0.0, time_to_collision - time_to_collision_margin);
      return obstacle_vel * cropped_time_to_collision;
    }();

    // calculate distance during deceleration, slow down preparation, and slow down
    const double min_slow_down_prepare_dist = 3.0;
    const double slow_down_prepare_dist = std::max(
      min_slow_down_prepare_dist, slow_down_vel * slow_down_param_.time_margin_on_target_velocity);
    const double deceleration_dist = offset_dist_to_collision + dist_to_front_collision -
                                     abs_ego_offset - dist_to_ego - slow_down_prepare_dist;
    const double slow_down_dist =
      dist_to_back_collision - dist_to_front_collision + slow_down_prepare_dist;

    // calculate distance to start/end slow down
    const double dist_to_slow_down_start = dist_to_ego + deceleration_dist;
    const double dist_to_slow_down_end = dist_to_ego + deceleration_dist + slow_down_dist;
    if (100.0 < dist_to_slow_down_start) {
      // NOTE: distance to slow down is too far.
      return std::nullopt;
    }

    // apply low-pass filter to distance to start/end slow down
    const auto apply_lowpass_filter = [&](const double dist_to_slow_down, const auto prev_point) {
      if (prev_output && prev_point) {
        const size_t seg_idx =
          autoware::motion_utils::findNearestSegmentIndex(traj_points, prev_point->position);
        const double prev_dist_to_slow_down = autoware::motion_utils::calcSignedArcLength(
          traj_points, 0, prev_point->position, seg_idx);
        return autoware::signal_processing::lowpassFilter(
          dist_to_slow_down, prev_dist_to_slow_down, slow_down_param_.lpf_gain_dist_to_slow_down);
      }
      return dist_to_slow_down;
    };
    const double filtered_dist_to_slow_down_start =
      apply_lowpass_filter(dist_to_slow_down_start, prev_output->start_point);
    const double filtered_dist_to_slow_down_end =
      apply_lowpass_filter(dist_to_slow_down_end, prev_output->end_point);

    // calculate velocity considering constraints
    const double feasible_slow_down_vel = [&]() {
      if (deceleration_dist < 0) {
        if (prev_output) {
          return prev_output->target_vel;
        }
        return std::max(planner_data.current_odometry.twist.twist.linear.x, slow_down_vel);
      }
      if (planner_data.current_odometry.twist.twist.linear.x < slow_down_vel) {
        return slow_down_vel;
      }

      const double one_shot_feasible_slow_down_vel = [&]() {
        if (planner_data.current_acceleration.accel.accel.linear.x < longitudinal_info_.min_accel) {
          const double squared_vel =
            std::pow(planner_data.current_odometry.twist.twist.linear.x, 2) +
            2 * deceleration_dist * longitudinal_info_.min_accel;
          if (squared_vel < 0) {
            return slow_down_vel;
          }
          return std::max(std::sqrt(squared_vel), slow_down_vel);
        }
        // TODO(murooka) Calculate more precisely. Final acceleration should be zero.
        const double min_feasible_slow_down_vel = calcDecelerationVelocityFromDistanceToTarget(
          slow_down_param_.slow_down_min_jerk, slow_down_param_.slow_down_min_accel,
          planner_data.current_acceleration.accel.accel.linear.x,
          planner_data.current_odometry.twist.twist.linear.x, deceleration_dist);
        return min_feasible_slow_down_vel;
      }();
      if (prev_output) {
        // NOTE: If longitudinal controllability is not good, one_shot_slow_down_vel may be getting
        // larger since we use actual ego's velocity and acceleration for its calculation.
        //       Suppress one_shot_slow_down_vel getting larger here.
        const double feasible_slow_down_vel =
          std::min(one_shot_feasible_slow_down_vel, prev_output->feasible_target_vel);
        return std::max(slow_down_vel, feasible_slow_down_vel);
      }
      return std::max(slow_down_vel, one_shot_feasible_slow_down_vel);
    }();

    return std::make_tuple(
      filtered_dist_to_slow_down_start, filtered_dist_to_slow_down_end, feasible_slow_down_vel);
  }

  double calculateSlowDownVelocity(
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
    const bool is_obstacle_moving) const
  {
    const auto & p =
      slow_down_param_.getObstacleParamByLabel(obstacle.classification, is_obstacle_moving);
    const double stable_precise_lat_dist = [&]() {
      if (prev_output) {
        return autoware::signal_processing::lowpassFilter(
          obstacle.precise_lat_dist, prev_output->precise_lat_dist,
          slow_down_param_.lpf_gain_lat_dist);
      }
      return obstacle.precise_lat_dist;
    }();

    const double ratio = std::clamp(
      (std::abs(stable_precise_lat_dist) - p.min_lat_margin) /
        (p.max_lat_margin - p.min_lat_margin),
      0.0, 1.0);
    const double slow_down_vel =
      p.min_ego_velocity + ratio * (p.max_ego_velocity - p.min_ego_velocity);

    return slow_down_vel;
  }

  Float32MultiArrayStamped slow_down_debug_multi_array_;

  rclcpp::Publisher<MetricArray>::SharedPtr metrics_pub_;
  rclcpp::Publisher<VelocityFactorArray>::SharedPtr velocity_factors_pub_;

  rclcpp::Clock::SharedPtr clock_;
  SlowDownParam slow_down_param_;
  LongitudinalInfo longitudinal_info_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;

  // Parameters
  bool enable_debug_info_{false};
  bool use_pointcloud_{false};

  void updateCommonParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    longitudinal_info_.onParam(parameters);
  }

  std::unique_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>
    objects_of_interest_marker_interface_;
  CommonBehaviorDeterminationParam common_behavior_determination_param_;

  rclcpp::Publisher<MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_slow_down_wall_marker_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_slow_down_planning_info_pub_;

  rclcpp::Publisher<VelocityLimit>::SharedPtr vel_limit_pub_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr clear_vel_limit_pub_;

  std::unordered_map<std::string, bool> need_to_clear_vel_limit_{{"slow_down", false}};
};
}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__SLOW_DOWN__OBSTACLE_SLOW_DOWN_MODULE_HPP_
