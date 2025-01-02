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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__STOP__TYPES_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__STOP__TYPES_HPP_

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"
#include "autoware/obstacle_cruise_planner/stop/stop_planning_debug_info.hpp"
#include "autoware/obstacle_cruise_planner/stop/type_alias.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"
#include "autoware/universe_utils/ros/parameter.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_planning
{
using autoware::universe_utils::getOrDeclareParameter;

struct StopLongitudinalInfo
{
  explicit StopLongitudinalInfo(rclcpp::Node & node)
  {
    max_accel = getOrDeclareParameter<double>(node, "normal.max_acc");
    min_accel = getOrDeclareParameter<double>(node, "normal.min_acc");
    max_jerk = getOrDeclareParameter<double>(node, "normal.max_jerk");
    min_jerk = getOrDeclareParameter<double>(node, "normal.min_jerk");
    limit_max_accel = getOrDeclareParameter<double>(node, "limit.max_acc");
    limit_min_accel = getOrDeclareParameter<double>(node, "limit.min_acc");
    limit_max_jerk = getOrDeclareParameter<double>(node, "limit.max_jerk");
    limit_min_jerk = getOrDeclareParameter<double>(node, "limit.min_jerk");

    safe_distance_margin = getOrDeclareParameter<double>(node, "stop.safe_distance_margin");
    terminal_safe_distance_margin =
      getOrDeclareParameter<double>(node, "stop.terminal_safe_distance_margin");
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
      parameters, "stop.safe_distance_margin", safe_distance_margin);
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.terminal_safe_distance_margin", terminal_safe_distance_margin);
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

  // distance margin
  double safe_distance_margin;
  double terminal_safe_distance_margin;
};

struct StopObstacle
{
  StopObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const ObjectClassification & object_classification, const geometry_msgs::msg::Pose & arg_pose,
    const Shape & arg_shape, const double arg_lon_velocity,
    const geometry_msgs::msg::Point arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_lon_velocity),
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

  Shape shape;
  geometry_msgs::msg::Point
    collision_point;  // TODO(yuki_takagi): this member variable still used in
                      // calculateMarginFromObstacleOnCurve() and  should be removed as it can be
                      // replaced by ”dist_to_collide_on_decimated_traj”
  double dist_to_collide_on_decimated_traj;
  ObjectClassification classification;
};

struct DebugData
{
  DebugData() = default;
  std::vector<PredictedObjectBasedObstacle> intentionally_ignored_obstacles;
  std::vector<StopObstacle> obstacles_to_stop;
  MarkerArray stop_wall_marker;
};

struct StopParam
{
  double hold_stop_velocity_threshold;
  double hold_stop_distance_threshold;
  double collision_time_margin;
  double max_lat_margin_for_stop;
  double max_lat_margin_for_stop_against_unknown;
  double min_velocity_to_reach_collision_point;
  double max_lat_time_margin_for_stop;
  int num_of_predicted_paths_for_outside_stop_obstacle;
  double pedestrian_deceleration_rate;
  double bicycle_deceleration_rate;
  double stop_obstacle_hold_time_threshold;
  double obstacle_velocity_threshold_to_stop;
  double obstacle_velocity_threshold_from_stop;

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

  explicit StopParam(rclcpp::Node & node, const StopLongitudinalInfo & longitudinal_info)
  {
    hold_stop_velocity_threshold =
      getOrDeclareParameter<double>(node, "stop.hold_stop_velocity_threshold");
    hold_stop_distance_threshold =
      getOrDeclareParameter<double>(node, "stop.hold_stop_distance_threshold");

    // behavior determination
    collision_time_margin = getOrDeclareParameter<double>(
      node, "stop.obstacle_filtering.crossing_obstacle.collision_time_margin");
    max_lat_margin_for_stop =
      getOrDeclareParameter<double>(node, "stop.obstacle_filtering.max_lat_margin");
    max_lat_margin_for_stop_against_unknown =
      getOrDeclareParameter<double>(node, "stop.obstacle_filtering.max_lat_margin_against_unknown");
    min_velocity_to_reach_collision_point = getOrDeclareParameter<double>(
      node, "stop.obstacle_filtering.min_velocity_to_reach_collision_point");
    max_lat_time_margin_for_stop = getOrDeclareParameter<double>(
      node, "stop.obstacle_filtering.outside_obstacle.max_lateral_time_margin");
    num_of_predicted_paths_for_outside_stop_obstacle = getOrDeclareParameter<int>(
      node, "stop.obstacle_filtering.outside_obstacle.num_of_predicted_paths");
    pedestrian_deceleration_rate = getOrDeclareParameter<double>(
      node, "stop.obstacle_filtering.outside_obstacle.pedestrian_deceleration_rate");
    bicycle_deceleration_rate = getOrDeclareParameter<double>(
      node, "stop.obstacle_filtering.outside_obstacle.bicycle_deceleration_rate");
    stop_obstacle_hold_time_threshold = getOrDeclareParameter<double>(
      node, "stop.obstacle_filtering.stop_obstacle_hold_time_threshold");
    obstacle_velocity_threshold_to_stop = getOrDeclareParameter<double>(
      node, "stop.obstacle_filtering.obstacle_velocity_threshold_to_stop");
    obstacle_velocity_threshold_from_stop = getOrDeclareParameter<double>(
      node, "stop.obstacle_filtering.obstacle_velocity_threshold_from_stop");

    const std::string param_prefix = "stop.type_specified_params.";
    std::vector<std::string> obstacle_labels{"default"};
    obstacle_labels =
      getOrDeclareParameter<std::vector<std::string>>(node, param_prefix + "labels");

    for (const auto & type_str : obstacle_labels) {
      if (type_str != "default") {
        ObstacleSpecificParams param{
          getOrDeclareParameter<double>(node, param_prefix + type_str + ".limit_min_acc"),
          getOrDeclareParameter<double>(
            node, param_prefix + type_str + ".sudden_object_acc_threshold"),
          getOrDeclareParameter<double>(
            node, param_prefix + type_str + ".sudden_object_dist_threshold"),
          getOrDeclareParameter<bool>(node, param_prefix + type_str + ".abandon_to_stop")};

        param.sudden_object_acc_threshold =
          std::min(param.sudden_object_acc_threshold, longitudinal_info.limit_min_accel);
        param.limit_min_acc = std::min(param.limit_min_acc, param.sudden_object_acc_threshold);

        type_specified_param_list.emplace(type_str, param);
      }
    }
  }

  void onParam(
    const std::vector<rclcpp::Parameter> & parameters,
    const StopLongitudinalInfo & longitudinal_info)
  {
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.hold_stop_velocity_threshold", hold_stop_velocity_threshold);
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.hold_stop_distance_threshold", hold_stop_distance_threshold);

    autoware::universe_utils::updateParam<double>(
      parameters, "stop.obstacle_filtering.crossing_obstacle.collision_time_margin",
      collision_time_margin);
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.obstacle_filtering.max_lat_margin", max_lat_margin_for_stop);
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.obstacle_filtering.max_lat_margin_against_unknown",
      max_lat_margin_for_stop_against_unknown);
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.obstacle_filtering.min_velocity_to_reach_collision_point",
      min_velocity_to_reach_collision_point);
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.obstacle_filtering.outside_obstacle.max_lateral_time_margin",
      max_lat_time_margin_for_stop);
    autoware::universe_utils::updateParam<int>(
      parameters, "stop.obstacle_filtering.outside_obstacle.num_of_predicted_paths",
      num_of_predicted_paths_for_outside_stop_obstacle);
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.obstacle_filtering.outside_obstacle.pedestrian_deceleration_rate",
      pedestrian_deceleration_rate);
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.obstacle_filtering.outside_obstacle.bicycle_deceleration_rate",
      bicycle_deceleration_rate);
    autoware::universe_utils::updateParam<double>(
      parameters, "stop.obstacle_filtering_obstacle_hold_time_threshold",
      stop_obstacle_hold_time_threshold);

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

}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__STOP__TYPES_HPP_
