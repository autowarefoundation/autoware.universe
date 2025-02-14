// Copyright 2025 TIER IV, Inc.
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

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/universe_utils/ros/parameter.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware::universe_utils::getOrDeclareParameter;

struct CommonParam
{
  double max_accel{};
  double min_accel{};
  double max_jerk{};
  double min_jerk{};
  double limit_max_accel{};
  double limit_min_accel{};
  double limit_max_jerk{};
  double limit_min_jerk{};

  CommonParam() = default;
  explicit CommonParam(rclcpp::Node & node)
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
};

struct ObstacleFilteringParam
{
  std::vector<uint8_t> object_types{};

  double min_lat_margin{};
  double max_lat_margin{};
  double lat_hysteresis_margin{};

  int successive_num_to_entry_slow_down_condition{};
  int successive_num_to_exit_slow_down_condition{};

  ObstacleFilteringParam() = default;
  explicit ObstacleFilteringParam(rclcpp::Node & node)
  {
    object_types =
      utils::get_target_object_type(node, "obstacle_slow_down.obstacle_filtering.object_type.");
    min_lat_margin =
      getOrDeclareParameter<double>(node, "obstacle_slow_down.obstacle_filtering.min_lat_margin");
    max_lat_margin =
      getOrDeclareParameter<double>(node, "obstacle_slow_down.obstacle_filtering.max_lat_margin");
    lat_hysteresis_margin = getOrDeclareParameter<double>(
      node, "obstacle_slow_down.obstacle_filtering.lat_hysteresis_margin");
    successive_num_to_entry_slow_down_condition = getOrDeclareParameter<int>(
      node, "obstacle_slow_down.obstacle_filtering.successive_num_to_entry_slow_down_condition");
    successive_num_to_exit_slow_down_condition = getOrDeclareParameter<int>(
      node, "obstacle_slow_down.obstacle_filtering.successive_num_to_exit_slow_down_condition");
  }
};

struct SlowDownPlanningParam
{
  double slow_down_min_acc{};
  double slow_down_min_jerk{};

  double lpf_gain_slow_down_vel{};
  double lpf_gain_lat_dist{};
  double lpf_gain_dist_to_slow_down{};

  double time_margin_on_target_velocity{};

  double moving_object_speed_threshold{};
  double moving_object_hysteresis_range{};

  std::vector<std::string> obstacle_labels{"default"};
  std::vector<std::string> obstacle_moving_classification{"static", "moving"};
  struct ObjectTypeSpecificParams
  {
    double min_lat_margin;
    double max_lat_margin;
    double min_ego_velocity;
    double max_ego_velocity;
  };
  std::unordered_map<uint8_t, std::string> object_types_maps = {
    {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
    {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
    {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
    {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};
  std::unordered_map<std::string, ObjectTypeSpecificParams> object_type_specific_param_map;

  SlowDownPlanningParam() = default;
  explicit SlowDownPlanningParam(rclcpp::Node & node)
  {
    slow_down_min_acc = getOrDeclareParameter<double>(
      node, "obstacle_slow_down.slow_down_planning.slow_down_min_acc");
    slow_down_min_jerk = getOrDeclareParameter<double>(
      node, "obstacle_slow_down.slow_down_planning.slow_down_min_jerk");

    lpf_gain_slow_down_vel = getOrDeclareParameter<double>(
      node, "obstacle_slow_down.slow_down_planning.lpf_gain_slow_down_vel");
    lpf_gain_lat_dist = getOrDeclareParameter<double>(
      node, "obstacle_slow_down.slow_down_planning.lpf_gain_lat_dist");
    lpf_gain_dist_to_slow_down = getOrDeclareParameter<double>(
      node, "obstacle_slow_down.slow_down_planning.lpf_gain_dist_to_slow_down");
    time_margin_on_target_velocity = getOrDeclareParameter<double>(
      node, "obstacle_slow_down.slow_down_planning.time_margin_on_target_velocity");

    moving_object_speed_threshold = getOrDeclareParameter<double>(
      node, "obstacle_slow_down.slow_down_planning.moving_object_speed_threshold");
    moving_object_hysteresis_range = getOrDeclareParameter<double>(
      node, "obstacle_slow_down.slow_down_planning.moving_object_hysteresis_range");

    const std::string param_prefix =
      "obstacle_slow_down.slow_down_planning.object_type_specified_params.";
    const auto object_types =
      getOrDeclareParameter<std::vector<std::string>>(node, param_prefix + "types");

    for (const auto & type_str : object_types) {
      for (const auto & movement_type : std::vector<std::string>{"moving", "static"}) {
        ObjectTypeSpecificParams param{
          getOrDeclareParameter<double>(
            node, param_prefix + type_str + "." + movement_type + ".min_lat_margin"),
          getOrDeclareParameter<double>(
            node, param_prefix + type_str + "." + movement_type + ".max_lat_margin"),
          getOrDeclareParameter<double>(
            node, param_prefix + type_str + "." + movement_type + ".min_ego_velocity"),
          getOrDeclareParameter<double>(
            node, param_prefix + type_str + "." + movement_type + ".max_ego_velocity")};

        object_type_specific_param_map.emplace(type_str + "." + movement_type, param);
      }
    }
  }

  std::string get_param_type(const ObjectClassification label) const
  {
    const auto type_str = object_types_maps.at(label.label);
    if (object_type_specific_param_map.count(type_str) == 0) {
      return "default";
    }
    return type_str;
  }
  ObjectTypeSpecificParams get_object_param_by_label(
    const ObjectClassification label, const bool is_obstacle_moving) const
  {
    const std::string movement_type = is_obstacle_moving ? "moving" : "static";
    return object_type_specific_param_map.at(get_param_type(label) + "." + movement_type);
  }
};
}  // namespace autoware::motion_velocity_planner

#endif  // PARAMETERS_HPP_
