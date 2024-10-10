// Copyright 2023 TIER IV, Inc.
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
#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__PARAMETER_HELPER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__PARAMETER_HELPER_HPP_

#include "autoware/universe_utils/ros/parameter.hpp"

#include <autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp>
#include <rclcpp/node.hpp>

#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>

#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::universe_utils::getOrDeclareParameter;
using autoware_perception_msgs::msg::ObjectClassification;

AvoidanceParameters getParameter(
  std::shared_ptr<::static_obstacle_avoidance::ParamListener> param_listener)
{
  AvoidanceParameters p;
  p = param_listener->get_params();

  // target object
  {
    const auto get_object_param = [&](const auto & object_param) {
      ObjectParameter param{};
      param.moving_speed_threshold = object_param.th_moving_speed;
      param.moving_time_threshold = object_param.th_moving_time;
      param.max_expand_ratio = object_param.max_expand_ratio;
      param.envelope_buffer_margin = object_param.envelope_buffer_margin;
      param.lateral_soft_margin = object_param.lateral_margin.soft_margin;
      param.lateral_hard_margin = object_param.lateral_margin.hard_margin;
      param.lateral_hard_margin_for_parked_vehicle =
        object_param.lateral_margin.hard_margin_for_parked_vehicle;
      param.longitudinal_margin = object_param.longitudinal_margin;
      param.th_error_eclipse_long_radius = object_param.th_error_eclipse_long_radius;
      return param;
    };

    p.object_parameters.emplace(
      ObjectClassification::MOTORCYCLE, get_object_param(p.target_object.motorcycle));
    p.object_parameters.emplace(ObjectClassification::CAR, get_object_param(p.target_object.car));
    p.object_parameters.emplace(
      ObjectClassification::TRUCK, get_object_param(p.target_object.truck));
    p.object_parameters.emplace(
      ObjectClassification::TRAILER, get_object_param(p.target_object.trailer));
    p.object_parameters.emplace(ObjectClassification::BUS, get_object_param(p.target_object.bus));
    p.object_parameters.emplace(
      ObjectClassification::PEDESTRIAN, get_object_param(p.target_object.pedestrian));
    p.object_parameters.emplace(
      ObjectClassification::BICYCLE, get_object_param(p.target_object.bicycle));
    p.object_parameters.emplace(
      ObjectClassification::UNKNOWN, get_object_param(p.target_object.unknown));
  }

  // target filtering
  {
    const auto set_target_flag = [&](
                                   const uint8_t & object_type, const bool & is_avoidance_target) {
      if (p.object_parameters.count(object_type) == 0) {
        return;
      }
      p.object_parameters.at(object_type).is_avoidance_target = is_avoidance_target;
    };

    set_target_flag(ObjectClassification::CAR, p.target_filtering.target_type.car);
    set_target_flag(ObjectClassification::TRUCK, p.target_filtering.target_type.truck);
    set_target_flag(ObjectClassification::TRAILER, p.target_filtering.target_type.trailer);
    set_target_flag(ObjectClassification::BUS, p.target_filtering.target_type.bus);
    set_target_flag(ObjectClassification::PEDESTRIAN, p.target_filtering.target_type.pedestrian);
    set_target_flag(ObjectClassification::BICYCLE, p.target_filtering.target_type.bicycle);
    set_target_flag(ObjectClassification::MOTORCYCLE, p.target_filtering.target_type.motorcycle);
    set_target_flag(ObjectClassification::UNKNOWN, p.target_filtering.target_type.unknown);
  }

  // safety check general params
  {
    const auto set_target_flag =
      [&](const uint8_t & object_type, const bool & is_safety_check_target) {
        if (p.object_parameters.count(object_type) == 0) {
          return;
        }
        p.object_parameters.at(object_type).is_safety_check_target = is_safety_check_target;
      };

    set_target_flag(ObjectClassification::CAR, p.safety_check.target_type.car);
    set_target_flag(ObjectClassification::TRUCK, p.safety_check.target_type.truck);
    set_target_flag(ObjectClassification::TRAILER, p.safety_check.target_type.trailer);
    set_target_flag(ObjectClassification::BUS, p.safety_check.target_type.bus);
    set_target_flag(ObjectClassification::PEDESTRIAN, p.safety_check.target_type.pedestrian);
    set_target_flag(ObjectClassification::BICYCLE, p.safety_check.target_type.bicycle);
    set_target_flag(ObjectClassification::MOTORCYCLE, p.safety_check.target_type.motorcycle);
    set_target_flag(ObjectClassification::UNKNOWN, p.safety_check.target_type.unknown);
  }

  // safety check predicted path params
  {
    p.ego_predicted_path_params.min_velocity = p.safety_check.min_velocity;
    p.ego_predicted_path_params.acceleration = p.constraints.longitudinal.max_acceleration;
    p.ego_predicted_path_params.time_horizon_for_front_object =
      p.safety_check.time_horizon_for_front_object;
    p.ego_predicted_path_params.time_horizon_for_rear_object =
      p.safety_check.time_horizon_for_rear_object;
    p.ego_predicted_path_params.time_resolution = p.safety_check.time_resolution;
    p.ego_predicted_path_params.delay_until_departure = p.safety_check.delay_until_departure;
  }

  // safety check rss params
  {
    p.rss_params.extended_polygon_policy = p.safety_check.extended_polygon_policy;
    p.rss_params.longitudinal_distance_min_threshold =
      p.safety_check.longitudinal_distance_min_threshold;
    p.rss_params.longitudinal_velocity_delta_time = p.safety_check.longitudinal_velocity_delta_time;
    p.rss_params.front_vehicle_deceleration = p.safety_check.expected_front_deceleration;
    p.rss_params.rear_vehicle_deceleration = p.safety_check.expected_rear_deceleration;
    p.rss_params.rear_vehicle_reaction_time = p.safety_check.rear_vehicle_reaction_time;
    p.rss_params.rear_vehicle_safety_time_margin = p.safety_check.rear_vehicle_safety_time_margin;
    p.rss_params.lateral_distance_max_threshold = p.safety_check.lateral_distance_max_threshold;
  }

  return p;
}
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__PARAMETER_HELPER_HPP_
