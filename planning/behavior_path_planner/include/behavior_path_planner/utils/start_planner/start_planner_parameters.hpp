
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__START_PLANNER__START_PLANNER_PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__START_PLANNER__START_PLANNER_PARAMETERS_HPP_

#include "behavior_path_planner/utils/geometric_parallel_parking/geometric_parallel_parking.hpp"

#include <string>
#include <vector>

namespace behavior_path_planner
{
struct StartPlannerParameters
{
  double th_arrived_distance;
  double th_stopped_velocity;
  double th_stopped_time;
  double th_turn_signal_on_lateral_offset;
  double intersection_search_length;
  double length_ratio_for_turn_signal_deactivation_near_intersection;
  double collision_check_margin;
  double collision_check_distance_from_end;
  double th_moving_object_velocity;
  // shift pull out
  bool enable_shift_pull_out;
  bool check_shift_path_lane_departure;
  double minimum_shift_pull_out_distance;
  int lateral_acceleration_sampling_num;
  double lateral_jerk;
  double maximum_lateral_acc;
  double minimum_lateral_acc;
  double maximum_curvature;  // maximum curvature considered in the path generation
  double deceleration_interval;
  // geometric pull out
  bool enable_geometric_pull_out;
  bool divide_pull_out_path;
  ParallelParkingParameters parallel_parking_parameters;
  // search start pose backward
  std::string search_priority;  // "efficient_path" or "short_back_distance"
  bool enable_back;
  double backward_velocity;
  double max_back_distance;
  double backward_search_resolution;
  double backward_path_update_duration;
  double ignore_distance_from_lane_end;
  struct TargetFiltering
  {
    double object_check_forward_distance;
    double object_check_backward_distance;
    struct ObjectTypesToCheck
    {
      bool check_car;
      bool check_truck;
      bool check_bus;
      bool check_trailer;
      bool check_bicycle;
      bool check_motorcycle;
      bool check_pedestrian;
      bool check_unknown;
    } object_types_to_check;

    struct ObjectLaneConfiguration
    {
      bool check_current_lane;
      bool check_right_lane;
      bool check_left_lane;
      bool check_shoulder_lane;
      bool check_other_lane;
    } object_lane_configuration;

    bool include_opposite_lane;
    bool invert_opposite_lane;
    bool check_all_predicted_path;
    bool use_all_predicted_path;
    bool use_predicted_path_outside_lanelet;
  } target_filtering;

  double shift_pull_out_velocity;
  double acceleration_to_target_velocity;
  double prediction_time_horizon;
  double prediction_time_resolution;

  struct SafetyCheck
  {
    double stop_time_before_departure;
    double prediction_time_horizon;
    double prediction_time_resolution;
    bool enable_safety_check;
  } safety_check;

  // Common parameters
  double backward_path_length;
  double forward_path_length;

  struct RssParams
  {
    double rear_vehicle_reaction_time;
    double rear_vehicle_safety_time_margin;
    double lateral_distance_max_threshold;
    double longitudinal_distance_min_threshold;
    double longitudinal_velocity_delta_time;
  } rss_params;

  bool publish_debug_marker;
  // safety check
  // double stop_time_before_departure;  // stop time before departure which is used to create ego's
  //                                     // predicted path
  // double acceleration_to_target_velocity;  // acceleration to target velocity which is used to
  //                                          // create ego's predicted path
  // double prediction_time_resolution;       // resolution of predicted path
  // bool enable_safety_check;                // flag to enable safety check
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__START_PLANNER__START_PLANNER_PARAMETERS_HPP_
