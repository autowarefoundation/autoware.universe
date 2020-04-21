/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <lane_change_planner/state/common_functions.h>
#include <lane_change_planner/utilities.h>
#include <lanelet2_extension/utility/utilities.h>

namespace lane_change_planner
{
namespace state_machine
{
namespace common_functions
{
bool isLaneChangePathSafe(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & dynamic_objects,
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Twist & current_twist,
  const LaneChangerParameters & ros_parameters, const bool use_buffer)
{
  if (path.points.empty()) {
    return false;
  }
  if (target_lanes.empty() || current_lanes.empty()) {
    return false;
  }
  if (dynamic_objects == nullptr) {
    return true;
  }
  const auto arc = lanelet::utils::getArcCoordinates(current_lanes, current_pose);
  constexpr double check_distance = 100.0;
  const auto target_lane_object_indices =
    util::filterObjectsByLanelets(*dynamic_objects, target_lanes);
  const auto current_lane_object_indices = util::filterObjectsByLanelets(
    *dynamic_objects, current_lanes, arc.length, arc.length + check_distance);

  const double min_thresh = ros_parameters.min_stop_distance;
  const double stop_time = ros_parameters.stop_time;
  const double vehicle_width = ros_parameters.vehicle_width;
  double buffer;
  double lateral_buffer;
  if (use_buffer) {
    buffer = ros_parameters.hysteresis_buffer_distance;
    lateral_buffer = 1.0;
  } else {
    buffer = 0.0;
    lateral_buffer = 1.0;
  }

  const double time_resolution = ros_parameters.prediction_time_resolution;
  const double prediction_duration = ros_parameters.prediction_duration;
  const double current_lane_check_start_time = 0.0;
  const double current_lane_check_end_time =
    ros_parameters.lane_change_prepare_duration + 0.75 * ros_parameters.lane_changing_duration;
  const double target_lane_check_start_time = 0.0;
  const double target_lane_check_end_time =
    ros_parameters.lane_change_prepare_duration + ros_parameters.lane_changing_duration;

  const auto & vehicle_predicted_path =
    util::convertToPredictedPath(path, current_twist, current_pose);

  for (const auto & i : current_lane_object_indices) {
    const auto & obj = dynamic_objects->objects.at(i);
    for (const auto & obj_path : obj.state.predicted_paths) {
      double distance = util::getDistanceBetweenPredictedPaths(
        obj_path, vehicle_predicted_path, current_lane_check_start_time,
        current_lane_check_end_time, time_resolution, true, vehicle_width + lateral_buffer);
      double thresh = util::l2Norm(obj.state.twist_covariance.twist.linear) * stop_time;
      thresh = std::max(thresh, min_thresh);
      thresh += buffer;
      if (distance < thresh) {
        return false;
      }
    }
  }

  for (const auto & i : target_lane_object_indices) {
    const auto & obj = dynamic_objects->objects.at(i);
    for (const auto & obj_path : obj.state.predicted_paths) {
      double distance = util::getDistanceBetweenPredictedPaths(
        obj_path, vehicle_predicted_path, target_lane_check_start_time, target_lane_check_end_time,
        time_resolution, false, 0.0);
      double thresh = util::l2Norm(obj.state.twist_covariance.twist.linear) * stop_time;
      thresh = std::max(thresh, min_thresh);
      thresh += buffer;
      if (distance < thresh) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace common_functions
}  // namespace state_machine
}  // namespace lane_change_planner
