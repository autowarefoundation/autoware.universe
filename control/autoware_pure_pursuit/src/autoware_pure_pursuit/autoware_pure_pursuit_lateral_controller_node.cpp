// Copyright 2020-2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp"

#include "autoware/pure_pursuit/util/planning_utils.hpp"
#include "autoware/pure_pursuit/util/tf_utils.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>

namespace autoware::pure_pursuit
{
PurePursuitLateralController::PurePursuitLateralController(rclcpp::Node & node)
: clock_(node.get_clock()),
  logger_(node.get_logger().get_child("lateral_controller")),
  tf_buffer_(clock_),
  tf_listener_(tf_buffer_)
{
  // Vehicle Parameters
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  param_.ld_velocity_ratio = node.declare_parameter<double>("ld_velocity_ratio");
  param_.ld_lateral_error_ratio = node.declare_parameter<double>("ld_lateral_error_ratio");
  param_.ld_curvature_ratio = node.declare_parameter<double>("ld_curvature_ratio");
  param_.long_ld_lateral_error_threshold =
    node.declare_parameter<double>("long_ld_lateral_error_threshold");
  param_.min_lookahead_distance = node.declare_parameter<double>("min_lookahead_distance");
  param_.max_lookahead_distance = node.declare_parameter<double>("max_lookahead_distance");
  param_.reverse_min_lookahead_distance =
    node.declare_parameter<double>("reverse_min_lookahead_distance");
  param_.converged_steer_rad_ = node.declare_parameter<double>("converged_steer_rad");
  param_.prediction_ds = node.declare_parameter<double>("prediction_ds");
  param_.prediction_distance_length = node.declare_parameter<double>("prediction_distance_length");
  param_.resampling_ds = node.declare_parameter<double>("resampling_ds");
  param_.curvature_calculation_distance =
    node.declare_parameter<double>("curvature_calculation_distance");
  param_.enable_path_smoothing = node.declare_parameter<bool>("enable_path_smoothing");
  param_.path_filter_moving_ave_num = node.declare_parameter<int64_t>("path_filter_moving_ave_num");

  // Debug Publishers
  pub_debug_marker_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 0);
  pub_debug_values_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
      "~/debug/ld_outputs", rclcpp::QoS{1});

  // Publish predicted trajectory
  pub_predicted_trajectory_ = node.create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/predicted_trajectory", 1);
}

bool PurePursuitLateralController::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

LateralOutput PurePursuitLateralController::run(const InputData & input_data)
{
}

void PurePursuitLateralController::writePurePursuitFailure(
  const PurePursuit::Failure & failure) const
{
  using Reason = PurePursuit::Failure::Reason;
  const auto logger = rclcpp::get_logger("pure_pursuit");
  switch (failure.reason) {
    case Reason::data_not_ready:
      break;
    case Reason::closest_waypoint_not_found:
      RCLCPP_WARN(
        logger, "cannot find, curr_bool: %d, closest_idx: %d", false, failure.closest_index);
      break;
    case Reason::lane_direction_unknown:
      RCLCPP_ERROR(rclcpp::get_logger(PLANNING_UTILS_LOGGER), "lane is something wrong");
      RCLCPP_WARN(logger, "lost next waypoint");
      break;
    case Reason::next_waypoint_not_found:
      RCLCPP_WARN(logger, "lost next waypoint");
      break;
    case Reason::waypoint_interval_too_short:
      RCLCPP_ERROR(logger, "waypoint interval is almost 0");
      RCLCPP_WARN(logger, "lost target! ");
      break;
    case Reason::lateral_error_beyond_lookahead:
      RCLCPP_ERROR(logger, "lateral error is larger than lookahead distance");
      RCLCPP_ERROR(
        logger, "lateral error: %lf, lookahead distance: %lf", failure.lateral_error,
        failure.lookahead_distance);
      RCLCPP_WARN(logger, "lost target! ");
      break;
  }
}
}  // namespace autoware::pure_pursuit
