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

#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller_node.hpp"

#include "autoware/pure_pursuit/util/planning_utils.hpp"
#include "autoware/pure_pursuit/util/tf_utils.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>

namespace autoware::pure_pursuit
{
PurePursuitLateralControllerNode::PurePursuitLateralControllerNode(rclcpp::Node & node)
: clock_(node.get_clock()),
  logger_(node.get_logger().get_child("lateral_controller")),
  tf_buffer_(clock_),
  tf_listener_(tf_buffer_),
  controller_(std::make_unique<PurePursuitLateralController>(createConfig(node)))
{
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

PurePursuitLateralControllerConfig PurePursuitLateralControllerNode::createConfig(
  rclcpp::Node & node)
{
  PurePursuitLateralControllerConfig config{};

  // Vehicle Parameters
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  config.wheel_base = vehicle_info.wheel_base_m;
  config.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  config.ld_velocity_ratio = node.declare_parameter<double>("ld_velocity_ratio");
  config.ld_lateral_error_ratio = node.declare_parameter<double>("ld_lateral_error_ratio");
  config.ld_curvature_ratio = node.declare_parameter<double>("ld_curvature_ratio");
  config.long_ld_lateral_error_threshold =
    node.declare_parameter<double>("long_ld_lateral_error_threshold");
  config.min_lookahead_distance = node.declare_parameter<double>("min_lookahead_distance");
  config.max_lookahead_distance = node.declare_parameter<double>("max_lookahead_distance");
  config.reverse_min_lookahead_distance =
    node.declare_parameter<double>("reverse_min_lookahead_distance");
  config.converged_steer_rad_ = node.declare_parameter<double>("converged_steer_rad");
  config.prediction_ds = node.declare_parameter<double>("prediction_ds");
  config.prediction_distance_length = node.declare_parameter<double>("prediction_distance_length");
  config.resampling_ds = node.declare_parameter<double>("resampling_ds");
  config.curvature_calculation_distance =
    node.declare_parameter<double>("curvature_calculation_distance");
  config.enable_path_smoothing = node.declare_parameter<bool>("enable_path_smoothing");
  config.path_filter_moving_ave_num = node.declare_parameter<int64_t>("path_filter_moving_ave_num");
  return config;
}

bool PurePursuitLateralControllerNode::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

LateralOutput PurePursuitLateralControllerNode::run(const InputData & input_data)
{
  const auto result = controller_->run(input_data, clock_->now());

  if (result.lookahead_debug_values) {
    pub_debug_values_->publish(*result.lookahead_debug_values);
  }
  if (result.predicted_trajectory) {
    pub_predicted_trajectory_->publish(*result.predicted_trajectory);
  }
  writeLog(result);

  return result.output;
}

void PurePursuitLateralControllerNode::writeLog(
  const PurePursuitLateralControllerResult & result) const
{
  if (result.path_too_short_to_smooth) {
    RCLCPP_ERROR(logger_, "Cannot smooth path! Trajectory size is too low!");
  }
  if (result.path_too_short) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "received path size is < 3, ignored");
  }
  if (result.closest_waypoint_not_found) {
    RCLCPP_ERROR(logger_, "cannot find closest waypoint");
  }
  if (result.curvature_error) {
    RCLCPP_WARN(rclcpp::get_logger("pure_pursuit"), "%s", result.curvature_error->c_str());
  }
  if (result.pure_pursuit_failure) {
    writePurePursuitFailure(*result.pure_pursuit_failure);
  }
  if (result.control_command_failed) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "failed to solve pure_pursuit for control command calculation");
  }
  if (result.prediction_failed_at_first_point) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "failed to solve pure_pursuit for prediction");
  }
  if (result.prediction_failed_at_later_point) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "failed to solve pure_pursuit for prediction");
  }
  if (!result.predicted_trajectory) {
    RCLCPP_ERROR(logger_, "Failed to generate predicted trajectory.");
  }
}

void PurePursuitLateralControllerNode::writePurePursuitFailure(
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
