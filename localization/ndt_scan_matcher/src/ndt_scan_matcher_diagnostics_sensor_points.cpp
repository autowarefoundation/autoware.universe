// Copyright 2023 Autoware Foundation
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

#include "ndt_scan_matcher/diagnostics_module.hpp"
#include "ndt_scan_matcher/ndt_scan_matcher_core.hpp"

#include <string>

void NDTScanMatcher::initialize_diagnostics_key_value()
{
  diagnostics_module_->addKeyValue("is_activated", false);
  diagnostics_module_->addKeyValue("is_set_map_points", false);
  diagnostics_module_->addKeyValue("is_set_sensor_points", false);
  diagnostics_module_->addKeyValue("sensor_points_size", 0);
  diagnostics_module_->addKeyValue("sensor_points_delay_time_sec", 0.0);
  diagnostics_module_->addKeyValue("initial_pose_array_size", 0);
  diagnostics_module_->addKeyValue("time_difference_of_lidar_and_old_pose", 0.0);
  diagnostics_module_->addKeyValue("time_difference_of_lidar_and_new_pose", 0.0);
  diagnostics_module_->addKeyValue("distance_old_pose_to_new_pose", 0.0);
  diagnostics_module_->addKeyValue("iteration_num", 0);
  diagnostics_module_->addKeyValue("is_ok_local_optimal_solution_oscillation", false);
  diagnostics_module_->addKeyValue("transform_probability", 0.0);
  diagnostics_module_->addKeyValue("nearest_voxel_transformation_likelihood", 0.0);
  diagnostics_module_->addKeyValue("distance_initial_to_result", 0.0);
  diagnostics_module_->addKeyValue("execution_time", 0.0);
  diagnostics_module_->addKeyValue("skipping_publish_num", 0);
}

bool NDTScanMatcher::validate_is_node_activated()
{
  diagnostics_module_->addKeyValue("is_activated", is_activated_);

  bool is_ok = is_activated_;
  if (!is_ok) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Node is not activated");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "[WARN] Node is not activated");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_is_set_map_points()
{
  bool is_set_map_points = (ndt_ptr_->getInputTarget() != nullptr);
  diagnostics_module_->addKeyValue("is_set_map_points", is_set_map_points);

  bool is_ok = is_set_map_points;
  if (!is_ok) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Map points have not arrived");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "[WARN] Map points have not arrived");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_is_set_sensor_points(const bool is_set_sensor_points)
{
  diagnostics_module_->addKeyValue("is_set_sensor_points", is_set_sensor_points);

  bool is_ok = is_set_sensor_points;
  if (!is_ok) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Sensor-points is not set");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "[WARN] Sensor points is not set");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_sensor_points_empty(const size_t sensor_points_size)
{
  diagnostics_module_->addKeyValue("sensor_points_size", sensor_points_size);

  bool is_ok = (sensor_points_size > 0);
  if (!is_ok) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Empty sensor points!");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "[WARN] Empty sensor points");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_sensor_points_delay_time(
  const rclcpp::Time & sensor_ros_time, const rclcpp::Time & ros_time_now,
  const double warn_timeout_sec)
{
  const double sensor_points_delay_time_sec = (ros_time_now - sensor_ros_time).seconds();
  diagnostics_module_->addKeyValue("sensor_points_delay_time_sec", sensor_points_delay_time_sec);

  bool is_ok = sensor_points_delay_time_sec < warn_timeout_sec;
  if (!is_ok) {
    RCLCPP_WARN(
      this->get_logger(),
      "The sensor points is experiencing latency. The delay time is %lf[sec] (the tolerance is "
      "%lf[sec])",
      sensor_points_delay_time_sec, warn_timeout_sec);

    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "[WARN] sensor_points_delay_time_sec exceed limit");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_initial_pose_array_size(const size_t initial_pose_array_size)
{
  diagnostics_module_->addKeyValue("initial_pose_array_size", initial_pose_array_size);

  bool is_ok =
    (initial_pose_array_size >= 2);  // To perform linear interpolation, need at least two pose
  if (!is_ok) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Initial poses have not arrived");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "[WARN] Initial poses have not arrived");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_time_stamp_difference(
  const std::string & name, const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
  const double time_tolerance_sec)
{
  const double dt = std::abs((target_time - reference_time).seconds());
  diagnostics_module_->addKeyValue("time_difference_of_lidar_and_" + name, dt);

  bool is_ok = dt < time_tolerance_sec;
  if (!is_ok) {
    RCLCPP_WARN(
      this->get_logger(),
      "Validation error. The reference time is %lf[sec], but the target time is %lf[sec]. The "
      "difference is %lf[sec] (the tolerance is %lf[sec]).",
      reference_time.seconds(), target_time.seconds(), dt, time_tolerance_sec);

    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "[WARN] The time stamp difference is too large to perform pose linear interpolation");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_position_difference(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & reference_point,
  const double distance_tolerance_m_)
{
  double distance = norm(target_point, reference_point);
  diagnostics_module_->addKeyValue("distance_old_pose_to_new_pose", distance);

  bool is_ok = distance < distance_tolerance_m_;
  if (!is_ok) {
    RCLCPP_WARN(
      this->get_logger(),
      "Validation error. The distance from reference position to target position is %lf[m] (the "
      "tolerance is %lf[m]).",
      distance, distance_tolerance_m_);
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "[WARN] The distance of poses is too large to perform pose linear interpolation");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_num_iteration(const int iter_num, const int max_iter_num)
{
  diagnostics_module_->addKeyValue("iteration_num", iter_num);

  bool is_ok = iter_num < max_iter_num;
  if (!is_ok) {
    RCLCPP_WARN(
      this->get_logger(),
      "The number of iterations has reached its upper limit. The number of iterations: %d, Limit: "
      "%d",
      iter_num, max_iter_num);
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "[WARN] The number of iterations has reached its upper limit");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_local_optimal_solution_oscillation(
  const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array,
  const float oscillation_threshold, const float inversion_vector_threshold)
{
  bool is_ok_local_optimal_solution_oscillation = !is_local_optimal_solution_oscillation(
    result_pose_msg_array, oscillation_threshold, inversion_vector_threshold);
  diagnostics_module_->addKeyValue(
    "is_ok_local_optimal_solution_oscillation", is_ok_local_optimal_solution_oscillation);

  bool is_ok = is_ok_local_optimal_solution_oscillation;
  if (!is_ok) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "There is a possibility of oscillation in a local minimum");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "[WARN] There is a possibility of oscillation in a local minimum.");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_score(
  const double score, const double score_threshold, const std::string & score_name)
{
  bool is_ok = score > score_threshold;
  if (!is_ok) {
    RCLCPP_WARN(
      this->get_logger(), "%s is below the threshold. Score: %lf, Threshold: %lf",
      score_name.c_str(), score, score_threshold);
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "[WARN] NDT score is unreliably low");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_converged_param(
  const double transform_probability, const double nearest_voxel_transformation_likelihood)
{
  diagnostics_module_->addKeyValue("transform_probability", transform_probability);
  diagnostics_module_->addKeyValue(
    "nearest_voxel_transformation_likelihood", nearest_voxel_transformation_likelihood);

  bool is_ok = false;
  if (param_.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
    is_ok = validate_score(
      transform_probability, param_.score_estimation.converged_param_transform_probability, "Transform Probability");
  } else if (param_.score_estimation.converged_param_type == ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    is_ok = validate_score(
      nearest_voxel_transformation_likelihood,
      param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood,
      "Nearest Voxel Transformation Likelihood");
  } else {
    is_ok = false;
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Unknown converged param type.");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "[WARN] Unknown converged param type");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_distance_initial_to_result(
  const double distance_initial_to_result, const double warn_distance_initial_to_result)
{
  diagnostics_module_->addKeyValue("distance_initial_to_result", distance_initial_to_result);

  bool is_ok = distance_initial_to_result < warn_distance_initial_to_result;
  if (!is_ok) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "distance_initial_to_result is too large. ( " << distance_initial_to_result << " [m])");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "[WARN] distance_initial_to_result is too large");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_execution_time(
  const double execution_time, const double warn_execution_time)
{
  diagnostics_module_->addKeyValue("execution_time", execution_time);

  bool is_ok = execution_time < warn_execution_time;
  if (!is_ok) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "NDT exe time is too long. (took " << execution_time << " [ms])");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "[WARN] NDT exe time is too long");
  }
  return is_ok;
}

bool NDTScanMatcher::validate_skipping_publish_num(
  const size_t skipping_publish_num, const size_t error_num)
{
  diagnostics_module_->addKeyValue("skipping_publish_num", skipping_publish_num);

  bool is_ok = skipping_publish_num < error_num;
  if (!is_ok) {
    RCLCPP_ERROR(this->get_logger(), "skipping_publish_num exceed limit");
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "[WARN] skipping_publish_num exceed limit");
  }
  return is_ok;
}
