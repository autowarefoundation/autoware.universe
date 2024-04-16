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

#include <sstream>
#include <string>

void NDTScanMatcher::initialize_diagnostics_key_value()
{
  diagnostics_scan_points_->addKeyValue("topic_time_stamp", 0.0);
  diagnostics_scan_points_->addKeyValue("is_activated", false);
  diagnostics_scan_points_->addKeyValue("is_set_map_points", false);
  diagnostics_scan_points_->addKeyValue("is_set_sensor_points", false);
  diagnostics_scan_points_->addKeyValue("sensor_points_size", 0);
  diagnostics_scan_points_->addKeyValue("sensor_points_delay_time_sec", 0.0);
  diagnostics_scan_points_->addKeyValue("sensor_points_max_distance", 0.0);
  diagnostics_scan_points_->addKeyValue("is_succeed_interpolate_initial_pose", false);
  diagnostics_scan_points_->addKeyValue("iteration_num", 0);
  diagnostics_scan_points_->addKeyValue("local_optimal_solution_oscillation_count", 0.0);
  diagnostics_scan_points_->addKeyValue("transform_probability", 0.0);
  diagnostics_scan_points_->addKeyValue("nearest_voxel_transformation_likelihood", 0.0);
  diagnostics_scan_points_->addKeyValue("distance_initial_to_result", 0.0);
  diagnostics_scan_points_->addKeyValue("execution_time", 0.0);
  diagnostics_scan_points_->addKeyValue("skipping_publish_num", 0);
}

bool NDTScanMatcher::validate_is_node_activated(const bool is_activated)
{
  diagnostics_scan_points_->addKeyValue("is_activated", is_activated);

  bool is_ok = is_activated;
  if (!is_ok) {
    std::stringstream message;
    message << "Node is not activated.";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_is_set_map_points(const bool is_set_map_points)
{
  diagnostics_scan_points_->addKeyValue("is_set_map_points", is_set_map_points);

  bool is_ok = is_set_map_points;
  if (!is_ok) {
    std::stringstream message;
    message << "Map points is not set.";  // TODO "Map points are?"
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_is_set_sensor_points(const bool is_set_sensor_points)
{
  diagnostics_scan_points_->addKeyValue("is_set_sensor_points", is_set_sensor_points);

  bool is_ok = is_set_sensor_points;
  if (!is_ok) {
    std::stringstream message;
    message << "Sensor points is not set.";  // TODO "Sensor points are?"
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_sensor_points_empty(const size_t sensor_points_size)
{
  diagnostics_scan_points_->addKeyValue("sensor_points_size", sensor_points_size);

  bool is_ok = (sensor_points_size > 0);
  if (!is_ok) {
    std::stringstream message;
    message << "Sensor points is empty.";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_sensor_points_delay_time(
  const rclcpp::Time & sensor_ros_time, const rclcpp::Time & ros_time_now,
  const double warn_timeout_sec)
{
  const double sensor_points_delay_time_sec = (ros_time_now - sensor_ros_time).seconds();
  diagnostics_scan_points_->addKeyValue(
    "sensor_points_delay_time_sec", sensor_points_delay_time_sec);

  bool is_ok = sensor_points_delay_time_sec < warn_timeout_sec;
  if (!is_ok) {
    std::stringstream message;
    message << "sensor points is experiencing latency."
            << "The delay time is " << sensor_points_delay_time_sec << "[sec] "
            << "(the tolerance is " << warn_timeout_sec << "[sec]).";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_sensor_points_max_distance(
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points, const double warn_distance)
{
  double max_distance = 0.0;
  for (const auto & point : sensor_points->points) {
    const double distance = std::hypot(point.x, point.y, point.z);
    max_distance = std::max(max_distance, distance);
  }

  diagnostics_scan_points_->addKeyValue("sensor_points_max_distance", max_distance);

  bool is_ok = (max_distance > warn_distance);
  if (!is_ok) {
    std::stringstream message;
    message << "Max distance of sensor points = " << std::fixed << std::setprecision(3)
            << max_distance << " [m] < " << warn_distance << " [m]";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_initial_pose_array_size(const size_t initial_pose_array_size)
{
  diagnostics_scan_points_->addKeyValue("initial_pose_array_size", initial_pose_array_size);

  bool is_ok =
    (initial_pose_array_size >= 2);  // To perform linear interpolation, need at least two pose
  if (!is_ok) {
    std::stringstream message;
    message << "Initial poses have not arrived. Please check the initial pose topic.";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_succeed_interpolate_initial_pose(const bool is_succeed)
{
  diagnostics_scan_points_->addKeyValue("is_succeed_interpolate_initial_pose", is_succeed);

  bool is_ok = is_succeed;
  if (!is_ok) {
    std::stringstream message;
    message << "Couldn't interpolate pose. Please check the initial pose topic";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_num_iteration(const int iter_num, const int max_iter_num)
{
  diagnostics_scan_points_->addKeyValue("iteration_num", iter_num);

  bool is_ok = iter_num < max_iter_num;
  if (!is_ok) {
    std::stringstream message;
    message << "The number of iterations has reached its upper limit. The number of iterations: "
            << iter_num << ", Limit: " << max_iter_num << ".";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_local_optimal_solution_oscillation(
  const int oscillation_count, const int oscillation_count_threshold)
{
  diagnostics_scan_points_->addKeyValue(
    "local_optimal_solution_oscillation_count", oscillation_count);

  bool is_ok = oscillation_count < oscillation_count_threshold;
  if (!is_ok) {
    std::stringstream message;
    message << "There is a possibility of oscillation in a local minimum";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_score(
  const double score, const double score_threshold, const std::string & score_name)
{
  bool is_ok = score > score_threshold;
  if (!is_ok) {
    std::stringstream message;
    message << score_name << " is below the threshold. Score: " << score
            << ", Threshold: " << score_threshold << ".";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_converged_param(
  const double transform_probability, const double nearest_voxel_transformation_likelihood)
{
  diagnostics_scan_points_->addKeyValue("transform_probability", transform_probability);
  diagnostics_scan_points_->addKeyValue(
    "nearest_voxel_transformation_likelihood", nearest_voxel_transformation_likelihood);

  bool is_ok = false;
  if (param_.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
    is_ok = validate_score(
      transform_probability, param_.score_estimation.converged_param_transform_probability,
      "Transform Probability");
  } else if (
    param_.score_estimation.converged_param_type ==
    ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    is_ok = validate_score(
      nearest_voxel_transformation_likelihood,
      param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood,
      "Nearest Voxel Transformation Likelihood");
  } else {
    is_ok = false;

    std::stringstream message;
    message << "Unknown converged param type. Please check `score_estimation.converged_param_type`";
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_distance_initial_to_result(
  const double distance_initial_to_result, const double warn_distance_initial_to_result)
{
  diagnostics_scan_points_->addKeyValue("distance_initial_to_result", distance_initial_to_result);

  bool is_ok = distance_initial_to_result < warn_distance_initial_to_result;
  if (!is_ok) {
    std::stringstream message;
    message << "distance_initial_to_result is too large (" << distance_initial_to_result
            << " [m]).";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_execution_time(
  const double execution_time, const double warn_execution_time)
{
  diagnostics_scan_points_->addKeyValue("execution_time", execution_time);

  bool is_ok = execution_time < warn_execution_time;
  if (!is_ok) {
    std::stringstream message;
    message << "NDT exe time is too long (took " << execution_time << " [ms]).";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }
  return is_ok;
}

bool NDTScanMatcher::validate_skipping_publish_num(
  const size_t skipping_publish_num, const size_t error_num)
{
  diagnostics_scan_points_->addKeyValue("skipping_publish_num", skipping_publish_num);

  bool is_ok = skipping_publish_num < error_num;
  if (!is_ok) {
    std::stringstream message;
    message << "skipping_publish_num exceed limit ( " << skipping_publish_num << " times).";
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10, message.str());
    diagnostics_scan_points_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
  }
  return is_ok;
}
