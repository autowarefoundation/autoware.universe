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

#ifndef PERCEPTION_UTILS__PREDICTED_PATH_UTILS_HPP_
#define PERCEPTION_UTILS__PREDICTED_PATH_UTILS_HPP_

#include "perception_utils/geometry.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_path.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include <boost/optional.hpp>

#include <vector>

namespace perception_utils
{
/**
 * @brief Calculate Interpolated Pose from predicted path by time
 * @param path Input predicted path
 * @param relative_time time at interpolated point. This should be within [0.0,
 * time_step*(num_of_path_points)]
 * @return interpolated pose
 */
inline boost::optional<geometry_msgs::msg::Pose> calcInterpolatedPose(
  const autoware_auto_perception_msgs::msg::PredictedPath & path, const double relative_time)
{
  // Check if relative time is in the valid range
  if (path.path.empty() || relative_time < 0.0) {
    return boost::none;
  }

  constexpr double epsilon = 1e-6;
  const double & time_step = rclcpp::Duration(path.time_step).seconds();
  for (size_t path_idx = 1; path_idx < path.path.size(); ++path_idx) {
    const auto & pt = path.path.at(path_idx);
    const auto & prev_pt = path.path.at(path_idx - 1);
    if (relative_time - epsilon < time_step * path_idx) {
      const double offset = relative_time - time_step * (path_idx - 1);
      const double ratio = std::clamp(offset / time_step, 0.0, 1.0);
      return tier4_autoware_utils::calcInterpolatedPose(prev_pt, pt, ratio);
    }
  }

  return boost::none;
}

/**
 * @brief Resampling predicted path by time step vector. Note this function does not substitute
 * original time step
 * @param path Input predicted path
 * @param relative_time_vec time at each resampling point. Each time should be within [0.0,
 * time_step*(num_of_path_points)]
 * @return resampled path
 */
autoware_auto_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & path,
  const std::vector<double> & relative_time_vec)
{
  autoware_auto_perception_msgs::msg::PredictedPath resampled_path;
  resampled_path.confidence = path.confidence;

  for (const auto & rel_time : relative_time_vec) {
    const auto opt_pose = calcInterpolatedPose(path, rel_time);
    if (!opt_pose) {
      continue;
    }

    resampled_path.path.push_back(*opt_pose);
  }

  return resampled_path;
}

/**
 * @brief Resampling predicted path by sampling time interval. Note that this function samples
 * terminal point as well as points by sampling time interval
 * @param path Input predicted path
 * @param sampling_time_interval sampling time interval for each point
 * @param enable_sampling_terminal_point flag if to sample terminal point
 * @return resampled path
 */
autoware_auto_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & path,
  const double sampling_time_interval, bool enable_sampling_terminal_point = true)
{
  std::vector<double> sampling_time_vector;
  const double predicted_horizon =
    rclcpp::Duration(path.time_step).seconds() * static_cast<double>(path.path.size() - 1);
  for (double t = 0.0; t < predicted_horizon; t += sampling_time_interval) {
    sampling_time_vector.push_back(t);
  }

  // Insert Predicted Horizon(Terminal Predicted Point)
  if (enable_sampling_terminal_point) {
    if (predicted_horizon - sampling_time_vector.back() > 1e-3) {
      sampling_time_vector.push_back(predicted_horizon);
    } else {
      sampling_time_vector.back() = predicted_horizon;
    }
  }

  // Insert time step
  auto resampled_path = resamplePredictedPath(path, sampling_time_vector);
  resampled_path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval);
  return resampled_path;
}
}  // namespace perception_utils

#endif  // PERCEPTION_UTILS__PREDICTED_PATH_UTILS_HPP_
