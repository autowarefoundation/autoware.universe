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

#ifndef EKF_LOCALIZER__YAW_BIAS_MONITOR_HPP_
#define EKF_LOCALIZER__YAW_BIAS_MONITOR_HPP_

#include "ekf_localizer/simple_filter_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <string>

class YawBiasEstimator : public Simple1DFilter
{
public:
  YawBiasEstimator()
  : Simple1DFilter(),
    time_lower_limit_(0.03),
    speed_lower_limit_(2),
    rotation_speed_upper_limit_(0.01),
    distance_upper_limit_(10),
    distance_lower_limit_(0.1)
  {
  }

  void update(const geometry_msgs::msg::PoseWithCovarianceStamped & pose, double obs_variance = 0.1)
  {
    auto pose_time_diff =
      rclcpp::Time(pose.header.stamp) - rclcpp::Time(previous_ndt_pose_.header.stamp);
    double time_diff = pose_time_diff.seconds();

    std::cout << "1DKF: time_diff: " << time_diff << std::endl;
    if (time_diff < time_lower_limit_) {
      return;
    }

    double dx = pose.pose.pose.position.x - previous_ndt_pose_.pose.pose.position.x;
    double dy = pose.pose.pose.position.y - previous_ndt_pose_.pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double estimated_yaw = std::atan2(dy, dx);
    double measured_yaw = angle_from_pose(pose);

    double yaw_bias = normalize_angle_diff(measured_yaw - estimated_yaw);

    // Compute Speed and Rotation Speed to make sure the result is reliable
    double speed = distance / time_diff;

    //// Rotational Speed:
    double previous_yaw = angle_from_pose(previous_ndt_pose_);
    double yaw_increment = normalize_angle_diff(measured_yaw - previous_yaw);
    double rotation_speed = std::abs(yaw_increment / time_diff);

    previous_ndt_pose_ = pose;
    if (
      (speed < speed_lower_limit_) || (rotation_speed > rotation_speed_upper_limit_) ||
      (distance > distance_upper_limit_) || (distance < distance_lower_limit_)) {
      return;  // ignore when speed is low or rotation speed is high
    }
    Simple1DFilter::update(yaw_bias, obs_variance, pose.header.stamp);
  }

private:
  geometry_msgs::msg::PoseWithCovarianceStamped previous_ndt_pose_;
  const double time_lower_limit_;
  const double speed_lower_limit_;
  const double rotation_speed_upper_limit_;
  const double distance_upper_limit_;
  const double distance_lower_limit_;

  double normalize_angle_diff(double angle_difference)
  {
    while (angle_difference > M_PI) {
      angle_difference -= 2 * M_PI;
    }
    while (angle_difference < -M_PI) {
      angle_difference += 2 * M_PI;
    }
    return angle_difference;
  }
  double angle_from_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
  {
    return std::atan2(pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) * 2;
  }
};

#endif  // EKF_LOCALIZER__YAW_BIAS_MONITOR_HPP_
