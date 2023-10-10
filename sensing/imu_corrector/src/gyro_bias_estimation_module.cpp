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

#include "gyro_bias_estimation_module.hpp"

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>

namespace imu_corrector
{

/**
 * @brief perform dead reckoning based on "gyro_list" and return a relative pose (in RPY)
 */
geometry_msgs::msg::Vector3 integrate_orientation(
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias)
{
  geometry_msgs::msg::Vector3 d_rpy{};
  double t_prev = rclcpp::Time(gyro_list.front().header.stamp).seconds();
  for (std::size_t i = 0; i < gyro_list.size() - 1; ++i) {
    double t_cur = rclcpp::Time(gyro_list[i + 1].header.stamp).seconds();

    d_rpy.x += (t_cur - t_prev) * (gyro_list[i].vector.x - gyro_bias.x);
    d_rpy.y += (t_cur - t_prev) * (gyro_list[i].vector.y - gyro_bias.y);
    d_rpy.z += (t_cur - t_prev) * (gyro_list[i].vector.z - gyro_bias.z);

    t_prev = t_cur;
  }
  return d_rpy;
}

/**
 * @brief calculate RPY error on dead-reckoning (calculated from "gyro_list") compared to the
 * ground-truth pose from "pose_list".
 */
geometry_msgs::msg::Vector3 calculate_error_rpy(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias)
{
  const geometry_msgs::msg::Vector3 rpy_0 =
    tier4_autoware_utils::getRPY(pose_list.front().pose.orientation);
  const geometry_msgs::msg::Vector3 rpy_1 =
    tier4_autoware_utils::getRPY(pose_list.back().pose.orientation);
  const geometry_msgs::msg::Vector3 d_rpy = integrate_orientation(gyro_list, gyro_bias);

  geometry_msgs::msg::Vector3 error_rpy;
  error_rpy.x = tier4_autoware_utils::normalizeRadian(-rpy_1.x + rpy_0.x + d_rpy.x);
  error_rpy.y = tier4_autoware_utils::normalizeRadian(-rpy_1.y + rpy_0.y + d_rpy.y);
  error_rpy.z = tier4_autoware_utils::normalizeRadian(-rpy_1.z + rpy_0.z + d_rpy.z);
  return error_rpy;
}

GyroBiasEstimationModule::GyroBiasEstimationModule(const size_t data_num_threshold)
: data_num_threshold_(data_num_threshold)
{
}

/**
 * @brief update gyroscope bias based on a given trajectory data
 */
void GyroBiasEstimationModule::update_bias(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list)
{
  const double dt_pose =
    (rclcpp::Time(pose_list.back().header.stamp) - rclcpp::Time(pose_list.front().header.stamp))
      .seconds();
  const double dt_gyro =
    (rclcpp::Time(gyro_list.back().header.stamp) - rclcpp::Time(gyro_list.front().header.stamp))
      .seconds();
  if (dt_pose == 0 || dt_gyro == 0) {
    throw std::runtime_error("dt_pose or dt_gyro is zero");
  }

  auto error_rpy = calculate_error_rpy(pose_list, gyro_list, geometry_msgs::msg::Vector3{});
  error_rpy.x *= dt_pose / dt_gyro;
  error_rpy.y *= dt_pose / dt_gyro;
  error_rpy.z *= dt_pose / dt_gyro;

  gyro_bias_pair_.first.x += dt_pose * error_rpy.x;
  gyro_bias_pair_.first.y += dt_pose * error_rpy.y;
  gyro_bias_pair_.first.z += dt_pose * error_rpy.z;
  gyro_bias_pair_.second.x += dt_pose * dt_pose;
  gyro_bias_pair_.second.y += dt_pose * dt_pose;
  gyro_bias_pair_.second.z += dt_pose * dt_pose;

  geometry_msgs::msg::Vector3 gyro_bias;
  gyro_bias.x = error_rpy.x / dt_pose;
  gyro_bias.y = error_rpy.y / dt_pose;
  gyro_bias.z = error_rpy.z / dt_pose;
  gyro_bias_deque_.push_back(gyro_bias);
  if (gyro_bias_deque_.size() > data_num_threshold_) {
    gyro_bias_deque_.pop_front();
  }
}

/**
 * @brief getter function for current estimated bias
 */
geometry_msgs::msg::Vector3 GyroBiasEstimationModule::get_bias_base_link() const
{
  geometry_msgs::msg::Vector3 gyro_bias_base;
  if (
    gyro_bias_pair_.second.x == 0 || gyro_bias_pair_.second.y == 0 ||
    gyro_bias_pair_.second.z == 0) {
    throw std::runtime_error("gyro_bias_pair_.second is zero");
  }
  gyro_bias_base.x = gyro_bias_pair_.first.x / gyro_bias_pair_.second.x;
  gyro_bias_base.y = gyro_bias_pair_.first.y / gyro_bias_pair_.second.y;
  gyro_bias_base.z = gyro_bias_pair_.first.z / gyro_bias_pair_.second.z;
  return gyro_bias_base;
}

/**
 * @brief getter function for current standard deviation of estimated bias
 */
geometry_msgs::msg::Vector3 GyroBiasEstimationModule::get_bias_std() const
{
  auto calculate_std_mean_const = [](const std::vector<double> & v, const double mean) {
    if (v.size() == 0) {
      return 0.0;
    }

    double error = 0;
    for (const double & t : v) {
      error += pow(t - mean, 2);
    }
    return std::sqrt(error / v.size());
  };

  std::vector<double> stddev_bias_list_x, stddev_bias_list_y, stddev_bias_list_z;
  for (const auto & e : gyro_bias_deque_) {
    stddev_bias_list_x.push_back(e.x);
    stddev_bias_list_y.push_back(e.y);
    stddev_bias_list_z.push_back(e.z);
  }
  geometry_msgs::msg::Vector3 mean = get_bias_base_link();
  geometry_msgs::msg::Vector3 stddev_bias;
  stddev_bias.x = calculate_std_mean_const(stddev_bias_list_x, mean.x);
  stddev_bias.y = calculate_std_mean_const(stddev_bias_list_y, mean.y);
  stddev_bias.z = calculate_std_mean_const(stddev_bias_list_z, mean.z);
  return stddev_bias;
}

}  // namespace imu_corrector
