// Copyright 2020 Tier IV, Inc.
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
#ifndef GYRO_BIAS_VALIDATOR_HPP_
#define GYRO_BIAS_VALIDATOR_HPP_

#include "gyro_bias_estimation_module.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>
#include <string>

namespace imu_corrector
{
class GyroBiasValidator : public rclcpp::Node
{
private:
  using Imu = sensor_msgs::msg::Imu;
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;

public:
  explicit GyroBiasValidator(const rclcpp::NodeOptions & node_options);

private:
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void callback_imu(const Imu::ConstSharedPtr imu_msg_ptr);
  void callback_twist(const TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr);

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_sub_;

  rclcpp::Publisher<Vector3Stamped>::SharedPtr gyro_bias_pub_;

  std::unique_ptr<GyroBiasEstimationModule> gyro_bias_estimation_module_;

  const double gyro_bias_threshold_;
  diagnostic_updater::Updater updater_;
};
}  // namespace imu_corrector

#endif  // GYRO_BIAS_VALIDATOR_HPP_
