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

#include "gyro_bias_validator.hpp"

namespace imu_corrector
{
GyroBiasValidator::GyroBiasValidator(const rclcpp::NodeOptions & node_options)
: Node("gyro_bias_validator", node_options),
  gyro_bias_threshold_(declare_parameter<double>("gyro_bias_threshold"))
{
  const double velocity_threshold = declare_parameter<double>("velocity_threshold");
  const double timestamp_threshold = declare_parameter<double>("timestamp_threshold");
  const int data_num_threshold = declare_parameter<int>("data_num_threshold");
  gyro_bias_estimation_module_ = std::make_unique<GyroBiasEstimationModule>(
    velocity_threshold, timestamp_threshold, static_cast<size_t>(data_num_threshold));

  imu_sub_ = create_subscription<Imu>(
    "~/input/imu", rclcpp::SensorDataQoS(),
    [this](const Imu::ConstSharedPtr msg) { callback_imu(msg); });
  twist_sub_ = create_subscription<TwistWithCovarianceStamped>(
    "~/input/twist", rclcpp::SensorDataQoS(),
    [this](const TwistWithCovarianceStamped::ConstSharedPtr msg) { callback_twist(msg); });

  gyro_bias_pub_ = create_publisher<Vector3Stamped>("~/output/gyro_bias", rclcpp::SensorDataQoS());
}

void GyroBiasValidator::callback_imu(const Imu::ConstSharedPtr imu_msg_ptr)
{
  try {
    gyro_bias_estimation_module_->update_gyro(
      rclcpp::Time(imu_msg_ptr->header.stamp).seconds(), imu_msg_ptr->angular_velocity);
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 1000, e.what());
  }

  try {
    const auto gyro_bias = gyro_bias_estimation_module_->get_bias();
    if (
      std::abs(gyro_bias.x) > gyro_bias_threshold_ ||
      std::abs(gyro_bias.y) > gyro_bias_threshold_ ||
      std::abs(gyro_bias.z) > gyro_bias_threshold_) {
      const std::string warn_msg =
        "Gyro bias is too large: " + std::to_string(gyro_bias.x) + ", " +
        std::to_string(gyro_bias.y) + ", " + std::to_string(gyro_bias.z) +
        ". You may need to update the calibration file in imu_corrector.";
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 1000, warn_msg);

      Vector3Stamped gyro_bias_msg;
      gyro_bias_msg.header = imu_msg_ptr->header;
      gyro_bias_msg.vector = gyro_bias;
      gyro_bias_pub_->publish(gyro_bias_msg);
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 1000, e.what());
  }
}

void GyroBiasValidator::callback_twist(
  const TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  gyro_bias_estimation_module_->update_velocity(
    rclcpp::Time(twist_msg_ptr->header.stamp).seconds(), twist_msg_ptr->twist.twist.linear.x);
}

}  // namespace imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_corrector::GyroBiasValidator)
