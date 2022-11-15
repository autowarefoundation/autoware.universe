// Copyright 2015-2019 Autoware Foundation
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

#ifndef GYRO_ODOMETER__GYRO_ODOMETER_CORE_HPP_
#define GYRO_ODOMETER__GYRO_ODOMETER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/transform_datatypes.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <vector>

class GyroOdometer : public rclcpp::Node
{
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;

public:
  GyroOdometer();
  ~GyroOdometer();

private:
  void timerCallback();

  void callbackTwistWithCovariance(
    const TwistWithCovarianceStamped::ConstSharedPtr twist_with_cov_msg_ptr);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr);
  void validityCheck(
    const bool is_velocity_arrived, const bool is_imu_arrived,
    const std::vector<TwistWithCovarianceStamped> & velocity_buffer,
    const std::vector<TwistWithCovarianceStamped> & gyro_buffer) const;

  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr vehicle_twist_with_cov_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<TwistStamped>::SharedPtr twist_raw_pub_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr twist_with_covariance_raw_pub_;

  rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr twist_with_covariance_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<TwistWithCovarianceStamped> vel_buffer_;
  std::vector<TwistWithCovarianceStamped> gyro_buffer_;

  std::string output_frame_;
  double message_timeout_sec_;

  bool is_velocity_arrived_;
  bool is_imu_arrived_;
};

#endif  // GYRO_ODOMETER__GYRO_ODOMETER_CORE_HPP_
