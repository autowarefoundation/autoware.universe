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

#ifndef GYRO_ODOMETER_CORE_HPP_
#define GYRO_ODOMETER_CORE_HPP_

#include "autoware/universe_utils/ros/diagnostics_interface.hpp"
#include "autoware/universe_utils/ros/logger_level_configure.hpp"
#include "autoware/universe_utils/ros/msg_covariance.hpp"
#include "autoware/universe_utils/ros/transform_listener.hpp"

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

#include <deque>
#include <memory>
#include <string>

namespace autoware::gyro_odometer
{

class GyroOdometerNode : public rclcpp::Node
{
private:
  using COV_IDX = autoware::universe_utils::xyz_covariance_index::XYZ_COV_IDX;

public:
  explicit GyroOdometerNode(const rclcpp::NodeOptions & node_options);

private:
  void callback_vehicle_twist(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vehicle_twist_msg_ptr);
  void callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  void concat_gyro_and_odometer();
  void publish_data(const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw);

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    vehicle_twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_raw_pub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_pub_;

  std::shared_ptr<autoware::universe_utils::TransformListener> transform_listener_;
  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  std::string output_frame_;
  double message_timeout_sec_;

  bool vehicle_twist_arrived_;
  bool imu_arrived_;
  rclcpp::Time latest_vehicle_twist_ros_time_;
  rclcpp::Time latest_imu_ros_time_;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> vehicle_twist_queue_;
  std::deque<sensor_msgs::msg::Imu> gyro_queue_;

  std::unique_ptr<autoware::universe_utils::DiagnosticsInterface> diagnostics_;
};

}  // namespace autoware::gyro_odometer

#endif  // GYRO_ODOMETER_CORE_HPP_
