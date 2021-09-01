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

#include "gyro_odometer/gyro_odometer_core.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

GyroOdometer::GyroOdometer()
: Node("gyro_odometer"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  output_frame_(declare_parameter("base_link", "base_link"))
{
  use_twist_with_covariance_ = declare_parameter("use_twist_with_covariance", true);

  vehicle_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "vehicle/twist", rclcpp::QoS{100},
    std::bind(
      &GyroOdometer::callbackTwist, this,
      std::placeholders::_1));  // Deprecated

  vehicle_twist_with_cov_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "vehicle/twist_with_covariance", rclcpp::QoS{100},
    std::bind(&GyroOdometer::callbackTwistWithCovariance, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS{100}, std::bind(&GyroOdometer::callbackImu, this, std::placeholders::_1));

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS{10});
  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10});

  // TODO(YamatoAndo) createTimer
}

GyroOdometer::~GyroOdometer() {}

void GyroOdometer::callbackTwist(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg_ptr)
{
  if (use_twist_with_covariance_) {
    return;
  }

  // TODO(YamatoAndo) trans from twist_msg_ptr->header to base_frame

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_msg;
  twist_with_cov_msg.header = twist_msg_ptr->header;
  twist_with_cov_msg.twist.twist = twist_msg_ptr->twist;

  // NOTE
  // linear.y, linear.z, angular.x, and angular.y are not measured values.
  // Therefore, they should be assigned large variance values.
  twist_with_cov_msg.twist.covariance[0] = 0.2 * 0.2;
  twist_with_cov_msg.twist.covariance[7] = 10000.0;
  twist_with_cov_msg.twist.covariance[14] = 10000.0;
  twist_with_cov_msg.twist.covariance[21] = 10000.0;
  twist_with_cov_msg.twist.covariance[28] = 10000.0;
  twist_with_cov_msg.twist.covariance[35] = 0.1 * 0.1;

  twist_with_cov_msg_ptr_ =
    std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_with_cov_msg);
}

void GyroOdometer::callbackTwistWithCovariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_with_cov_msg_ptr)
{
  if (!use_twist_with_covariance_) {
    return;
  }

  // TODO(YamatoAndo) trans from twist_with_cov_msg_ptr->header to base_frame
  twist_with_cov_msg_ptr_ = twist_with_cov_msg_ptr;
}

void GyroOdometer::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  if (!twist_with_cov_msg_ptr_) {
    return;
  }

  geometry_msgs::msg::TransformStamped::SharedPtr tf_base2imu_ptr =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  getTransform(output_frame_, imu_msg_ptr->header.frame_id, tf_base2imu_ptr);

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.header = imu_msg_ptr->header;
  angular_velocity.vector = imu_msg_ptr->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  transformed_angular_velocity.header = tf_base2imu_ptr->header;

  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_base2imu_ptr);

  // clear imu yaw bias if vehicle is stopped
  if (
    std::fabs(transformed_angular_velocity.vector.z) < 0.01 &&
    std::fabs(twist_with_cov_msg_ptr_->twist.twist.linear.x) < 0.01)
  {
    transformed_angular_velocity.vector.z = 0.0;
  }

  // TODO(YamatoAndo) move code
  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = imu_msg_ptr->header.stamp;
  twist.header.frame_id = output_frame_;
  twist.twist.linear = twist_with_cov_msg_ptr_->twist.twist.linear;
  twist.twist.angular.z = transformed_angular_velocity.vector.z;  // TODO(YamatoAndo) yaw_rate only
  twist_pub_->publish(twist);

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance;
  twist_with_covariance.header.stamp = imu_msg_ptr->header.stamp;
  twist_with_covariance.header.frame_id = output_frame_;
  twist_with_covariance.twist.twist.linear = twist_with_cov_msg_ptr_->twist.twist.linear;
  twist_with_covariance.twist.twist.angular.z =
    transformed_angular_velocity.vector.z;  // TODO(YamatoAndo) yaw_rate only

  // NOTE
  // linear.y, linear.z, angular.x, and angular.y are not measured values.
  // Therefore, they should be assigned large variance values.
  twist_with_covariance.twist.covariance[0] = twist_with_cov_msg_ptr_->twist.covariance[0];
  twist_with_covariance.twist.covariance[7] = 10000.0;
  twist_with_covariance.twist.covariance[14] = 10000.0;
  twist_with_covariance.twist.covariance[21] = 10000.0;
  twist_with_covariance.twist.covariance[28] = 10000.0;
  twist_with_covariance.twist.covariance[35] = imu_msg_ptr->angular_velocity_covariance[8];

  twist_with_covariance_pub_->publish(twist_with_covariance);
}

bool GyroOdometer::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = this->get_clock()->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = this->get_clock()->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}
