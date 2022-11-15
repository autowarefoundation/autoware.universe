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

#include <fmt/core.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <cmath>
#include <memory>
#include <string>

geometry_msgs::msg::TwistWithCovarianceStamped get_twist_from_velocity_buffer(
  const std::vector<geometry_msgs::msg::TwistWithCovarianceStamped> & velocity_buffer)
{
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;
  if (velocity_buffer.empty()) {
    twist_with_cov.twist.covariance[0 * 6 + 0] = 100000.0;
    return twist_with_cov;
  }

  for (const auto vel : velocity_buffer) {
    twist_with_cov.twist.twist.linear.x += vel.twist.twist.linear.x;
    twist_with_cov.twist.covariance[0] += vel.twist.covariance[0];
  }
  twist_with_cov.twist.twist.linear.x /= velocity_buffer.size();
  twist_with_cov.twist.covariance[0] /=
    velocity_buffer.size();  // TODO: divide by appropriate value!!!!!!!!!
  return twist_with_cov;
}

geometry_msgs::msg::TwistWithCovarianceStamped get_twist_from_gyro_buffer(
  const std::vector<geometry_msgs::msg::TwistWithCovarianceStamped> & gyro_buffer)
{
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;
  if (gyro_buffer.empty()) {
    twist_with_cov.twist.covariance[3 * 6 + 3] = 100000.0;
    twist_with_cov.twist.covariance[4 * 6 + 4] = 100000.0;
    twist_with_cov.twist.covariance[5 * 6 + 5] = 100000.0;
    return twist_with_cov;
  }

  for (const auto gyro : gyro_buffer) {
    twist_with_cov.twist.twist.angular.x += gyro.twist.twist.angular.x;
    twist_with_cov.twist.twist.angular.y += gyro.twist.twist.angular.y;
    twist_with_cov.twist.twist.angular.z += gyro.twist.twist.angular.z;
    twist_with_cov.twist.covariance[3 * 6 + 3] += gyro.twist.covariance[3 * 6 + 3];
    twist_with_cov.twist.covariance[4 * 6 + 4] += gyro.twist.covariance[4 * 6 + 4];
    twist_with_cov.twist.covariance[5 * 6 + 5] += gyro.twist.covariance[5 * 6 + 5];
  }
  twist_with_cov.twist.twist.angular.x /= gyro_buffer.size();
  twist_with_cov.twist.twist.angular.y /= gyro_buffer.size();
  twist_with_cov.twist.twist.angular.z /= gyro_buffer.size();
  twist_with_cov.twist.covariance[3 * 6 + 3] /=
    gyro_buffer.size();  // TODO: divide by appropriate value!!!!!!!!!
  twist_with_cov.twist.covariance[4 * 6 + 4] /= gyro_buffer.size();
  twist_with_cov.twist.covariance[5 * 6 + 5] /= gyro_buffer.size();
  return twist_with_cov;
}

geometry_msgs::msg::TwistWithCovarianceStamped concat_twist_from_velocity_and_gyro(
  geometry_msgs::msg::TwistWithCovarianceStamped & twist_from_velocity,
  geometry_msgs::msg::TwistWithCovarianceStamped & twist_from_gyro)
{
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;

  twist_with_cov.twist.twist.linear.x = twist_from_velocity.twist.twist.linear.x;
  twist_with_cov.twist.covariance[0 * 6 + 0] = twist_from_velocity.twist.covariance[0 * 6 + 0];

  twist_with_cov.twist.twist.angular.x = twist_from_gyro.twist.twist.angular.x;
  twist_with_cov.twist.twist.angular.y = twist_from_gyro.twist.twist.angular.y;
  twist_with_cov.twist.twist.angular.z = twist_from_gyro.twist.twist.angular.z;
  twist_with_cov.twist.covariance[3 * 6 + 3] = twist_from_gyro.twist.covariance[3 * 6 + 3];
  twist_with_cov.twist.covariance[4 * 6 + 4] = twist_from_gyro.twist.covariance[4 * 6 + 4];
  twist_with_cov.twist.covariance[5 * 6 + 5] = twist_from_gyro.twist.covariance[5 * 6 + 5];
  return twist_with_cov;
}

GyroOdometer::GyroOdometer()
: Node("gyro_odometer"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  output_frame_(declare_parameter("base_link", "base_link")),
  message_timeout_sec_(declare_parameter("message_timeout_sec", 0.2)),
  is_velocity_arrived_(false),
  is_imu_arrived_(false)
{
  vehicle_twist_with_cov_sub_ = create_subscription<TwistWithCovarianceStamped>(
    "vehicle/twist_with_covariance", rclcpp::QoS{100},
    std::bind(&GyroOdometer::callbackTwistWithCovariance, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS{100}, std::bind(&GyroOdometer::callbackImu, this, std::placeholders::_1));

  twist_raw_pub_ = create_publisher<TwistStamped>("twist_raw", rclcpp::QoS{10});
  twist_with_covariance_raw_pub_ =
    create_publisher<TwistWithCovarianceStamped>("twist_with_covariance_raw", rclcpp::QoS{10});

  twist_pub_ = create_publisher<TwistStamped>("twist", rclcpp::QoS{10});
  twist_with_covariance_pub_ =
    create_publisher<TwistWithCovarianceStamped>("twist_with_covariance", rclcpp::QoS{10});

  double output_rate = declare_parameter("output_rate", 50.0);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(output_rate).period(),
    std::bind(&GyroOdometer::timerCallback, this));
}

GyroOdometer::~GyroOdometer() {}

void GyroOdometer::timerCallback()
{
  try {
    validityCheck(is_velocity_arrived_, is_imu_arrived_, vel_buffer_, gyro_buffer_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, e.what());
    vel_buffer_.clear();
    gyro_buffer_.clear();
    return;
  }

  TwistWithCovarianceStamped twist_with_cov_from_vel = get_twist_from_velocity_buffer(vel_buffer_);
  vel_buffer_.clear();

  TwistWithCovarianceStamped twist_with_cov_from_gyro = get_twist_from_gyro_buffer(gyro_buffer_);
  gyro_buffer_.clear();

  TwistWithCovarianceStamped twist_with_cov_raw =
    concat_twist_from_velocity_and_gyro(twist_with_cov_from_vel, twist_with_cov_from_gyro);
  twist_with_cov_raw.header.stamp = this->now();
  twist_with_cov_raw.header.frame_id = output_frame_;
  twist_with_cov_raw.twist.covariance[1 * 6 + 1] = 100000.0;  // vy
  twist_with_cov_raw.twist.covariance[2 * 6 + 2] = 100000.0;  // vz
  twist_with_covariance_raw_pub_->publish(twist_with_cov_raw);

  TwistStamped twist_raw;
  twist_raw.header = twist_with_cov_raw.header;
  twist_raw.twist = twist_with_cov_raw.twist.twist;
  twist_raw_pub_->publish(twist_raw);

  // clear imu yaw bias if vehicle is stopped
  TwistStamped twist = twist_raw;
  TwistWithCovarianceStamped twist_with_cov = twist_with_cov_raw;
  if (
    std::fabs(twist_with_cov_raw.twist.twist.linear.x) < 0.01 &&
    std::fabs(twist_with_cov_raw.twist.twist.angular.z) < 0.01) {
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;
    twist_with_cov.twist.twist.angular.x = 0.0;
    twist_with_cov.twist.twist.angular.y = 0.0;
    twist_with_cov.twist.twist.angular.z = 0.0;
  }

  twist_pub_->publish(twist);
  twist_with_covariance_pub_->publish(twist_with_cov);
}

void GyroOdometer::callbackTwistWithCovariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_with_cov_msg_ptr)
{
  // TODO(YamatoAndo) trans from twist_with_cov_msg_ptr->header to base_frame
  vel_buffer_.push_back(*twist_with_cov_msg_ptr);
  is_velocity_arrived_ = true;
}

void GyroOdometer::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  geometry_msgs::msg::TransformStamped::SharedPtr tf_base2imu_ptr =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  getTransform(output_frame_, imu_msg_ptr->header.frame_id, tf_base2imu_ptr);

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.header = imu_msg_ptr->header;
  angular_velocity.vector = imu_msg_ptr->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  transformed_angular_velocity.header = tf_base2imu_ptr->header;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_base2imu_ptr);

  TwistWithCovarianceStamped gyro;
  gyro.header = imu_msg_ptr->header;
  gyro.twist.twist.angular.x = transformed_angular_velocity.vector.x;
  gyro.twist.twist.angular.y = transformed_angular_velocity.vector.y;
  gyro.twist.twist.angular.z = transformed_angular_velocity.vector.z;
  // TODO: The following assumes that the covariance does not change before and after the
  // transformation
  gyro.twist.covariance[3 * 6 + 3] = imu_msg_ptr->angular_velocity_covariance[0 * 3 + 0];
  gyro.twist.covariance[4 * 6 + 4] = imu_msg_ptr->angular_velocity_covariance[1 * 3 + 1];
  gyro.twist.covariance[5 * 6 + 5] = imu_msg_ptr->angular_velocity_covariance[2 * 3 + 2];
  gyro_buffer_.push_back(gyro);
  is_imu_arrived_ = true;
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

void GyroOdometer::validityCheck(
  const bool is_velocity_arrived, const bool is_imu_arrived,
  const std::vector<geometry_msgs::msg::TwistWithCovarianceStamped> & velocity_buffer,
  const std::vector<geometry_msgs::msg::TwistWithCovarianceStamped> & gyro_buffer) const
{
  std::string error_msg;
  if (!is_velocity_arrived) {
    error_msg = "Twist msg is not subscribed";
    throw std::domain_error(error_msg);
  }
  if (!is_imu_arrived) {
    error_msg = "Imu msg is not subscribed";
    throw std::domain_error(error_msg);
  }
  if (!velocity_buffer.empty()) {
    const double velocity_dt =
      std::abs((this->now() - velocity_buffer.front().header.stamp).seconds());
    if (velocity_dt > message_timeout_sec_) {
      error_msg = fmt::format(
        "Twist msg is timeout. twist_dt: {}[sec], tolerance {}[sec]", velocity_dt,
        message_timeout_sec_);
      throw std::domain_error(error_msg);
    }
  }
  if (!gyro_buffer.empty()) {
    const double imu_dt = std::abs((this->now() - gyro_buffer.front().header.stamp).seconds());
    if (imu_dt > message_timeout_sec_) {
      error_msg = fmt::format(
        "Imu msg is timeout. imu_dt: {}[sec], tolerance {}[sec]", imu_dt, message_timeout_sec_);
      throw std::domain_error(error_msg);
    }
  }
}
