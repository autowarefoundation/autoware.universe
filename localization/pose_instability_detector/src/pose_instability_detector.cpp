// Copyright 2023- Autoware Foundation
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

#include "pose_instability_detector.hpp"

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <string>

PoseInstabilityDetector::PoseInstabilityDetector(const rclcpp::NodeOptions & options)
: Node("pose_instability_detector", options),
  timer_period_(this->declare_parameter<double>("timer_period")),
  heading_velocity_maximum_(this->declare_parameter<double>("heading_velocity_maximum")),
  heading_velocity_scale_factor_tolerance_(
    this->declare_parameter<double>("heading_velocity_scale_factor_tolerance")),
  angular_velocity_maximum_(this->declare_parameter<double>("angular_velocity_maximum")),
  angular_velocity_scale_factor_tolerance_(
    this->declare_parameter<double>("angular_velocity_scale_factor_tolerance")),
  angular_velocity_bias_tolerance_(
    this->declare_parameter<double>("angular_velocity_bias_tolerance")),
  pose_estimator_longitudinal_tolerance_(
    this->declare_parameter<double>("pose_estimator_longitudinal_tolerance")),
  pose_estimator_lateral_tolerance_(
    this->declare_parameter<double>("pose_estimator_lateral_tolerance")),
  pose_estimator_vertical_tolerance_(
    this->declare_parameter<double>("pose_estimator_vertical_tolerance")),
  pose_estimator_angular_tolerance_(
    this->declare_parameter<double>("pose_estimator_angular_tolerance"))
{
  // Define subscibers, publishers and a timer.
  odometry_sub_ = this->create_subscription<Odometry>(
    "~/input/odometry", 10,
    std::bind(&PoseInstabilityDetector::callback_odometry, this, std::placeholders::_1));

  twist_sub_ = this->create_subscription<TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(&PoseInstabilityDetector::callback_twist, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::duration<double>(timer_period_),
    std::bind(&PoseInstabilityDetector::callback_timer, this));

  diff_pose_pub_ = this->create_publisher<PoseStamped>("~/debug/diff_pose", 10);
  diagnostics_pub_ = this->create_publisher<DiagnosticArray>("/diagnostics", 10);
}

void PoseInstabilityDetector::callback_odometry(Odometry::ConstSharedPtr odometry_msg_ptr)
{
  latest_odometry_ = *odometry_msg_ptr;
}

void PoseInstabilityDetector::callback_twist(
  TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  twist_buffer_.push_back(*twist_msg_ptr);
}

void PoseInstabilityDetector::callback_timer()
{
  // odometry callback and timer callback has to be called at least once
  if (latest_odometry_ == std::nullopt) {
    return;
  }
  if (prev_odometry_ == std::nullopt) {
    prev_odometry_ = latest_odometry_;
    return;
  }

  // twist callback has to be called at least once
  if (twist_buffer_.empty()) {
    return;
  }

  // time variables
  const rclcpp::Time latest_odometry_time = rclcpp::Time(latest_odometry_->header.stamp);
  const rclcpp::Time prev_odometry_time = rclcpp::Time(prev_odometry_->header.stamp);

  // define lambda function to convert quaternion to rpy
  auto quat_to_rpy = [](const Quaternion & quat) {
    tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(tf2_quat);
    double roll{};
    double pitch{};
    double yaw{};
    mat.getRPY(roll, pitch, yaw);
    return std::make_tuple(roll, pitch, yaw);
  };

  // delete twist data older than prev_odometry_ (but preserve the one right before prev_odometry_)
  while (twist_buffer_.size() > 1) {
    if (rclcpp::Time(twist_buffer_[1].header.stamp) < prev_odometry_time) {
      twist_buffer_.pop_front();
    } else {
      break;
    }
  }

  // dead reckoning from prev_odometry_ to latest_odometry_
  PoseStamped::SharedPtr prev_pose = std::make_shared<PoseStamped>();
  prev_pose->header = prev_odometry_->header;
  prev_pose->pose = prev_odometry_->pose.pose;

  Pose::SharedPtr DR_pose = std::make_shared<Pose>();
  dead_reckon(prev_pose, latest_odometry_time, twist_buffer_, DR_pose);

  // compare dead reckoning pose and latest_odometry_
  const Pose latest_ekf_pose = latest_odometry_->pose.pose;
  const Pose ekf_to_DR = tier4_autoware_utils::inverseTransformPose(*DR_pose, latest_ekf_pose);
  const geometry_msgs::msg::Point pos = ekf_to_DR.position;
  const auto [ang_x, ang_y, ang_z] = quat_to_rpy(ekf_to_DR.orientation);
  const std::vector<double> values = {pos.x, pos.y, pos.z, ang_x, ang_y, ang_z};

  // publish diff_pose for debug
  PoseStamped diff_pose;
  diff_pose.header.stamp = latest_odometry_time;
  diff_pose.header.frame_id = "base_link";
  diff_pose.pose = ekf_to_DR;
  diff_pose_pub_->publish(diff_pose);

  // publish diagnostics
  calculate_threshold((latest_odometry_time - prev_odometry_time).seconds());

  const std::vector<double> thresholds = {threshold_diff_position_x_, threshold_diff_position_y_,
                                          threshold_diff_position_z_, threshold_diff_angle_x_,
                                          threshold_diff_angle_y_,    threshold_diff_angle_z_};

  const std::vector<std::string> labels = {"diff_position_x", "diff_position_y", "diff_position_z",
                                           "diff_angle_x",    "diff_angle_y",    "diff_angle_z"};

  DiagnosticStatus status;
  status.name = "localization: pose_instability_detector";
  status.hardware_id = this->get_name();
  bool all_ok = true;

  for (size_t i = 0; i < values.size(); ++i) {
    const bool ok = (std::abs(values[i]) < thresholds[i]);
    all_ok &= ok;
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = labels[i] + ":threshold";
    kv.value = std::to_string(thresholds[i]);
    status.values.push_back(kv);
    kv.key = labels[i] + ":value";
    kv.value = std::to_string(values[i]);
    status.values.push_back(kv);
    kv.key = labels[i] + ":status";
    kv.value = (ok ? "OK" : "WARN");
    status.values.push_back(kv);
  }
  status.level = (all_ok ? DiagnosticStatus::OK : DiagnosticStatus::WARN);
  status.message = (all_ok ? "OK" : "WARN");

  DiagnosticArray diagnostics;
  diagnostics.header.stamp = latest_odometry_time;
  diagnostics.status.emplace_back(status);
  diagnostics_pub_->publish(diagnostics);

  // prepare for next loop
  prev_odometry_ = latest_odometry_;
}

void PoseInstabilityDetector::calculate_threshold(double interval_sec)
{
  const double longitudinal_difference =
    heading_velocity_maximum_ * heading_velocity_scale_factor_tolerance_ * 0.01 * interval_sec;
  const double lateral_difference =
    heading_velocity_maximum_ * (1 + heading_velocity_scale_factor_tolerance_ * 0.01) *
    sin(
      0.5 *
      (angular_velocity_maximum_ * angular_velocity_scale_factor_tolerance_ * 0.01 +
       angular_velocity_bias_tolerance_) *
      interval_sec);
  const double vertical_difference =
    heading_velocity_maximum_ * (1 + heading_velocity_scale_factor_tolerance_ * 0.01) *
    sin(
      0.5 *
      (angular_velocity_maximum_ * angular_velocity_scale_factor_tolerance_ * 0.01 +
       angular_velocity_bias_tolerance_) *
      interval_sec);

  const double roll_difference =
    (angular_velocity_maximum_ * angular_velocity_scale_factor_tolerance_ * 0.01 +
     angular_velocity_bias_tolerance_) *
    interval_sec;
  const double pitch_difference =
    (angular_velocity_maximum_ * angular_velocity_scale_factor_tolerance_ * 0.01 +
     angular_velocity_bias_tolerance_) *
    interval_sec;
  const double yaw_difference =
    (angular_velocity_maximum_ * angular_velocity_scale_factor_tolerance_ * 0.01 +
     angular_velocity_bias_tolerance_) *
    interval_sec;

  threshold_diff_position_x_ = longitudinal_difference + pose_estimator_longitudinal_tolerance_;
  threshold_diff_position_y_ = lateral_difference + pose_estimator_lateral_tolerance_;
  threshold_diff_position_z_ = vertical_difference + pose_estimator_vertical_tolerance_;
  threshold_diff_angle_x_ = roll_difference + pose_estimator_angular_tolerance_;
  threshold_diff_angle_y_ = pitch_difference + pose_estimator_angular_tolerance_;
  threshold_diff_angle_z_ = yaw_difference + pose_estimator_angular_tolerance_;
}

void PoseInstabilityDetector::dead_reckon(
  PoseStamped::SharedPtr & initial_pose, const rclcpp::Time & end_time,
  const std::deque<TwistWithCovarianceStamped> & twist_deque, Pose::SharedPtr & estimated_pose)
{
  // get start time
  rclcpp::Time start_time = rclcpp::Time(initial_pose->header.stamp);

  // initialize estimated_pose
  estimated_pose->position = initial_pose->pose.position;
  estimated_pose->orientation = initial_pose->pose.orientation;

  // define lambda function to convert quaternion to rpy
  auto quat_to_rpy = [](const Quaternion & quat) {
    tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(tf2_quat);
    double roll{};
    double pitch{};
    double yaw{};
    mat.getRPY(roll, pitch, yaw);
    return std::make_tuple(roll, pitch, yaw);
  };

  // cut out necessary twist data
  std::deque<TwistWithCovarianceStamped> sliced_twist_deque =
    clip_out_necessary_twist(twist_deque, start_time, end_time);

  // dead reckoning
  rclcpp::Time prev_odometry_time = rclcpp::Time(sliced_twist_deque.front().header.stamp);

  for (size_t i = 1; i < sliced_twist_deque.size(); ++i) {
    const rclcpp::Time curr_time = rclcpp::Time(sliced_twist_deque[i].header.stamp);
    const double time_diff_sec = (curr_time - prev_odometry_time).seconds();

    const Twist twist = sliced_twist_deque[i].twist.twist;

    // quat to rpy
    auto [ang_x, ang_y, ang_z] = quat_to_rpy(estimated_pose->orientation);

    // average quaternion of two frames
    tf2::Quaternion average_quat;
    average_quat.setRPY(
      ang_x + 0.5 * twist.angular.x * time_diff_sec, ang_y + 0.5 * twist.angular.y * time_diff_sec,
      ang_z + 0.5 * twist.angular.z * time_diff_sec);

    // Convert twist to world frame (take average of two frames)
    tf2::Vector3 linear_velocity(twist.linear.x, twist.linear.y, twist.linear.z);
    linear_velocity = tf2::quatRotate(average_quat, linear_velocity);

    // xyz update
    estimated_pose->position.x += linear_velocity.x() * time_diff_sec;
    estimated_pose->position.y += linear_velocity.y() * time_diff_sec;
    estimated_pose->position.z += linear_velocity.z() * time_diff_sec;

    // rpy update
    ang_x += twist.angular.x * time_diff_sec;
    ang_y += twist.angular.y * time_diff_sec;
    ang_z += twist.angular.z * time_diff_sec;
    tf2::Quaternion quat;
    quat.setRPY(ang_x, ang_y, ang_z);
    estimated_pose->orientation.x = quat.x();
    estimated_pose->orientation.y = quat.y();
    estimated_pose->orientation.z = quat.z();
    estimated_pose->orientation.w = quat.w();

    // update previous variables
    prev_odometry_time = curr_time;
  }
}

std::deque<PoseInstabilityDetector::TwistWithCovarianceStamped>
PoseInstabilityDetector::clip_out_necessary_twist(
  const std::deque<TwistWithCovarianceStamped> & twist_buffer, const rclcpp::Time & start_time,
  const rclcpp::Time & end_time)
{
  // get iterator to the element that is right before start_time (if it does not exist, start_it =
  // twist_buffer.begin())
  auto start_it = twist_buffer.begin();
  for (auto it = twist_buffer.begin(); it != twist_buffer.end(); ++it) {
    if (rclcpp::Time(it->header.stamp) > start_time) {
      break;
    }
    start_it = it;
  }

  // get iterator to the element that is right after end_time (if it does not exist, end_it =
  // twist_buffer.end())
  auto end_it = twist_buffer.end();
  end_it--;
  for (auto it = end_it; it != twist_buffer.begin(); --it) {
    if (rclcpp::Time(it->header.stamp) < end_time) {
      break;
    }
    end_it = it;
  }

  // Create result deque
  std::deque<TwistWithCovarianceStamped> result_deque(start_it, end_it);

  // If the result deque has only one element, return a deque that starts and ends with the same
  // element
  if (result_deque.size() == 1) {
    TwistWithCovarianceStamped twist = *start_it;
    result_deque.clear();

    twist.header.stamp.sec = static_cast<int32_t>(start_time.seconds());
    twist.header.stamp.nanosec = static_cast<uint32_t>(start_time.nanoseconds() % 1000000000);
    result_deque.push_back(twist);

    twist.header.stamp.sec = static_cast<int32_t>(end_time.seconds());
    twist.header.stamp.nanosec = static_cast<uint32_t>(end_time.nanoseconds() % 1000000000);
    result_deque.push_back(twist);

    return result_deque;
  }

  // If the first element is later than start_time, add the first element to the front of the
  // result_deque
  if (rclcpp::Time(result_deque.front().header.stamp) > start_time) {
    TwistWithCovarianceStamped start_twist = *start_it;
    start_twist.header.stamp.sec = static_cast<int32_t>(start_time.seconds());
    start_twist.header.stamp.nanosec = static_cast<uint32_t>(start_time.nanoseconds() % 1000000000);
    result_deque.push_front(start_twist);
  } else {
    // If the first element is earlier than start_time, interpolate the first element
    rclcpp::Time time0 = rclcpp::Time(result_deque[0].header.stamp);
    rclcpp::Time time1 = rclcpp::Time(result_deque[1].header.stamp);
    double ratio = (start_time - time0).seconds() / (time1 - time0).seconds();
    Twist twist0 = result_deque[0].twist.twist;
    Twist twist1 = result_deque[1].twist.twist;
    result_deque[0].twist.twist.linear.x = twist1.linear.x * ratio + twist0.linear.x * (1 - ratio);
    result_deque[0].twist.twist.linear.y = twist1.linear.y * ratio + twist0.linear.y * (1 - ratio);
    result_deque[0].twist.twist.linear.z = twist1.linear.z * ratio + twist0.linear.z * (1 - ratio);
    result_deque[0].twist.twist.angular.x =
      twist1.angular.x * ratio + twist0.angular.x * (1 - ratio);
    result_deque[0].twist.twist.angular.y =
      twist1.angular.y * ratio + twist0.angular.y * (1 - ratio);
    result_deque[0].twist.twist.angular.z =
      twist1.angular.z * ratio + twist0.angular.z * (1 - ratio);

    result_deque[0].header.stamp.sec = static_cast<int32_t>(start_time.seconds());
    result_deque[0].header.stamp.nanosec =
      static_cast<uint32_t>(start_time.nanoseconds() % 1000000000);
  }

  // If the last element is earlier than end_time, add the last element to the back of the
  // result_deque
  if (rclcpp::Time(result_deque.back().header.stamp) < end_time) {
    TwistWithCovarianceStamped end_twist = *end_it;
    end_twist.header.stamp.sec = static_cast<int32_t>(end_time.seconds());
    end_twist.header.stamp.nanosec = static_cast<uint32_t>(end_time.nanoseconds() % 1000000000);
    result_deque.push_back(end_twist);
  } else {
    // If the last element is later than end_time, interpolate the last element
    rclcpp::Time time0 = rclcpp::Time(result_deque[result_deque.size() - 2].header.stamp);
    rclcpp::Time time1 = rclcpp::Time(result_deque[result_deque.size() - 1].header.stamp);
    double ratio = (end_time - time0).seconds() / (time1 - time0).seconds();
    Twist twist0 = result_deque[result_deque.size() - 2].twist.twist;
    Twist twist1 = result_deque[result_deque.size() - 1].twist.twist;
    result_deque[result_deque.size() - 1].twist.twist.linear.x =
      twist1.linear.x * ratio + twist0.linear.x * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.linear.y =
      twist1.linear.y * ratio + twist0.linear.y * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.linear.z =
      twist1.linear.z * ratio + twist0.linear.z * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.angular.x =
      twist1.angular.x * ratio + twist0.angular.x * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.angular.y =
      twist1.angular.y * ratio + twist0.angular.y * (1 - ratio);
    result_deque[result_deque.size() - 1].twist.twist.angular.z =
      twist1.angular.z * ratio + twist0.angular.z * (1 - ratio);

    result_deque[result_deque.size() - 1].header.stamp.sec =
      static_cast<int32_t>(end_time.seconds());
    result_deque[result_deque.size() - 1].header.stamp.nanosec =
      static_cast<uint32_t>(end_time.nanoseconds() % 1000000000);
  }

  return result_deque;
}
