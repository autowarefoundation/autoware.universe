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

#include "pose_instability_detector.hpp"

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

PoseInstabilityDetector::PoseInstabilityDetector()
: Node("pose_instability_detector"),
  threshold_linear_x_(this->declare_parameter<double>("threshold_linear_x")),
  threshold_linear_y_(this->declare_parameter<double>("threshold_linear_y")),
  threshold_linear_z_(this->declare_parameter<double>("threshold_linear_z")),
  threshold_angular_x_(this->declare_parameter<double>("threshold_angular_x")),
  threshold_angular_y_(this->declare_parameter<double>("threshold_angular_y")),
  threshold_angular_z_(this->declare_parameter<double>("threshold_angular_z"))
{
  odometry_sub_ = this->create_subscription<Odometry>(
    "~/input/odometry", 10,
    std::bind(&PoseInstabilityDetector::callback_odometry, this, std::placeholders::_1));

  twist_sub_ = this->create_subscription<TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(&PoseInstabilityDetector::callback_twist, this, std::placeholders::_1));

  const double interval_sec = this->declare_parameter<double>("interval_sec");
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(interval_sec),
    std::bind(&PoseInstabilityDetector::callback_timer, this));

  diagnostics_pub_ = this->create_publisher<DiagnosticArray>("/diagnostics", 10);
}

void PoseInstabilityDetector::callback_odometry(Odometry::ConstSharedPtr odometry_msg_ptr)
{
  RCLCPP_INFO_STREAM(get_logger(), "callback_odometry:" << odometry_msg_ptr->header.stamp.sec);
  latest_odometry_ = *odometry_msg_ptr;
}

void PoseInstabilityDetector::callback_twist(
  TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  RCLCPP_INFO_STREAM(get_logger(), "callback_twist:" << twist_msg_ptr->header.stamp.sec);
  twist_buffer_.push_back(*twist_msg_ptr);
}

void PoseInstabilityDetector::callback_timer()
{
  RCLCPP_INFO_STREAM(get_logger(), "callback_timer");

  auto quat_to_rpy = [](const Quaternion & quat) {
    tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(tf2_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return std::make_tuple(roll, pitch, yaw);
  };

  Pose pose = prev_odometry_.pose.pose;
  rclcpp::Time time = rclcpp::Time(prev_odometry_.header.stamp);
  for (const TwistWithCovarianceStamped & twist_with_cov : twist_buffer_) {
    const Twist twist = twist_with_cov.twist.twist;

    const rclcpp::Time curr_time = rclcpp::Time(twist_with_cov.header.stamp);
    if (curr_time > latest_odometry_.header.stamp) {
      break;
    }

    const rclcpp::Duration time_diff = curr_time - rclcpp::Time(time);
    const double time_diff_sec = time_diff.seconds();
    if (time_diff_sec < 0.0) {
      RCLCPP_WARN_STREAM(get_logger(), "time_diff_sec(" << time_diff_sec << ") < 0.0");
      continue;
    }

    // quat to rpy
    auto [ang_x, ang_y, ang_z] = quat_to_rpy(pose.orientation);

    // rpy update
    ang_x += twist.angular.x * time_diff_sec;
    ang_y += twist.angular.y * time_diff_sec;
    ang_z += twist.angular.z * time_diff_sec;
    tf2::Quaternion quat;
    quat.setRPY(ang_x, ang_y, ang_z);

    // Convert twist to world frame
    tf2::Vector3 linear_velocity(twist.linear.x, twist.linear.y, twist.linear.z);
    linear_velocity = tf2::quatRotate(quat, linear_velocity);

    // update
    pose.position.x += linear_velocity.x() * time_diff_sec;
    pose.position.y += linear_velocity.y() * time_diff_sec;
    pose.position.z += linear_velocity.z() * time_diff_sec;
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    time = curr_time;
  }

  // compare pose and latest_odometry_
  const Pose latest_ekf_pose = latest_odometry_.pose.pose;
  PoseStamped ekf_to_odom;
  ekf_to_odom.pose = tier4_autoware_utils::inverseTransformPose(pose, latest_ekf_pose);
  ekf_to_odom.header.stamp = latest_odometry_.header.stamp;
  ekf_to_odom.header.frame_id = "base_link";
  const auto [ang_x, ang_y, ang_z] = quat_to_rpy(ekf_to_odom.pose.orientation);

  // publish diagnostics
  const std::vector<double> values = {
    ekf_to_odom.pose.position.x,
    ekf_to_odom.pose.position.y,
    ekf_to_odom.pose.position.z,
    ang_x,
    ang_y,
    ang_z};

  const std::vector<double> thresholds = {threshold_linear_x_,  threshold_linear_y_,
                                          threshold_linear_z_,  threshold_angular_x_,
                                          threshold_angular_y_, threshold_angular_z_};

  const std::vector<std::string> labels = {"linear_x",  "linear_y",  "linear_z",
                                           "angular_x", "angular_y", "angular_z"};

  DiagnosticArray diagnostics;
  diagnostics.header.stamp = latest_odometry_.header.stamp;

  DiagnosticStatus status;
  status.name = "pose_instability_detector";
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
  diagnostics.status.emplace_back(status);

  diagnostics_pub_->publish(diagnostics);

  // prepare for next loop
  prev_odometry_ = latest_odometry_;
  twist_buffer_.clear();
}
