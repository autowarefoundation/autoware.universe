// Copyright 2022 Tier IV, Inc.
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

#include "tier4_autoware_utils/vehicle/vehicle_state_checker.hpp"

#include "tier4_autoware_utils/trajectory/trajectory.hpp"

#include <string>

namespace tier4_autoware_utils
{
VehicleStateChecker::VehicleStateChecker(rclcpp::Node * node)
: clock_(node->get_clock()), logger_(node->get_logger())
{
  using std::placeholders::_1;

  sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1),
    std::bind(&VehicleStateChecker::onOdom, this, _1));

  sub_trajectory_ = node->create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS(1),
    std::bind(&VehicleStateChecker::onTrajectory, this, _1));

  sub_velocity_status_ = node->create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS(1),
    std::bind(&VehicleStateChecker::onVelocityStatus, this, _1));
}

bool VehicleStateChecker::checkStopped(const double stop_duration = 0.0) const
{
  if (twist_buffer_.empty()) {
    return false;
  }

  // Get velocities within stop_duration
  const auto now = clock_->now();
  std::vector<double> vs;
  for (const auto & velocity : twist_buffer_) {
    vs.push_back(velocity.twist.linear.x);

    const auto time_diff = now - velocity.header.stamp;
    if (time_diff.seconds() >= stop_duration) {
      break;
    }
  }

  // Check all velocities
  constexpr double stop_velocity = 1e-3;
  for (const auto & v : vs) {
    if (v >= stop_velocity) {
      return false;
    }
  }

  return true;
}

bool VehicleStateChecker::checkStoppedAtStopPoint(const double stop_duration = 0.0) const
{
  if (!odometry_ptr_ || !trajectory_ptr_) {
    return false;
  }

  if (!checkStopped(stop_duration)) {
    return false;
  }

  const auto & p = odometry_ptr_->pose.pose.position;
  const auto idx = searchZeroVelocityIndex(trajectory_ptr_->points);

  if (!idx) {
    return false;
  }

  return calcSignedArcLength(trajectory_ptr_->points, p, idx.get()) < th_arrived_distance_m;
}

bool VehicleStateChecker::checkStoppedWithoutLocalization(const double stop_duration = 0.0) const
{
  if (velocity_status_buffer_.empty()) {
    return false;
  }

  // Get velocities within stop_duration
  const auto now = clock_->now();
  std::vector<double> vs;
  for (const auto & velocity : velocity_status_buffer_) {
    vs.push_back(velocity.longitudinal_velocity);

    const auto time_diff = now - velocity.header.stamp;
    if (time_diff.seconds() >= stop_duration) {
      break;
    }
  }

  // Check all velocities
  constexpr double stop_velocity = 1e-3;
  for (const auto & v : vs) {
    if (v >= stop_velocity) {
      return false;
    }
  }

  return true;
}

void VehicleStateChecker::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odometry_ptr_ = msg;

  auto current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
  current_velocity->header = msg->header;
  current_velocity->twist = msg->twist.twist;

  twist_buffer_.push_front(*current_velocity);

  const auto now = clock_->now();
  while (true) {
    // Check oldest data time
    const auto time_diff = now - twist_buffer_.back().header.stamp;

    // Finish when oldest data is newer than threshold
    if (time_diff.seconds() <= velocity_buffer_time_sec) {
      break;
    }

    // Remove old data
    twist_buffer_.pop_back();
  }
}

void VehicleStateChecker::onVelocityStatus(const VelocityReport::SharedPtr msg)
{
  velocity_status_buffer_.push_front(*msg);

  const auto now = clock_->now();
  while (true) {
    // Check oldest data time
    const auto time_diff = now - velocity_status_buffer_.back().header.stamp;

    // Finish when oldest data is newer than threshold
    if (time_diff.seconds() <= velocity_buffer_time_sec) {
      break;
    }

    // Remove old data
    velocity_status_buffer_.pop_back();
  }
}

void VehicleStateChecker::onTrajectory(const Trajectory::SharedPtr msg) { trajectory_ptr_ = msg; }
}  // namespace tier4_autoware_utils
