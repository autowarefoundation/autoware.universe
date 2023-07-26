// Copyright 2023 TIER IV
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

#include "availability_module.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

AvailabilityModule::AvailabilityModule(rclcpp::Node * node)
: clock_(node->get_clock()),
  latest_yabloc_pose_stamp_ptr_(nullptr),
  timestamp_threshold_(node->declare_parameter<double>("availability/timestamp_threshold", 1.0))
{
  sub_yabloc_pose_ = node->create_subscription<PoseWithCovarianceStamped>(
    "yabloc_pose", 10,
    [this](PoseWithCovarianceStamped::ConstSharedPtr msg) { on_yabloc_pose(msg); });
}

bool AvailabilityModule::is_available() const
{
  if (latest_yabloc_pose_stamp_ptr_ == nullptr) {
    return false;
  }

  const auto now = clock_->now();

  const auto diff_pose = now - *latest_yabloc_pose_stamp_ptr_;
  const auto diff_pose_sec = diff_pose.seconds();
  bool is_pose_available = diff_pose_sec < timestamp_threshold_;

  if (is_pose_available) {
    return true;
  }

  return false;
}

void AvailabilityModule::on_yabloc_pose(const PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  latest_yabloc_pose_stamp_ptr_ = std::make_shared<rclcpp::Time>(rclcpp::Time(msg->header.stamp));
}
