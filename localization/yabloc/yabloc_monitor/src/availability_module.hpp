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

#ifndef AVAILABILITY_MODULE_HPP_
#define AVAILABILITY_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class AvailabilityModule
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  AvailabilityModule(rclcpp::Node * node);
  bool is_available() const;

private:
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_yabloc_pose_;
  void on_yabloc_pose(PoseWithCovarianceStamped::ConstSharedPtr msg);

  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp::Time> latest_yabloc_pose_stamp_ptr_;
  double timestamp_threshold_;
};

#endif  // AVAILABILITY_MODULE_HPP_
