// Copyright 2023 Autoware Foundation
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

#ifndef POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_EAGLEYE_HPP_
#define POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_EAGLEYE_HPP_
#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace pose_estimator_manager::sub_manager
{
class SubManagerEagleye : public BasePoseEstimatorSubManager
{
public:
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  explicit SubManagerEagleye(rclcpp::Node * node) : BasePoseEstimatorSubManager(node)
  {
    eagleye_is_enabled_ = true;

    using std::placeholders::_1;
    auto on_pose = std::bind(&SubManagerEagleye::on_pose, this, _1);
    sub_pose_ =
      node->create_subscription<PoseCovStamped>("~/input/eagleye/pose_with_covariance", 5, on_pose);
    pub_pose_ = node->create_publisher<PoseCovStamped>("~/output/eagleye/pose_with_covariance", 5);
  }

  void set_enable(bool enabled) override { eagleye_is_enabled_ = enabled; }

private:
  bool eagleye_is_enabled_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_;

  void on_pose(PoseCovStamped::ConstSharedPtr msg)
  {
    if (eagleye_is_enabled_) {
      pub_pose_->publish(*msg);
    }
  }
};
}  // namespace pose_estimator_manager::sub_manager

#endif  // POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_EAGLEYE_HPP_
