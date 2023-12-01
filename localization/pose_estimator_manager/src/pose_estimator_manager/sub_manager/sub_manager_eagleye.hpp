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

#include <memory>

namespace pose_estimator_manager::sub_manager
{
class SubManagerEagleye : public BasePoseEstimatorSubManager
{
public:
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  explicit SubManagerEagleye(
    rclcpp::Node * node, const std::shared_ptr<const SharedData> shared_data)
  : BasePoseEstimatorSubManager(node, shared_data)
  {
    eagleye_is_enabled_ = true;
    pub_pose_ = node->create_publisher<PoseCovStamped>("~/output/eagleye/pose_with_covariance", 5);

    shared_data_->eagleye_output_pose_cov.set_callback(
      [this](PoseCovStamped::ConstSharedPtr msg) -> void {
        if (eagleye_is_enabled_) {
          pub_pose_->publish(*msg);
        }
      });
  }

  void set_enable(bool enabled) override { eagleye_is_enabled_ = enabled; }

  // void callback() override
  // {
  //   if (!shared_data_->eagleye_output_pose_cov.updated) {
  //     return;
  //   }
  //   if (eagleye_is_enabled_) {
  //     pub_pose_->publish(*shared_data_->eagleye_output_pose_cov());
  //   }
  // }

private:
  bool eagleye_is_enabled_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_;
};
}  // namespace pose_estimator_manager::sub_manager

#endif  // POSE_ESTIMATOR_MANAGER__SUB_MANAGER__SUB_MANAGER_EAGLEYE_HPP_
