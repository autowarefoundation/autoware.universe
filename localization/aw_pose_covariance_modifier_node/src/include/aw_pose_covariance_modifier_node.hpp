// Copyright 2024 The Autoware Foundation
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
#ifndef AW_POSE_COVARIANCE_MODIFIER_NODE_HPP_
#define AW_POSE_COVARIANCE_MODIFIER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <string>

using namespace std::chrono_literals;

class AWPoseCovarianceModifierNode : public rclcpp::Node
{
public:
  AWPoseCovarianceModifierNode();

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr trusted_pose_with_cov_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr new_pose_estimator_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;

  void trusted_pose_with_cov_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg);
  bool callNDTCovarianceModifier();
private:
  double trusted_pose_rmse_;
  double trusted_pose_yaw_rmse_in_degrees_;
  bool startNDTCovModifier = 0;
};


#endif  // AW_POSE_COVARIANCE_MODIFIER_NODE_HPP_
