// Copyright 2023 TIER IV, Inc.
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

#pragma once
#include <eigen3/Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

namespace pcdless::ape_monitor
{
class AbsolutePoseError : public rclcpp::Node
{
public:
  using String = std_msgs::msg::String;
  using Odometry = nav_msgs::msg::Odometry;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  AbsolutePoseError();

private:
  struct Reference
  {
    Reference() { poses_.reserve(3e4); }
    rclcpp::Time start_stamp_;
    rclcpp::Time end_stamp_;
    std::vector<PoseStamped> poses_;
  };

  rclcpp::Publisher<String>::SharedPtr pub_string_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_cov_stamped_;
  std::vector<Reference> references_;

  Eigen::Vector2f compute_ape(const Reference & ref, const PoseCovStamped & pose_cov) const;
  void on_pose(const PoseCovStamped & pose_cov);
  void load_reference_rosbag(const std::string & bag_file);
};
}  // namespace pcdless::ape_monitor