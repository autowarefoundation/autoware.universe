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
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace yabloc::modularized_particle_filter
{
class ParticleInitializer : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Marker = visualization_msgs::msg::Marker;

  ParticleInitializer();

private:
  const std::vector<double> cov_xx_yy_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_initialpose_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_initialpose_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;

  void on_initial_pose(const PoseCovStamped & initialpose);

  void publish_range_marker(const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent);
  PoseCovStamped rectify_initial_pose(
    const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent,
    const PoseCovStamped & raw_initialpose);
};
}  // namespace yabloc::modularized_particle_filter