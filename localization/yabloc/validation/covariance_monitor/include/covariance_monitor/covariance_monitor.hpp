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
#include <eigen3/Eigen/Dense>
#include <pcdless_common/synchro_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace pcdless::covariance_monitor
{
class CovarianceMonitor : public rclcpp::Node
{
public:
  using String = std_msgs::msg::String;
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  CovarianceMonitor();

private:
  common::SynchroSubscriber<ParticleArray, PoseStamped>::SharedPtr synchro_subscriber_;
  rclcpp::Publisher<String>::SharedPtr pub_diagnostic_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_cov_stamped_;

  void particle_and_pose(const ParticleArray & particles, const PoseStamped & pose);
  Eigen::Vector3f compute_std(
    const ParticleArray & array, const Eigen::Quaternionf & orientation) const;
  void publish_pose_cov_stamped(const PoseStamped & pose, const Eigen::Vector3f & covariance);
};

}  // namespace pcdless::covariance_monitor