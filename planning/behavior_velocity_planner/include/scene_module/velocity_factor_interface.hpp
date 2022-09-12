
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

#ifndef SCENE_MODULE__VELOCITY_FACTOR_INTERFACE_HPP_
#define SCENE_MODULE__VELOCITY_FACTOR_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_ad_api_msgs/msg/velocity_factor.hpp>
#include <autoware_ad_api_msgs/msg/velocity_factor_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace planning_api_interface
{

using autoware_ad_api_msgs::msg::VelocityFactor;
using autoware_ad_api_msgs::msg::VelocityFactorArray;
using geometry_msgs::msg::Pose;
using VelocityFactorType = VelocityFactor::_type_type;
using VelocityFactorStatus = VelocityFactor::_status_type;

class VelocityFactorInterface
{
public:
  VelocityFactorInterface(rclcpp::Node * node, const std::string & name);
  void publish(const rclcpp::Time & stamp);
  void update(
    const VelocityFactorType type, const VelocityFactorStatus status, const Pose & pose,
    double distance, const std::string & detail);
  void clear();

private:
  rclcpp::Publisher<VelocityFactorArray>::SharedPtr pub_velocity_factors_;
  std::mutex mutex_;
  VelocityFactorArray velocity_factors_;
};

}  // namespace planning_api_interface

#endif  // SCENE_MODULE__VELOCITY_FACTOR_INTERFACE_HPP_
