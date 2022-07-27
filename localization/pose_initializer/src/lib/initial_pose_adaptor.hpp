// Copyright 2022 Autoware Foundation
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

#ifndef LIB__INITIAL_POSE_ADAPTOR_HPP_
#define LIB__INITIAL_POSE_ADAPTOR_HPP_

#include "map_fit_module.hpp"

#include <autoware_ad_api_specs/localization.hpp>
#include <component_interface_utils/macros.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using Initialize = autoware_ad_api::localization::Initialize;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

class InitialPoseAdaptor : public rclcpp::Node
{
public:
  InitialPoseAdaptor();

private:
  MapFitModule map_fit_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  component_interface_utils::Client<Initialize>::SharedPtr cli_initialize_;
  std::array<double, 36> rviz_particle_covariance_;

  void OnInitialPose(PoseWithCovarianceStamped::ConstSharedPtr msg);
};

#endif  // LIB__INITIAL_POSE_ADAPTOR_HPP_
