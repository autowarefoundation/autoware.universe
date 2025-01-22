// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__LOCALIZATION_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__LOCALIZATION_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_localization_msgs/srv/initialize_localization.hpp>

namespace autoware::component_interface_specs_universe::localization
{

struct Initialize
{
  using Service = tier4_localization_msgs::srv::InitializeLocalization;
  static constexpr char name[] = "/localization/initialize";
};

struct InitializationState
{
  using Message = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  static constexpr char name[] = "/localization/initialization_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct KinematicState
{
  using Message = nav_msgs::msg::Odometry;
  static constexpr char name[] = "/localization/kinematic_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct Acceleration
{
  using Message = geometry_msgs::msg::AccelWithCovarianceStamped;
  static constexpr char name[] = "/localization/acceleration";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace autoware::component_interface_specs_universe::localization

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__LOCALIZATION_HPP_
