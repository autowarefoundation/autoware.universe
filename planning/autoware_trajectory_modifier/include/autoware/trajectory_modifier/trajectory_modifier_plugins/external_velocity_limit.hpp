// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__EXTERNAL_VELOCITY_LIMIT_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__EXTERNAL_VELOCITY_LIMIT_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <autoware_utils_rclcpp/polling_subscriber.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>

#include <memory>

namespace autoware::trajectory_modifier::plugin
{

namespace detail
{
/// @brief Deceleration magnitude from the message constraints when provided and non-zero,
/// otherwise the default.
double get_external_velocity_limit_deceleration(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit,
  double default_deceleration);
/// @brief Jerk magnitude from the message constraints when provided and non-zero, otherwise the
/// default.
double get_external_velocity_limit_min_jerk(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit, double default_jerk);
}  // namespace detail

class ExternalVelocityLimit : public TrajectoryModifierPluginBase
{
public:
  ProcessingResult process(TrajectoryPoints & traj_points, TrajectoryModifierData & input) override;

  void update_params(const TrajectoryModifierParams & params) override;

  /// @brief How the profile of the latest processed candidate was started (Measured when no
  /// profile was generated).
  [[nodiscard]] detail::ProfileStart last_profile_start() const { return last_profile_start_; }

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  using VelocityLimit = autoware_internal_planning_msgs::msg::VelocityLimit;

  std::shared_ptr<autoware_utils_rclcpp::InterProcessPollingSubscriber<VelocityLimit>>
    velocity_limit_sub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::StringStamped>::SharedPtr profile_reset_pub_;
  detail::VelocityLimitConstraints constraints_;
  detail::FixedProfileMemory fixed_profile_;
  detail::ProfileStart last_profile_start_{detail::ProfileStart::Measured};
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__EXTERNAL_VELOCITY_LIMIT_HPP_
