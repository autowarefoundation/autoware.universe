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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__MAP_VELOCITY_LIMITS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__MAP_VELOCITY_LIMITS_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <autoware/avoidance_target_detector/boundary.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>

#include <memory>

namespace autoware::trajectory_modifier::plugin
{
using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::TrajectoryModifierParams;
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using ModifierParams = trajectory_modifier_params::Params;

class MapVelocityLimits : public TrajectoryModifierPluginBase
{
public:
  MapVelocityLimits() = default;

  ProcessingResult process(TrajectoryPoints & traj_points, TrajectoryModifierData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const TrajectoryModifierData & input);

  void update_params(const TrajectoryModifierParams & params) override;

  /// @brief How the profile of the latest processed candidate was started (Measured when no
  /// profile was generated).
  [[nodiscard]] detail::ProfileStart last_profile_start() const { return last_profile_start_; }

protected:
  autoware_planning_msgs::msg::LaneletRoute::_uuid_type previous_route_uuid_;
  std::shared_ptr<autoware::avoidance_target_detector::ExtendedRouteHandler>
    extended_route_handler_;
  autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides
    limit_overrides_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::StringStamped>::SharedPtr profile_reset_pub_;
  detail::VelocityLimitConstraints constraints_;
  detail::FixedProfileMemory fixed_profile_;
  detail::ProfileStart last_profile_start_{detail::ProfileStart::Measured};

  void on_initialize(const TrajectoryModifierParams & params) override;
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__MAP_VELOCITY_LIMITS_HPP_
