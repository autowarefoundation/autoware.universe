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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/map_velocity_limits.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::trajectory_modifier::plugin
{

namespace
{
autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides
make_velocity_limit_overrides(const TrajectoryModifierParams & params)
{
  autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides overrides;
  const auto & ids = params.map_velocity_limits.limit_velocity_from_map_debug_lanelet_ids;
  const auto & velocities = params.map_velocity_limits.limit_velocity_from_map_debug_max_velocities;
  if (ids.size() != velocities.size()) {
    throw std::invalid_argument(
      "limit_velocity_from_map_debug_lanelet_ids and "
      "limit_velocity_from_map_debug_max_velocities must have equal lengths");
  }
  for (std::size_t index = 0; index < ids.size(); ++index) {
    if (!std::isfinite(velocities[index]) || velocities[index] < 0.0) {
      throw std::invalid_argument(
        "limit_velocity_from_map_debug_max_velocities must contain finite non-negative values");
    }
    if (!overrides.emplace(ids[index], velocities[index]).second) {
      throw std::invalid_argument(
        "limit_velocity_from_map_debug_lanelet_ids must not contain duplicates");
    }
  }
  return overrides;
}
}  // namespace

void MapVelocityLimits::on_initialize(const TrajectoryModifierParams & params)
{
  update_params(params);
  profile_reset_pub_ =
    get_node_ptr()->create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
      "~/debug/velocity_profile_reset", rclcpp::QoS{10}.reliable().durability_volatile());
}

void MapVelocityLimits::update_params(const TrajectoryModifierParams & params)
{
  try {
    limit_overrides_ = make_velocity_limit_overrides(params);
  } catch (const std::invalid_argument & error) {
    enabled_ = false;
    limit_overrides_.clear();
    fixed_profile_.clear();
    RCLCPP_ERROR(
      get_node_ptr()->get_logger(),
      "%s: invalid map velocity limit parameters: %s; disabling map velocity limits",
      get_short_name().c_str(), error.what());
    return;
  }
  enabled_ = params.use_map_velocity_limits;
  constraints_ = detail::make_velocity_limit_constraints(params);
  fixed_profile_.set_parameters(detail::make_fixed_profile_parameters(params));
  if (!enabled_) {
    fixed_profile_.clear();
  }
}

bool MapVelocityLimits::is_trajectory_modification_required(
  [[maybe_unused]] const TrajectoryPoints & traj_points, const TrajectoryModifierData & input)
{
  if (
    !input.lanelet_map_bin || !input.route ||
    (input.current_odometry && input.current_odometry->twist.twist.linear.x < 0.1)) {
    return false;
  }
  if (!extended_route_handler_ || previous_route_uuid_ != input.route->uuid) {
    auto handler = std::make_shared<autoware::avoidance_target_detector::ExtendedRouteHandler>(
      *input.lanelet_map_bin, *input.route);
    handler->create_map();
    extended_route_handler_ = handler;
    previous_route_uuid_ = input.route->uuid;
  }
  return true;
}

ProcessingResult MapVelocityLimits::process(
  TrajectoryPoints & traj_points, TrajectoryModifierData & input)
{
  last_profile_start_ = detail::ProfileStart::Measured;
  if (!enabled_ || traj_points.empty() || !input.current_odometry) {
    return ProcessingResult::Unchanged;
  }
  if (!is_trajectory_modification_required(traj_points, input)) {
    fixed_profile_.store(input, traj_points);
    return ProcessingResult::Unchanged;
  }
  const auto start = fixed_profile_.start(input);
  last_profile_start_ = start.start;
  if (detail::is_reset(start.start)) {
    RCLCPP_DEBUG(
      get_node_ptr()->get_logger(), "%s: candidate %zu restarted from the measured ego state (%s)",
      get_short_name().c_str(), input.candidate_index, detail::to_string(start.start));
    autoware_internal_debug_msgs::msg::StringStamped reset;
    reset.stamp = get_clock()->now();
    reset.data = "plugin=" + get_name() + " candidate=" + std::to_string(input.candidate_index) +
                 " reason=" + detail::to_string(start.start);
    profile_reset_pub_->publish(reset);
  }
  const auto result = detail::apply_velocity_limits(
    traj_points, constraints_,
    [this](const geometry_msgs::msg::Point & position) {
      return extended_route_handler_->get_velocity_limit(position, limit_overrides_);
    },
    start.options);
  fixed_profile_.store(input, traj_points);
  return result.status;
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::MapVelocityLimits,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
