// Copyright 2021 Tier IV, Inc.
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

#include "autoware/external_velocity_limit_selector/external_velocity_limit_selector_node.hpp"

#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::external_velocity_limit_selector
{

namespace
{
VelocityLimit getHardestLimit(
  const VelocityLimitTable & velocity_limits,
  const ::external_velocity_limit_selector::Params & node_param)
{
  VelocityLimit hardest_limit{};
  hardest_limit.max_velocity = node_param.max_vel;

  VelocityLimitConstraints normal_constraints{};
  normal_constraints.max_acceleration = node_param.normal.max_acc;
  normal_constraints.min_acceleration = node_param.normal.min_acc;
  normal_constraints.min_jerk = node_param.normal.min_jerk;
  normal_constraints.max_jerk = node_param.normal.max_jerk;

  double hardest_max_velocity = node_param.max_vel;
  double hardest_max_jerk = 0.0;
  double hardest_max_acceleration = 0.0;
  std::string hardest_max_acceleration_key;
  size_t constraint_num = 0;
  size_t acceleration_constraint_num = 0;

  for (const auto & limit : velocity_limits) {
    // guard nan, inf
    const auto max_velocity =
      std::isfinite(limit.second.max_velocity) ? limit.second.max_velocity : node_param.max_vel;

    // find hardest max velocity
    if (max_velocity < hardest_max_velocity) {
      hardest_limit.stamp = limit.second.stamp;
      hardest_limit.sender = limit.first;
      hardest_limit.max_velocity = max_velocity;
      hardest_max_velocity = max_velocity;
    }

    const auto constraints =
      limit.second.use_constraints && std::isfinite(limit.second.constraints.max_jerk)
        ? limit.second.constraints
        : normal_constraints;

    if (limit.second.use_constraints) {
      constraint_num++;
      if (limit.second.constraints.max_acceleration > normal_constraints.max_acceleration) {
        acceleration_constraint_num++;
        hardest_max_acceleration_key = limit.first;
      }
    }

    if (hardest_max_acceleration < limit.second.constraints.max_acceleration) {
      hardest_max_acceleration_key = limit.first;
      hardest_max_acceleration = limit.second.constraints.max_acceleration;
    }

    // find hardest jerk
    if (hardest_max_jerk < constraints.max_jerk) {
      hardest_limit.constraints = constraints;
      hardest_limit.use_constraints = true;
      hardest_max_jerk = constraints.max_jerk;
    }
  }

  if (constraint_num > 0 && constraint_num == acceleration_constraint_num) {
    if (velocity_limits.find(hardest_max_acceleration_key) != velocity_limits.end()) {
      const auto constraints = velocity_limits.at(hardest_max_acceleration_key).constraints;
      hardest_limit.constraints = constraints;
      hardest_limit.use_constraints = true;
    }
  }

  return hardest_limit;
}

std::string getDebugString(const VelocityLimitTable & velocity_limits)
{
  std::ostringstream string_stream;
  string_stream << std::boolalpha << std::fixed << std::setprecision(2);
  for (const auto & limit : velocity_limits) {
    string_stream << "[" << limit.first << "]";
    string_stream << "(";
    string_stream << limit.second.use_constraints << ",";
    string_stream << limit.second.max_velocity << ",";
    string_stream << limit.second.constraints.min_acceleration << ",";
    string_stream << limit.second.constraints.min_jerk << ",";
    string_stream << limit.second.constraints.max_jerk << ")";
  }

  return string_stream.str();
}
}  // namespace

ExternalVelocityLimitSelectorNode::ExternalVelocityLimitSelectorNode(
  const rclcpp::NodeOptions & node_options)
: Node("external_velocity_limit_selector", node_options)
{
  using std::placeholders::_1;
  // Input
  sub_external_velocity_limit_from_api_ = this->create_subscription<VelocityLimit>(
    "input/velocity_limit_from_api", rclcpp::QoS{1}.transient_local(),
    std::bind(&ExternalVelocityLimitSelectorNode::onVelocityLimitFromAPI, this, _1));

  sub_external_velocity_limit_from_internal_ = this->create_subscription<VelocityLimit>(
    "input/velocity_limit_from_internal", rclcpp::QoS{10}.transient_local(),
    std::bind(&ExternalVelocityLimitSelectorNode::onVelocityLimitFromInternal, this, _1));

  sub_velocity_limit_clear_command_ = this->create_subscription<VelocityLimitClearCommand>(
    "input/velocity_limit_clear_command_from_internal", rclcpp::QoS{10}.transient_local(),
    std::bind(&ExternalVelocityLimitSelectorNode::onVelocityLimitClearCommand, this, _1));

  // Output
  pub_external_velocity_limit_ =
    this->create_publisher<VelocityLimit>("output/external_velocity_limit", 1);

  pub_debug_string_ = this->create_publisher<StringStamped>("output/debug", 1);

  // Params
  param_listener_ = std::make_shared<::external_velocity_limit_selector::ParamListener>(
    this->get_node_parameters_interface());
}

void ExternalVelocityLimitSelectorNode::onVelocityLimitFromAPI(
  const VelocityLimit::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "set velocity limit. sender:%s", msg->sender.c_str());
  setVelocityLimitFromAPI(*msg);

  const auto velocity_limit = getCurrentVelocityLimit();
  publishVelocityLimit(velocity_limit);

  publishDebugString();
}

void ExternalVelocityLimitSelectorNode::onVelocityLimitFromInternal(
  const VelocityLimit::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "set velocity limit. sender:%s", msg->sender.c_str());
  setVelocityLimitFromInternal(*msg);

  const auto velocity_limit = getCurrentVelocityLimit();
  publishVelocityLimit(velocity_limit);

  publishDebugString();
}

void ExternalVelocityLimitSelectorNode::onVelocityLimitClearCommand(
  const VelocityLimitClearCommand::ConstSharedPtr msg)
{
  if (!msg->command) {
    return;
  }

  clearVelocityLimit(msg->sender);

  const auto velocity_limit = getCurrentVelocityLimit();
  publishVelocityLimit(velocity_limit);

  publishDebugString();
}

void ExternalVelocityLimitSelectorNode::publishVelocityLimit(const VelocityLimit & velocity_limit)
{
  pub_external_velocity_limit_->publish(velocity_limit);
}

void ExternalVelocityLimitSelectorNode::publishDebugString()
{
  StringStamped debug_string{};
  debug_string.stamp = this->now();
  debug_string.data = getDebugString(velocity_limit_table_);
  pub_debug_string_->publish(debug_string);
}

void ExternalVelocityLimitSelectorNode::setVelocityLimitFromAPI(
  const VelocityLimit & velocity_limit)
{
  const std::string sender = "api";

  if (velocity_limit_table_.count(sender) == 0) {
    velocity_limit_table_.emplace(sender, velocity_limit);
  } else {
    velocity_limit_table_.at(sender) = velocity_limit;
    RCLCPP_DEBUG(get_logger(), "overwrite velocity limit. sender:%s", sender.c_str());
  }

  updateVelocityLimit();
}

void ExternalVelocityLimitSelectorNode::setVelocityLimitFromInternal(
  const VelocityLimit & velocity_limit)
{
  const auto sender = velocity_limit.sender;

  if (velocity_limit_table_.count(sender) == 0) {
    velocity_limit_table_.emplace(sender, velocity_limit);
  } else {
    velocity_limit_table_.at(sender) = velocity_limit;
    RCLCPP_DEBUG(get_logger(), "overwrite velocity limit. sender:%s", sender.c_str());
  }

  updateVelocityLimit();
}

void ExternalVelocityLimitSelectorNode::clearVelocityLimit(const std::string & sender)
{
  if (velocity_limit_table_.empty()) {
    RCLCPP_WARN(get_logger(), "no velocity limit has been set from internal.");
    return;
  }

  velocity_limit_table_.erase(sender);

  updateVelocityLimit();
}

void ExternalVelocityLimitSelectorNode::updateVelocityLimit()
{
  const auto param = param_listener_->get_params();

  if (velocity_limit_table_.empty()) {
    VelocityLimit default_velocity_limit{};
    default_velocity_limit.stamp = this->now();
    default_velocity_limit.max_velocity = param.max_vel;

    hardest_limit_ = default_velocity_limit;

    RCLCPP_DEBUG(
      get_logger(),
      "no velocity limit has been set or latest velocity limit has been already cleared.");

    return;
  }

  hardest_limit_ = getHardestLimit(velocity_limit_table_, param);
}
}  // namespace autoware::external_velocity_limit_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::external_velocity_limit_selector::ExternalVelocityLimitSelectorNode)
