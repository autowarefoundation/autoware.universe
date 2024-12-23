// Copyright 2024 TIER IV, Inc.
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

#include "planning_factor_rviz_plugin.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>

namespace autoware::rviz_plugins
{

using autoware::motion_utils::createDeadLineVirtualWallMarker;
using autoware::motion_utils::createSlowDownVirtualWallMarker;
using autoware::motion_utils::createStopVirtualWallMarker;

void PlanningFactorRvizPlugin::processMessage(
  const tier4_planning_msgs::msg::PlanningFactorArray::ConstSharedPtr msg)
{
  size_t i = 0L;
  for (const auto & factor : msg->factors) {
    for (const auto & control_point : factor.control_points) {
      if (factor.behavior == tier4_planning_msgs::msg::PlanningFactor::STOP) {
        const auto virtual_wall = createStopVirtualWallMarker(
          control_point.pose, factor.module, msg->header.stamp, i++, baselink2front_.getFloat());
        add_marker(std::make_shared<visualization_msgs::msg::MarkerArray>(virtual_wall));
        continue;
      }

      if (factor.behavior == tier4_planning_msgs::msg::PlanningFactor::SLOW_DOWN) {
        const auto virtual_wall = createSlowDownVirtualWallMarker(
          control_point.pose, factor.module, msg->header.stamp, i++, baselink2front_.getFloat());
        add_marker(std::make_shared<visualization_msgs::msg::MarkerArray>(virtual_wall));
        continue;
      }
    }
  }
}
}  // namespace autoware::rviz_plugins

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(autoware::rviz_plugins::PlanningFactorRvizPlugin, rviz_common::Display)
