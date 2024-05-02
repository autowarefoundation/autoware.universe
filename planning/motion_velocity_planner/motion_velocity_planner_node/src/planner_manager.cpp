// Copyright 2024 Tier IV, Inc.
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

#include "planner_manager.hpp"

#include <boost/format.hpp>

#include <memory>
#include <string>

namespace motion_velocity_planner
{
namespace
{
std::string json_dumps_pose(const geometry_msgs::msg::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}

diagnostic_msgs::msg::DiagnosticStatus make_stop_reason_diag(
  const std::string stop_reason, const geometry_msgs::msg::Pose & stop_pose)
{
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag;
  diagnostic_msgs::msg::KeyValue stop_reason_diag_kv;
  stop_reason_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stop_reason_diag.name = "stop_reason";
  stop_reason_diag.message = stop_reason;
  stop_reason_diag_kv.key = "stop_pose";
  stop_reason_diag_kv.value = json_dumps_pose(stop_pose);
  stop_reason_diag.values.push_back(stop_reason_diag_kv);
  return stop_reason_diag;
}
}  // namespace

MotionVelocityPlannerManager::MotionVelocityPlannerManager()
: plugin_loader_("motion_velocity_planner_node", "motion_velocity_planner::PluginModuleInterface")
{
}

void MotionVelocityPlannerManager::load_module_plugin(rclcpp::Node & node, const std::string & name)
{
  // Check if the plugin is already loaded.
  if (plugin_loader_.isClassLoaded(name)) {
    RCLCPP_WARN_STREAM(node.get_logger(), "The plugin '" << name << "' is already loaded.");
    return;
  }
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->init(node);

    // register
    loaded_plugins_.push_back(plugin);
    RCLCPP_DEBUG_STREAM(node.get_logger(), "The scene plugin '" << name << "' is loaded.");
  } else {
    RCLCPP_ERROR_STREAM(node.get_logger(), "The scene plugin '" << name << "' is not available.");
  }
}

void MotionVelocityPlannerManager::unload_module_plugin(
  rclcpp::Node & node, const std::string & name)
{
  auto it = std::remove_if(loaded_plugins_.begin(), loaded_plugins_.end(), [&](const auto plugin) {
    return plugin->get_module_name() == name;
  });

  if (it == loaded_plugins_.end()) {
    RCLCPP_WARN_STREAM(
      node.get_logger(),
      "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    loaded_plugins_.erase(it, loaded_plugins_.end());
    RCLCPP_INFO_STREAM(node.get_logger(), "The scene plugin '" << name << "' is unloaded.");
  }
}

void MotionVelocityPlannerManager::update_module_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (auto & plugin : loaded_plugins_) plugin->update_parameters(parameters);
}

std::vector<VelocityPlanningResult> MotionVelocityPlannerManager::plan_velocities(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
  const PlannerData & planner_data)
{
  std::string stop_reason_msg("path_end");

  std::vector<VelocityPlanningResult> results;
  for (auto & plugin : loaded_plugins_)
    results.push_back(plugin->plan(ego_trajectory_points, planner_data));

  // stop_reason_diag_ = makeStopReasonDiag(
  //   stop_reason_msg, output_trajectory_msg.points[first_stop_trajectory_point_index].point.pose);
  return results;
}
}  // namespace motion_velocity_planner
