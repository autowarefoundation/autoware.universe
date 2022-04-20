// Copyright 2020 Tier IV, Inc.
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

#include <scene_module/dynamic_obstacle_stop/manager.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace
{
}  // namespace

DynamicObstacleStopModuleManager::DynamicObstacleStopModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  // Vehicle Parameters
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
    auto & p = planner_param_.vehicle_param;
    p.base_to_front = vehicle_info.wheel_base_m + vehicle_info.front_overhang_m;
    p.base_to_rear = vehicle_info.rear_overhang_m;
    p.width = vehicle_info.vehicle_width_m;
  }

  const std::string ns(getModuleName());

  {
    auto & p = planner_param_.common;
    p.normal_min_jerk = node.declare_parameter(".normal.min_jerk", -0.3);
    p.normal_min_acc = node.declare_parameter(".normal.min_acc", -1.0);
    p.limit_min_jerk = node.declare_parameter(".limit.min_jerk", -1.5);
    p.limit_min_acc = node.declare_parameter(".limit.min_acc", -2.5);
  }

  {
    auto & p = planner_param_.dynamic_obstacle_stop;
    p.detection_method = node.declare_parameter(ns + ".detection_method", "Object");
    p.use_partition_lanelet = node.declare_parameter(ns + ".use_partition_lanelet", true);
    p.specify_decel_jerk = node.declare_parameter(ns + ".specify_decel_jerk", false);
    p.stop_margin = node.declare_parameter(ns + ".stop_margin", 2.5);
    p.passing_margin = node.declare_parameter(ns + ".passing_margin", 1.0);
    p.stop_start_jerk_dec = node.declare_parameter(ns + ".stop_start_jerk_dec", -0.3);
    p.obstacle_velocity_kph = node.declare_parameter(ns + ".obstacle_velocity_kph", 5.0);
    p.detection_distance = node.declare_parameter(ns + ".detection_distance", 45.0);
    p.detection_span = node.declare_parameter(ns + ".detection_span", 1.0);
    p.min_vel_ego_kmph = node.declare_parameter(ns + ".min_vel_ego_kmph", 5.0);
  }

  {
    auto & p = planner_param_.detection_area;
    const std::string ns_da = ns + ".detection_area_size";
    p.dist_ahead = node.declare_parameter(ns_da + ".dist_ahead", 50.0);
    p.dist_behind = node.declare_parameter(ns_da + ".dist_behind", 5.0);
    p.dist_right = node.declare_parameter(ns_da + ".dist_right", 10.0);
    p.dist_left = node.declare_parameter(ns_da + ".dist_left", 10.0);
  }

  {
    auto & p = planner_param_.dynamic_obstacle;
    const std::string ns_do = ns + ".dynamic_obstacle";
    p.min_vel_kmph = node.declare_parameter(ns_do + ".min_vel_kmph", 0.0);
    p.max_vel_kmph = node.declare_parameter(ns_do + ".max_vel_kmph", 5.0);
    p.diameter = node.declare_parameter(ns_do + ".diameter", 0.1);
    p.height = node.declare_parameter(ns_do + ".height", 2.0);
    p.path_size = node.declare_parameter(ns_do + ".path_size", 20);
    p.time_step = node.declare_parameter(ns_do + ".time_step", 0.5);
  }

  {
    auto & p = planner_param_.approaching;
    const std::string ns_a = ns + ".approaching";
    p.enable = node.declare_parameter(ns_a + ".enable", false);
    p.margin = node.declare_parameter(ns_a + ".margin", 0.0);
    p.limit_vel_kmph = node.declare_parameter(ns_a + ".limit_vel_kmph", 5.0);
    p.stop_thresh = node.declare_parameter(ns_a + ".stop_thresh", 0.01);
    p.stop_time_thresh = node.declare_parameter(ns_a + ".stop_time_thresh", 3.0);
    p.dist_thresh = node.declare_parameter(ns_a + ".dist_thresh", 0.5);
  }

  {
    auto & p = planner_param_.slow_down_limit;
    const std::string ns_m = ns + ".slow_down_limit";
    p.enable = node.declare_parameter(ns_m + ".enable", true);
    p.max_jerk = node.declare_parameter(ns_m + ".max_jerk", -0.7);
    p.max_acc = node.declare_parameter(ns_m + ".max_acc", -2.0);
  }

  debug_ptr_ = std::make_shared<DynamicObstacleStopDebug>(node);

  // Set parameter callback
  set_param_res_ = node.add_on_set_parameters_callback(
    std::bind(&DynamicObstacleStopModuleManager::paramCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult DynamicObstacleStopModuleManager::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    std::string ns = "dynamic_obstacle_stop.";
    tier4_autoware_utils::updateParam(
      parameters, ns + "detection_method", planner_param_.dynamic_obstacle_stop.detection_method);
    tier4_autoware_utils::updateParam(
      parameters, ns + "stop_margin", planner_param_.dynamic_obstacle_stop.stop_margin);
    tier4_autoware_utils::updateParam(
      parameters, ns + "passing_margin", planner_param_.dynamic_obstacle_stop.passing_margin);
    tier4_autoware_utils::updateParam(
      parameters, ns + "stop_start_jerk_dec",
      planner_param_.dynamic_obstacle_stop.stop_start_jerk_dec);
    tier4_autoware_utils::updateParam(
      parameters, ns + "obstacle_velocity_kph",
      planner_param_.dynamic_obstacle_stop.obstacle_velocity_kph);
    tier4_autoware_utils::updateParam(
      parameters, ns + "detection_distance",
      planner_param_.dynamic_obstacle_stop.detection_distance);
    tier4_autoware_utils::updateParam(
      parameters, ns + "detection_span", planner_param_.dynamic_obstacle_stop.detection_span);
    tier4_autoware_utils::updateParam(
      parameters, ns + "min_vel_ego_kmph", planner_param_.dynamic_obstacle_stop.min_vel_ego_kmph);

    ns = "dynamic_obstacle_stop.detection_area_size.";
    tier4_autoware_utils::updateParam(
      parameters, ns + "dist_ahead", planner_param_.detection_area.dist_ahead);
    tier4_autoware_utils::updateParam(
      parameters, ns + "dist_behind", planner_param_.detection_area.dist_behind);
    tier4_autoware_utils::updateParam(
      parameters, ns + "dist_right", planner_param_.detection_area.dist_right);
    tier4_autoware_utils::updateParam(
      parameters, ns + "dist_left", planner_param_.detection_area.dist_left);

    ns = "dynamic_obstacle_stop.dynamic_obstacle.";
    tier4_autoware_utils::updateParam(
      parameters, ns + "min_vel_kmph", planner_param_.dynamic_obstacle.min_vel_kmph);
    tier4_autoware_utils::updateParam(
      parameters, ns + "max_vel_kmph", planner_param_.dynamic_obstacle.max_vel_kmph);
    tier4_autoware_utils::updateParam(
      parameters, ns + "diameter", planner_param_.dynamic_obstacle.diameter);
    tier4_autoware_utils::updateParam(
      parameters, ns + "height", planner_param_.dynamic_obstacle.height);
    tier4_autoware_utils::updateParam(
      parameters, ns + "path_size", planner_param_.dynamic_obstacle.path_size);
    tier4_autoware_utils::updateParam(
      parameters, ns + "time_step", planner_param_.dynamic_obstacle.time_step);

    ns = "dynamic_obstacle_stop.approaching.";
    tier4_autoware_utils::updateParam(parameters, ns + "enable", planner_param_.approaching.enable);
    tier4_autoware_utils::updateParam(parameters, ns + "margin", planner_param_.approaching.margin);
    tier4_autoware_utils::updateParam(
      parameters, ns + "limit_vel_kmph", planner_param_.approaching.limit_vel_kmph);
    tier4_autoware_utils::updateParam(
      parameters, ns + "stop_thresh", planner_param_.approaching.stop_thresh);
    tier4_autoware_utils::updateParam(
      parameters, ns + "stop_time_thresh", planner_param_.approaching.stop_time_thresh);
    tier4_autoware_utils::updateParam(
      parameters, ns + "dist_thresh", planner_param_.approaching.dist_thresh);

    ns = "dynamic_obstacle_stop.slow_down_limit.";
    tier4_autoware_utils::updateParam(
      parameters, ns + "enable", planner_param_.slow_down_limit.enable);
    tier4_autoware_utils::updateParam(
      parameters, ns + "max_jerk", planner_param_.slow_down_limit.max_jerk);
    tier4_autoware_utils::updateParam(
      parameters, ns + "max_acc", planner_param_.slow_down_limit.max_acc);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  // TODO: this is not safe ?
  for (const auto & scene_module : scene_modules_) {
    std::dynamic_pointer_cast<DynamicObstacleStopModule>(scene_module)
      ->setPlannerParam(planner_param_);
  }

  return result;
}
void DynamicObstacleStopModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.empty()) {
    return;
  }

  // TODO(Tomohito Ando): appropreate id
  constexpr int64_t module_id = 0;
  if (!isModuleRegistered(module_id)) {
    registerModule(std::make_shared<DynamicObstacleStopModule>(
      module_id, planner_data_, planner_param_, logger_.get_child("dynamic_obstacle_stop_module"),
      debug_ptr_, clock_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
DynamicObstacleStopModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  return
    [&path]([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) -> bool {
      return false;
    };
}
}  // namespace behavior_velocity_planner
