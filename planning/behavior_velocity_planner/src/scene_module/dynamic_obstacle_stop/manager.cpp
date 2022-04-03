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

#include <lanelet2_extension/utility/query.hpp>
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
    p.enable_dynamic_obstacle_stop =
      node.declare_parameter(ns + ".enable_dynamic_obstacle_stop", true);
    p.use_objects = node.declare_parameter(ns + ".use_objects", true);
    p.use_predicted_path = node.declare_parameter(ns + ".use_predicted_path", false);
    p.use_partition_lanelet = node.declare_parameter(ns + ".use_partition_lanelet", true);
    p.extend_distance = node.declare_parameter(ns + ".extend_distance", 5.0);
    p.stop_margin = node.declare_parameter(ns + ".stop_margin", 2.5);
    p.passing_margin = node.declare_parameter(ns + ".passing_margin", 1.0);
    p.stop_start_jerk_dec = node.declare_parameter(ns + ".stop_start_jerk_dec", -0.3);
    p.obstacle_velocity_kph = node.declare_parameter(ns + ".obstacle_velocity_kph", 5.0);
    p.detection_distance = node.declare_parameter(ns + ".detection_distance", 45.0);
    p.detection_span = node.declare_parameter(ns + ".detection_span", 1.0);
    p.min_vel_ego_kmph = node.declare_parameter(ns + ".min_vel_ego_kmph", 5.0);
    p.velocity_limit_kmph = node.declare_parameter(ns + ".velocity_limit_kmph", 20.0);
    p.calc_collision_from_point = node.declare_parameter(ns + ".calc_collision_from_point", true);
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

  smoother_ = std::make_shared<motion_velocity_smoother::AnalyticalJerkConstrainedSmoother>(node);
  debug_ptr_ = std::make_shared<DynamicObstacleStopDebug>(node);

  // // Set parameter callback
  // set_param_res_ = this->add_on_set_parameters_callback(
  //   std::bind(&DynamicObstacleStopPlannerNode::paramCallback, this, std::placeholders::_1));
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
      smoother_, debug_ptr_, clock_));
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
