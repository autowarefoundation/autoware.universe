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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__PLANNER_DATA_YAML_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__PLANNER_DATA_YAML_HPP_

#include <autoware/behavior_path_planner_common/data_manager.hpp>

#include <yaml-cpp/yaml.h>

#include <string>
#include <type_traits>
#include <utility>

namespace autoware::behavior_path_planner
{

namespace impl
{

template <typename T>
std::string_view get_typename_impl()
{
  std::string_view name = __PRETTY_FUNCTION__;
  // see https://wandbox.org/permlink/lmBFiPVGyzpEPvOD
#if defined(__clang__)
  // std::string_view get_typename_impl() [T = std::string]
  name.remove_prefix(81);
  name.remove_suffix(1);
#elif defined(__GNUC__)
  // clang-format off
  // std::string_view get_typename_impl() [with T = std::__cxx11::basic_string<char>; std::string_view = std::basic_string_view<char>] // NOLINT
  // clang-format on
  name.remove_prefix(86);
  name.remove_suffix(50);
#endif
  return name;
}
}  // namespace impl

template <typename T>
std::string get_typename()
{
  return std::string(impl::get_typename_impl<std::decay_t<T>>());
}

enum YamlDataFormatVersion : int {
  v1 = 1,
};
static constexpr auto k_current_planner_data_yaml_format = YamlDataFormatVersion::v1;

std::pair<YAML::Node, std::string> get_planner_data_yaml(
  const nav_msgs::msg::Odometry & self_odometry,
  const geometry_msgs::msg::AccelWithCovarianceStamped & self_acceleration,
  const autoware_perception_msgs::msg::PredictedObjects & dynamic_object,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, const nav_msgs::msg::OccupancyGrid & costmap,
  tier4_planning_msgs::msg::LateralOffset::ConstSharedPtr lateral_offset,
  const autoware_adapi_v1_msgs::msg::OperationModeState & operation_mode,
  const autoware_planning_msgs::msg::LaneletRoute & route_ptr,
  autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_signal);

bool update_planner_data(PlannerData & planner_data, const YAML::Node & planner_data_yaml);

}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__PLANNER_DATA_YAML_HPP_
