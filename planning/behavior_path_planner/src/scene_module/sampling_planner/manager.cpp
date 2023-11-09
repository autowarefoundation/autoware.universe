// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/scene_module/sampling_planner/manager.hpp"

#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

SamplingPlannerModuleManager::SamplingPlannerModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config)
: SceneModuleManagerInterface(node, name, config, {})
{
  SamplingPlannerParameters p{};
  {
    std::string ns{"constraints.hard"};
    p.max_curvature = node->declare_parameter<double>(ns + ".max_curvature");
    p.min_curvature = node->declare_parameter<double>(ns + ".min_curvature");
    ns = std::string{"constraints.soft"};
    p.lateral_deviation_weight = node->declare_parameter<double>(ns + ".lateral_deviation_weight");
    p.length_weight = node->declare_parameter<double>(ns + ".length_weight");
    p.curvature_weight = node->declare_parameter<double>(ns + ".curvature_weight");
  }
  {
    std::string ns{"sampling"};
    p.enable_frenet = node->declare_parameter<bool>(ns + ".enable_frenet");
    p.enable_bezier = node->declare_parameter<bool>(ns + ".enable_bezier");
    p.resolution = node->declare_parameter<double>(ns + ".resolution");
    p.previous_path_reuse_points_nb =
      node->declare_parameter<int>(ns + ".previous_path_reuse_points_nb");
    p.nb_target_lateral_positions =
      node->declare_parameter<int>(ns + ".nb_target_lateral_positions");
    p.target_lengths = node->declare_parameter<std::vector<double>>(ns + ".target_lengths");
    p.target_lateral_positions =
      node->declare_parameter<std::vector<double>>(ns + ".target_lateral_positions");
    ns += ".frenet";
    p.target_lateral_velocities =
      node->declare_parameter<std::vector<double>>(ns + ".target_lateral_velocities");
    p.target_lateral_accelerations =
      node->declare_parameter<std::vector<double>>(ns + ".target_lateral_accelerations");
  }
  {
    std::string ns{"preprocessing"};
    p.force_zero_deviation = node->declare_parameter<bool>(ns + ".force_zero_initial_deviation");
    p.force_zero_heading = node->declare_parameter<bool>(ns + ".force_zero_initial_heading");
    p.smooth_reference = node->declare_parameter<bool>(ns + ".smooth_reference_trajectory");
  }
  parameters_ = std::make_shared<SamplingPlannerParameters>(p);
}

void SamplingPlannerModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto & p = parameters_;

  [[maybe_unused]] std::string ns = name_ + ".";

  std::for_each(observers_.begin(), observers_.end(), [&](const auto & observer) {
    if (!observer.expired()) {
      const auto sampling_planner_ptr =
        std::dynamic_pointer_cast<SamplingPlannerModule>(observer.lock());
      if (sampling_planner_ptr) {
        sampling_planner_ptr->updateModuleParams(p);
      }
    }
  });
}

}  // namespace behavior_path_planner
