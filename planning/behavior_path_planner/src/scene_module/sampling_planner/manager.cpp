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

  p.max_curvature = node->declare_parameter<double>(name + ".max_curvature");
  p.min_curvature = node->declare_parameter<double>(name + ".min_curvature");
  p.lateral_deviation_weight = node->declare_parameter<double>(name + ".lateral_deviation_weight");
  p.length_weight = node->declare_parameter<double>(name + ".length_weight");
  p.curvature_weight = node->declare_parameter<double>(name + ".curvature_weight");
  p.enable_frenet = node->declare_parameter<bool>(name + ".enable_frenet");
  p.enable_bezier = node->declare_parameter<bool>(name + ".enable_bezier");
  p.resolution = node->declare_parameter<double>(name + ".resolution");
  p.previous_path_reuse_points_nb =
    node->declare_parameter<int>(name + ".previous_path_reuse_points_nb");
  p.nb_target_lateral_positions =
    node->declare_parameter<int>(name + ".nb_target_lateral_positions");
  p.target_lengths = node->declare_parameter<std::vector<double>>(name + ".target_lengths");
  p.target_lateral_positions =
    node->declare_parameter<std::vector<double>>(name + ".target_lateral_positions");
  p.target_lateral_velocities =
    node->declare_parameter<std::vector<double>>(name + ".target_lateral_velocities");
  p.target_lateral_accelerations =
    node->declare_parameter<std::vector<double>>(name + ".target_lateral_accelerations");
  p.force_zero_deviation = node->declare_parameter<bool>(name + ".force_zero_deviation");
  p.force_zero_heading = node->declare_parameter<bool>(name + ".force_zero_heading");
  p.smooth_reference = node->declare_parameter<bool>(name + ".smooth_reference");

  parameters_ = std::make_shared<SamplingPlannerParameters>(p);
}

}  // namespace behavior_path_planner
