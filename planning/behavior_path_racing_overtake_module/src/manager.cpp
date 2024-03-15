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

#include "behavior_path_racing_overtake_module/manager.hpp"

#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

// #define RESET "\033[0m"
// #define BLACK "\033[30m"              /* Black */
// #define RED "\033[31m"                /* Red */
// #define GREEN "\033[32m"              /* Green */
// #define YELLOW "\033[33m"             /* Yellow */
// #define BLUE "\033[34m"               /* Blue */
// #define MAGENTA "\033[35m"            /* Magenta */
// #define CYAN "\033[36m"               /* Cyan */
// #define WHITE "\033[37m"              /* White */
// #define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
// #define BOLDRED "\033[1m\033[31m"     /* Bold Red */
// #define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
// #define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
// #define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
// #define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
// #define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
// #define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

namespace behavior_path_planner
{

void RacingOvertakeModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {});
  RacingOvertakeParameters p{};

  // const std::string ns = "racing_overtake.";
  // p.min_distance_to_start_shifting = node->declare_parameter<double>(ns +
  // "min_distance_to_start_shifting"); p.time_to_start_shifting =
  // node->declare_parameter<double>(ns + "time_to_start_shifting"); p.shifting_lateral_jerk =
  // node->declare_parameter<double>(ns + "shifting_lateral_jerk"); p.min_shifting_distance =
  // node->declare_parameter<double>(ns + "min_shifting_distance"); p.min_shifting_speed =
  // node->declare_parameter<double>(ns + "min_shifting_speed"); p.shift_request_time_limit =
  // node->declare_parameter<double>(ns + "shift_request_time_limit"); p.publish_debug_marker =
  // node->declare_parameter<bool>(ns + "publish_debug_marker");

  parameters_ = std::make_shared<RacingOvertakeParameters>(p);
}

void RacingOvertakeModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  // using tier4_autoware_utils::updateParam;

  // [[maybe_unused]] auto p = parameters_;

  // [[maybe_unused]] const std::string ns = "racing_overtake.";
  // // updateParam<bool>(parameters, ns + ..., ...);

  // std::for_each(observers_.begin(), observers_.end(), [&p](const auto& observer) {
  //   if (!observer.expired())
  //     observer.lock()->updateModuleParams(p);
  // });
}

}  // namespace behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_path_planner::RacingOvertakeModuleManager,
  behavior_path_planner::SceneModuleManagerInterface)
