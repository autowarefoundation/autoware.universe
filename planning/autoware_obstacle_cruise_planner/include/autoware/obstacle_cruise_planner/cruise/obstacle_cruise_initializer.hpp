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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__CRUISE__OBSTACLE_CRUISE_INITIALIZER_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__CRUISE__OBSTACLE_CRUISE_INITIALIZER_HPP_

#include "autoware/obstacle_cruise_planner/cruise/obstacle_cruise_module.hpp"
#include "autoware/obstacle_cruise_planner/cruise/optimization_based_planner/optimization_based_planner.hpp"
#include "autoware/obstacle_cruise_planner/cruise/pid_based_planner/pid_based_planner.hpp"

#include <memory>
#include <string>

namespace autoware::motion_planning
{
// planning algorithm
enum class PlanningAlgorithm { OPTIMIZATION_BASE, PID_BASE, INVALID };

PlanningAlgorithm getPlanningAlgorithmType(const std::string & param)
{
  if (param == "pid_base") {
    return PlanningAlgorithm::PID_BASE;
  } else if (param == "optimization_base") {
    return PlanningAlgorithm::OPTIMIZATION_BASE;
  }
  return PlanningAlgorithm::INVALID;
}

std::unique_ptr<ObstacleCruiseModule> getModule(
  rclcpp::Node & node, const LongitudinalInfo & longitudinal_info)
{
  const std::string planning_algorithm_param =
    node.declare_parameter<std::string>("cruise.planning_algorithm");
  const auto planning_algorithm = getPlanningAlgorithmType(planning_algorithm_param);

  if (planning_algorithm == PlanningAlgorithm::OPTIMIZATION_BASE) {
    return std::make_unique<OptimizationBasedPlanner>(node, longitudinal_info);
  } else if (planning_algorithm == PlanningAlgorithm::PID_BASE) {
    return std::make_unique<PIDBasedPlanner>(node, longitudinal_info);
  }
  throw std::logic_error("Designated algorithm is not supported.");
}

}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__CRUISE__OBSTACLE_CRUISE_INITIALIZER_HPP_
