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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__START_GOAL_PLANNER_COMMON__UTILS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__START_GOAL_PLANNER_COMMON__UTILS_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "behavior_path_planner/utils/start_planner/start_planner_parameters.hpp"

#include <motion_utils/distance/distance.hpp>

#include <memory>

namespace behavior_path_planner::utils::start_goal_planner_common
{

using behavior_path_planner::StartPlannerParameters;
using behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams;
using behavior_path_planner::utils::path_safety_checker::ObjectsFilteringParams;
using behavior_path_planner::utils::path_safety_checker::SafetyCheckParams;

boost::optional<double> calcFeasibleDecelDistance(
  std::shared_ptr<const PlannerData> planner_data, const double acc_lim, const double jerk_lim,
  const double target_velocity);

void updateEgoPredictedPathParams(
  std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::shared_ptr<StartPlannerParameters> & start_planner_params);

void updateSafetyCheckParams(
  std::shared_ptr<SafetyCheckParams> & safety_check_params,
  const std::shared_ptr<StartPlannerParameters> & start_planner_params);

void updateObjectsFilteringParams(
  std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
  const std::shared_ptr<StartPlannerParameters> & start_planner_params);

}  // namespace behavior_path_planner::utils::start_goal_planner_common

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__START_GOAL_PLANNER_COMMON__UTILS_HPP_
