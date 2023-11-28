// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__SAMPLING_PLANNER__UTIL_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__SAMPLING_PLANNER__UTIL_HPP_
#include "sampler_common/structures.hpp"

#include <any>
#include <functional>
#include <vector>

namespace behavior_path_planner
{
using geometry_msgs::msg::Pose;

struct SoftConstraintsInputs
{
  Pose goal_pose;
};

using SoftConstraintsFunctionVector = std::vector<std::function<double(
  sampler_common::Path &, const sampler_common::Constraints &, const SoftConstraintsInputs &)>>;

using HardConstraintsFunctionVector = std::vector<std::function<bool(
  sampler_common::Path &, const sampler_common::Constraints &, const MultiPoint2d &)>>;

inline void evaluateSoftConstraints(
  sampler_common::Path & path, const sampler_common::Constraints & constraints,
  const SoftConstraintsFunctionVector & soft_constraints, const SoftConstraintsInputs & input_data,
  std::vector<double> & constraints_results)
{
  constraints_results.clear();
  constraints_results.resize(soft_constraints.size());
  double constraints_evaluation = 0.0;
  for (const auto & f : soft_constraints) {
    const auto value = f(path, constraints, input_data);
    constraints_results.push_back(value);
    constraints_evaluation += value;
  }
  path.cost = constraints_evaluation;
  return;
}

inline void evaluateHardConstraints(
  sampler_common::Path & path, const sampler_common::Constraints & constraints,
  const MultiPoint2d & footprint, const HardConstraintsFunctionVector & hard_constraints,
  std::vector<bool> & constraints_results)
{
  constraints_results.clear();
  constraints_results.resize(hard_constraints.size());
  bool constraints_passed = true;
  int idx = 0;
  for (const auto & f : hard_constraints) {
    const bool constraint_check = f(path, constraints, footprint);
    constraints_passed &= constraint_check;
    constraints_results[idx] = constraint_check;
    ++idx;
  }

  path.constraints_satisfied = constraints_passed;
  return;
}

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__SAMPLING_PLANNER__UTIL_HPP_
