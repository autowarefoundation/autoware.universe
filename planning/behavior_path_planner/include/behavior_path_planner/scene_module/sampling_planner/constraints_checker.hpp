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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SAMPLING_PLANNER__CONSTRAINTS_CHECKER_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SAMPLING_PLANNER__CONSTRAINTS_CHECKER_HPP_

#include "sampler_common/structures.hpp"

#include <any>
#include <functional>
#include <vector>

using SoftConstraintsFunctionVector = std::vector<std::function<double>(
  const sampler_common::Path &, const sampler_common::Constraints &)>;

using HardConstraintsFunctionVector = std::vector<std::function<bool>(
  const sampler_common::Path &, const sampler_common::Constraints &)>;

double checkSoftConstraints(
  const sampler_common::Path & path, const sampler_common::Constraints & constraints,
  const SoftConstraintsFunctionVector & soft_constraints, std::vector<double> & constraints_results)
{
  std::vector<double> constraints_evaluations(soft_constraints.size(), false);
  double constraints_evaluation = 0.0;
  for (const auto & f : soft_constraints) {
    constraints_evaluation += f(path, constraints);
  }
  return constraints_evaluation;
}

bool checkHardConstraints(
  const sampler_common::Path & path, const sampler_common::Constraints & constraints,
  const HardConstraintsFunctionVector & hard_constraints, std::vector<double> & constraints_results)
{
  std::vector<double> constraints_evaluations(hard_constraints.size(), false);
  bool constraints_evaluation = true;
  for (const auto & f : hard_constraints) {
    constraints_evaluation &= f(path, constraints);
  }
  return constraints_evaluation;
}

// template <typename T>
// class ConstraintsChecker
// {
// private:
//   /* data */
// public:
//   ConstraintsChecker(ConstraintsFunctionVector constraints_functions)
//   : constraints_functions_(constraints_functions){};
//   ~ConstraintsChecker(){};
//   ConstraintsFunctionVector<T> constraints_functions_;

//   T checkConstraints(
//     const sampler_common::Path & path, const sampler_common::Constraints & constraints,
//     std::vector<double> & constraints_results) = 0;

//   template <>
//   double checkConstraints(
//     const sampler_common::Path & path, const sampler_common::Constraints & constraints,
//     std::vector<double> & constraints_results)
//   {
//     std::vector<double> constraints_evaluations(constraints_functions_.size(), false);
//     double constraints_evaluation = 0.0;
//     for (const auto & f : constraints_functions_) {
//       constraints_evaluation += f(path, constraints);
//     }
//     return constraints_evaluation;
//   }

//   template <>
//   bool checkConstraints(
//     const sampler_common::Path & path, const sampler_common::Constraints & constraints,
//     std::vector<bool> & constraints_results)
//   {
//     std::vector<bool> constraints_evaluations(constraints_functions_.size(), false);
//     bool all_constraints_passed = true;
//     for (const auto & f : constraints_functions_) {
//       all_constraints_passed &= f(path, constraints);
//     }
//     return all_constraints_passed;
//   }
// };

// class HardConstraintsChecker : public ConstraintsChecker
// {
// private:
//   /* data */
// public:
//   HardConstraintsChecker(/* args */){};
//   ~HardConstraintsChecker(){};
// };

// class SoftConstraintsChecker : public ConstraintsChecker
// {
// private:
//   /* data */
// public:
//   SoftConstraintsChecker(/* args */){};
//   ~SoftConstraintsChecker(){};
// };

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SAMPLING_PLANNER__CONSTRAINTS_CHECKER_HPP_
