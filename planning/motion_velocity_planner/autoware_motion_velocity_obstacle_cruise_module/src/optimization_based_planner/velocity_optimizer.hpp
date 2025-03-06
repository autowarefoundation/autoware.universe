// Copyright 2025 TIER IV, Inc.
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
#ifndef OPTIMIZATION_BASED_PLANNER__VELOCITY_OPTIMIZER_HPP_
#define OPTIMIZATION_BASED_PLANNER__VELOCITY_OPTIMIZER_HPP_

#include "autoware/osqp_interface/osqp_interface.hpp"
#include "s_boundary.hpp"

#include <vector>

namespace autoware::motion_velocity_planner
{
class VelocityOptimizer
{
public:
  struct OptimizationData
  {
    std::vector<double> time_vec;
    double s0;
    double v0;
    double a0;
    double v_max;
    double a_max;
    double a_min;
    double limit_a_max;
    double limit_a_min;
    double limit_j_max;
    double limit_j_min;
    double j_max;
    double j_min;
    double t_dangerous;
    double idling_time;
    SBoundaries s_boundary;
  };

  struct OptimizationResult
  {
    std::vector<double> t;
    std::vector<double> s;
    std::vector<double> v;
    std::vector<double> a;
    std::vector<double> j;
  };

  VelocityOptimizer(
    const double max_s_weight, const double max_v_weight, const double over_s_safety_weight,
    const double over_s_ideal_weight, const double over_v_weight, const double over_a_weight,
    const double over_j_weight);

  OptimizationResult optimize(const OptimizationData & data);

private:
  // Parameter
  double max_s_weight_;
  double max_v_weight_;
  double over_s_safety_weight_;
  double over_s_ideal_weight_;
  double over_v_weight_;
  double over_a_weight_;
  double over_j_weight_;

  // QPSolver
  autoware::osqp_interface::OSQPInterface qp_solver_;
};
}  // namespace autoware::motion_velocity_planner
#endif  // OPTIMIZATION_BASED_PLANNER__VELOCITY_OPTIMIZER_HPP_
