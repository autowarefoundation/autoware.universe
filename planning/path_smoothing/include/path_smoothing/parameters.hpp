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

#ifndef PATH_SMOOTHING__PARAMETERS_HPP_
#define PATH_SMOOTHING__PARAMETERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace path_smoothing
{
struct EBParam
{
  // qp
  struct QPParam
  {
    int max_iteration;
    double eps_abs;
    double eps_rel;
  };

  EBParam() = default;
  explicit EBParam(rclcpp::Node * node);
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

  // option
  bool enable_warm_start;
  bool enable_optimization_validation;

  // common
  double delta_arc_length;
  int num_points;

  // clearance
  int num_joint_points;
  double clearance_for_fix;
  double clearance_for_joint;
  double clearance_for_smooth;

  // weight
  double smooth_weight;
  double lat_error_weight;

  // qp
  QPParam qp_param;

  // validation
  double max_validation_error;
};
}  // namespace path_smoothing

#endif  // PATH_SMOOTHING__PARAMETERS_HPP_
