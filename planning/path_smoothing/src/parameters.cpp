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

#include "path_smoothing/parameters.hpp"

#include <tier4_autoware_utils/ros/update_param.hpp>

namespace path_smoothing
{
EBParam::EBParam(rclcpp::Node * node)
{
  {  // option
    enable_warm_start = node->declare_parameter<bool>("elastic_band.option.enable_warm_start");
    enable_optimization_validation =
      node->declare_parameter<bool>("elastic_band.option.enable_optimization_validation");
  }

  {  // common
    delta_arc_length = node->declare_parameter<double>("elastic_band.common.delta_arc_length");
    num_points = node->declare_parameter<int>("elastic_band.common.num_points");
  }

  {  // clearance
    num_joint_points = node->declare_parameter<int>("elastic_band.clearance.num_joint_points");
    clearance_for_fix = node->declare_parameter<double>("elastic_band.clearance.clearance_for_fix");
    clearance_for_joint =
      node->declare_parameter<double>("elastic_band.clearance.clearance_for_joint");
    clearance_for_smooth =
      node->declare_parameter<double>("elastic_band.clearance.clearance_for_smooth");
  }

  {  // weight
    smooth_weight = node->declare_parameter<double>("elastic_band.weight.smooth_weight");
    lat_error_weight = node->declare_parameter<double>("elastic_band.weight.lat_error_weight");
  }

  {  // qp
    qp_param.max_iteration = node->declare_parameter<int>("elastic_band.qp.max_iteration");
    qp_param.eps_abs = node->declare_parameter<double>("elastic_band.qp.eps_abs");
    qp_param.eps_rel = node->declare_parameter<double>("elastic_band.qp.eps_rel");
  }

  // validation
  max_validation_error = node->declare_parameter<double>("elastic_band.validation.max_error");
}

void EBParam::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // option
    updateParam<bool>(parameters, "elastic_band.option.enable_warm_start", enable_warm_start);
    updateParam<bool>(
      parameters, "elastic_band.option.enable_optimization_validation",
      enable_optimization_validation);
  }

  {  // common
    updateParam<double>(parameters, "elastic_band.common.delta_arc_length", delta_arc_length);
    updateParam<int>(parameters, "elastic_band.common.num_points", num_points);
  }

  {  // clearance
    updateParam<int>(parameters, "elastic_band.clearance.num_joint_points", num_joint_points);
    updateParam<double>(parameters, "elastic_band.clearance.clearance_for_fix", clearance_for_fix);
    updateParam<double>(
      parameters, "elastic_band.clearance.clearance_for_joint", clearance_for_joint);
    updateParam<double>(
      parameters, "elastic_band.clearance.clearance_for_smooth", clearance_for_smooth);
  }

  {  // weight
    updateParam<double>(parameters, "elastic_band.weight.smooth_weight", smooth_weight);
    updateParam<double>(parameters, "elastic_band.weight.lat_error_weight", lat_error_weight);
  }

  {  // qp
    updateParam<int>(parameters, "elastic_band.qp.max_iteration", qp_param.max_iteration);
    updateParam<double>(parameters, "elastic_band.qp.eps_abs", qp_param.eps_abs);
    updateParam<double>(parameters, "elastic_band.qp.eps_rel", qp_param.eps_rel);
  }
}
}  // namespace path_smoothing
