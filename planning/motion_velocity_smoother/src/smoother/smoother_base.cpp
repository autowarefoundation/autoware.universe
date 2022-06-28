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

#include "motion_velocity_smoother/smoother/smoother_base.hpp"

#include "motion_velocity_smoother/resample.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace motion_velocity_smoother
{
SmootherBase::SmootherBase(rclcpp::Node & node)
{
  auto & p = base_param_;
  p.max_accel = node.declare_parameter("normal.max_acc", 2.0);
  p.min_decel = node.declare_parameter("normal.min_acc", -3.0);
  p.stop_decel = node.declare_parameter("stop_decel", 0.0);
  p.max_jerk = node.declare_parameter("normal.max_jerk", 0.3);
  p.min_jerk = node.declare_parameter("normal.min_jerk", -0.1);
  p.max_lateral_accel = node.declare_parameter("max_lateral_accel", 0.2);
  p.decel_distance_before_curve = node.declare_parameter("decel_distance_before_curve", 3.5);
  p.decel_distance_after_curve = node.declare_parameter("decel_distance_after_curve", 0.0);
  p.min_curve_velocity = node.declare_parameter("min_curve_velocity", 1.38);
  p.resample_param.max_trajectory_length = node.declare_parameter("max_trajectory_length", 200.0);
  p.resample_param.min_trajectory_length = node.declare_parameter("min_trajectory_length", 30.0);
  p.resample_param.resample_time = node.declare_parameter("resample_time", 10.0);
  p.resample_param.dense_resample_dt = node.declare_parameter("dense_resample_dt", 0.1);
  p.resample_param.dense_min_interval_distance =
    node.declare_parameter("dense_min_interval_distance", 0.1);
  p.resample_param.sparse_resample_dt = node.declare_parameter("sparse_resample_dt", 0.5);
  p.resample_param.sparse_min_interval_distance =
    node.declare_parameter("sparse_min_interval_distance", 4.0);
}

void SmootherBase::setParam(const BaseParam & param) { base_param_ = param; }

SmootherBase::BaseParam SmootherBase::getBaseParam() const { return base_param_; }

double SmootherBase::getMaxAccel() const { return base_param_.max_accel; }

double SmootherBase::getMinDecel() const { return base_param_.min_decel; }

double SmootherBase::getMaxJerk() const { return base_param_.max_jerk; }

double SmootherBase::getMinJerk() const { return base_param_.min_jerk; }

boost::optional<TrajectoryPoints> SmootherBase::applyLateralAccelerationFilter(
  const TrajectoryPoints & input) const
{
  if (input.empty()) {
    return boost::none;
  }

  if (input.size() < 3) {
    return boost::optional<TrajectoryPoints>(input);  // cannot calculate lateral acc. do nothing.
  }

  // Interpolate with constant interval distance for lateral acceleration calculation.
  constexpr double points_interval = 0.1;  // [m]
  std::vector<double> out_arclength;
  const std::vector<double> in_arclength = trajectory_utils::calcArclengthArray(input);
  for (double s = 0; s < in_arclength.back(); s += points_interval) {
    out_arclength.push_back(s);
  }
  auto output = trajectory_utils::applyLinearInterpolation(in_arclength, input, out_arclength);
  if (!output) {
    RCLCPP_WARN(
      rclcpp::get_logger("smoother").get_child("smoother_base"),
      "interpolation failed at lateral acceleration filter.");
    return boost::none;
  }
  output->back() = input.back();  // keep the final speed.

  /*  Fill the steering tire angle w.r.t. orientation
   *
   *  delta_orientation / delta_time = velocity * tan(steering_tire_angle) / wheelbase
   *
   *  calculate desired steering_tire_angle w.r.t. delta_orientation, velocity, and wheelbase
   *
   */

  for (size_t i = 0; i < output->size() - 1; i++) {
      output->at(i).front_wheel_angle_rad = static_cast<float>(std::atan(
        (tf2::getYaw(output->at(i + 1).pose.orientation) -
         tf2::getYaw(output->at(i).pose.orientation)) *
        base_param_.wheel_base / (points_interval)));
  }
  output->back().front_wheel_angle_rad = 0.0;

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFromFrontSteeringAngle(
    *output, base_param_.wheel_base);
  if (!curvature_v) {
    return boost::optional<TrajectoryPoints>(input);
  }

  //  Decrease speed according to lateral G
  const size_t before_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_before_curve / points_interval));
  const size_t after_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_after_curve / points_interval));
  const double max_lateral_accel_abs = std::fabs(base_param_.max_lateral_accel);

  for (size_t i = 0; i < output->size(); ++i) {
    double curvature = 0.0;
    const size_t start = i > after_decel_index ? i - after_decel_index : 0;
    const size_t end = std::min(output->size(), i + before_decel_index + 1);
    for (size_t j = start; j < end; ++j) {
      curvature = std::max(curvature, std::fabs(curvature_v->at(j)));
    }
    double v_curvature_max = std::sqrt(max_lateral_accel_abs / std::max(curvature, 1.0E-5));
    v_curvature_max = std::max(v_curvature_max, base_param_.min_curve_velocity);
    if (output->at(i).longitudinal_velocity_mps > v_curvature_max) {
      output->at(i).longitudinal_velocity_mps = v_curvature_max;
    }
  }
  return output;
}

}  // namespace motion_velocity_smoother
