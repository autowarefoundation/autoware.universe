// Copyright 2015-2019 Autoware Foundation
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
#include <cmath>

#include <vehicle_cmd_gate/vehicle_cmd_filter.hpp>

VehicleCmdFilter::VehicleCmdFilter() {}

void VehicleCmdFilter::limitLongitudinalWithVel(autoware_control_msgs::msg::ControlCommand & input)
{
  input.velocity = std::max(std::min(input.velocity, vel_lim_), -vel_lim_);
}

void VehicleCmdFilter::limitLongitudinalWithAcc(
  const double dt, autoware_control_msgs::msg::ControlCommand & input)
{
  input.acceleration = std::max(std::min(input.acceleration, lon_acc_lim_), -lon_acc_lim_);
  input.velocity = limitDiff(input.velocity, prev_cmd_.velocity, lon_acc_lim_ * dt);
}

void VehicleCmdFilter::VehicleCmdFilter::limitLongitudinalWithJerk(
  const double dt, autoware_control_msgs::msg::ControlCommand & input)
{
  input.acceleration = limitDiff(input.acceleration, prev_cmd_.acceleration, lon_jerk_lim_ * dt);
}

void VehicleCmdFilter::limitLateralWithLatAcc(
  const double dt, autoware_control_msgs::msg::ControlCommand & input)
{
  double latacc = calcLatAcc(input);
  if (std::fabs(latacc) > lat_acc_lim_) {
    double v_sq = std::max(input.velocity * input.velocity, 0.001);
    double steer_lim = std::atan(lat_acc_lim_ * wheel_base_ / v_sq);
    input.steering_angle = latacc > 0.0 ? steer_lim : -steer_lim;
  }
}

void VehicleCmdFilter::limitLateralWithLatJerk(
  const double dt, autoware_control_msgs::msg::ControlCommand & input)
{
  double curr_latacc = calcLatAcc(input);
  double prev_latacc = calcLatAcc(prev_cmd_);
  double latacc_lim = limitDiff(curr_latacc, prev_latacc, lat_jerk_lim_ * dt);
  if (std::fabs(curr_latacc) > std::fabs(latacc_lim)) {
    double v_sq = std::max(input.velocity * input.velocity, 0.001);
    double steer_lim = std::atan(std::fabs(latacc_lim) * wheel_base_ / v_sq);
    input.steering_angle = curr_latacc > 0.0 ? steer_lim : -steer_lim;
  }
}

double VehicleCmdFilter::calcLatAcc(const autoware_control_msgs::msg::ControlCommand & cmd)
{
  double v = cmd.velocity;
  return v * v * std::tan(cmd.steering_angle) / wheel_base_;
}

double VehicleCmdFilter::limitDiff(const double curr, const double prev, const double diff_lim)
{
  double diff = std::max(std::min(curr - prev, diff_lim), -diff_lim);
  return prev + diff;
}