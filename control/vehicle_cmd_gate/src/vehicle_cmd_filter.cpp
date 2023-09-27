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

#include "vehicle_cmd_filter.hpp"

#include <algorithm>
#include <cmath>

namespace vehicle_cmd_gate
{

VehicleCmdFilter::VehicleCmdFilter()
{
}

bool VehicleCmdFilter::setParameterWithValidation(const VehicleCmdFilterParam & p)
{
  const auto s = p.reference_speed_points.size();
  if (
    p.lon_acc_lim.size() != s || p.lon_jerk_lim.size() != s || p.lat_acc_lim.size() != s ||
    p.lat_jerk_lim.size() != s || p.actual_steer_diff_lim.size() != s || p.steer_lim.size() != s ||
    p.steer_rate_lim.size() != s) {
    std::cerr << "VehicleCmdFilter::setParam() There is a size mismatch in the parameter. "
                 "Parameter initialization failed."
              << std::endl;
    return false;
  }

  param_ = p;
  return true;
}
void VehicleCmdFilter::setSteerLim(LimitArray v)
{
  auto tmp = param_;
  tmp.steer_lim = v;
  setParameterWithValidation(tmp);
}
void VehicleCmdFilter::setSteerRateLim(LimitArray v)
{
  auto tmp = param_;
  tmp.steer_rate_lim = v;
  setParameterWithValidation(tmp);
}
void VehicleCmdFilter::setLonAccLim(LimitArray v)
{
  auto tmp = param_;
  tmp.lon_acc_lim = v;
  setParameterWithValidation(tmp);
}
void VehicleCmdFilter::setLonJerkLim(LimitArray v)
{
  auto tmp = param_;
  tmp.lon_jerk_lim = v;
  setParameterWithValidation(tmp);
}
void VehicleCmdFilter::setLatAccLim(LimitArray v)
{
  auto tmp = param_;
  tmp.lat_acc_lim = v;
  setParameterWithValidation(tmp);
}
void VehicleCmdFilter::setLatJerkLim(LimitArray v)
{
  auto tmp = param_;
  tmp.lat_jerk_lim = v;
  setParameterWithValidation(tmp);
}
void VehicleCmdFilter::setActualSteerDiffLim(LimitArray v)
{
  auto tmp = param_;
  tmp.actual_steer_diff_lim = v;
  setParameterWithValidation(tmp);
}

void VehicleCmdFilter::setParam(const VehicleCmdFilterParam & p)
{
  if (!setParameterWithValidation(p)) {
    std::exit(EXIT_FAILURE);
  }
}

void VehicleCmdFilter::limitLongitudinalWithVel(
  AckermannControlCommand & input, GateFilterInfo & info) const
{
  info.limit.velocity = param_.vel_lim;
  info.raw.velocity = input.longitudinal.speed;

  input.longitudinal.speed = std::max(
    std::min(static_cast<double>(input.longitudinal.speed), param_.vel_lim), -param_.vel_lim);

  info.filtered.velocity = input.longitudinal.speed;
}

void VehicleCmdFilter::limitLongitudinalWithAcc(
  const double dt, AckermannControlCommand & input, GateFilterInfo & info) const
{
  const auto lon_acc_lim = getLonAccLim();
  info.limit.lon_acceleration = lon_acc_lim;

  info.raw.lon_acceleration = input.longitudinal.acceleration;

  input.longitudinal.acceleration = std::max(
    std::min(static_cast<double>(input.longitudinal.acceleration), lon_acc_lim), -lon_acc_lim);
  input.longitudinal.speed =
    limitDiff(input.longitudinal.speed, prev_cmd_.longitudinal.speed, lon_acc_lim * dt);

  info.filtered.lon_acceleration = input.longitudinal.acceleration;
}

void VehicleCmdFilter::VehicleCmdFilter::limitLongitudinalWithJerk(
  const double dt, AckermannControlCommand & input, GateFilterInfo & info) const
{
  const auto lon_jerk_lim = getLonJerkLim();
  info.limit.lon_jerk = lon_jerk_lim;

  info.raw.lon_jerk =
    (input.longitudinal.acceleration - prev_cmd_.longitudinal.acceleration) / std::max(dt, 1.0e-10);

  input.longitudinal.acceleration = limitDiff(
    input.longitudinal.acceleration, prev_cmd_.longitudinal.acceleration, lon_jerk_lim * dt);
  input.longitudinal.jerk =
    std::clamp(static_cast<double>(input.longitudinal.jerk), -lon_jerk_lim, lon_jerk_lim);

  info.filtered.lon_jerk =
    (input.longitudinal.acceleration - prev_cmd_.longitudinal.acceleration) / std::max(dt, 1.0e-10);
}

// Use ego vehicle speed (not speed command) for the lateral acceleration calculation, otherwise the
// filtered steering angle oscillates if the input velocity oscillates.
void VehicleCmdFilter::limitLateralWithLatAcc(
  [[maybe_unused]] const double dt, AckermannControlCommand & input, GateFilterInfo & info) const
{
  const auto lat_acc_lim = getLatAccLim();
  info.limit.lat_acceleration = lat_acc_lim;

  double latacc = calcLatAcc(input, current_speed_);
  info.raw.lat_acceleration = latacc;
  if (std::fabs(latacc) > lat_acc_lim) {
    double v_sq = std::max(static_cast<double>(current_speed_ * current_speed_), 0.001);
    double steer_lim = std::atan(lat_acc_lim * param_.wheel_base / v_sq);
    input.lateral.steering_tire_angle = latacc > 0.0 ? steer_lim : -steer_lim;
  }
  info.filtered.lat_acceleration = calcLatAcc(input, current_speed_);
}

// Use ego vehicle speed (not speed command) for the lateral acceleration calculation, otherwise the
// filtered steering angle oscillates if the input velocity oscillates.
void VehicleCmdFilter::limitLateralWithLatJerk(
  const double dt, AckermannControlCommand & input, GateFilterInfo & info) const
{
  double curr_latacc = calcLatAcc(input, current_speed_);
  double prev_latacc = calcLatAcc(prev_cmd_, current_speed_);

  const auto lat_jerk_lim = getLatJerkLim();
  info.limit.lat_jerk = lat_jerk_lim;
  info.raw.lat_jerk = (curr_latacc - prev_latacc) / std::max(dt, 1.0e-10);

  const double latacc_max = prev_latacc + lat_jerk_lim * dt;
  const double latacc_min = prev_latacc - lat_jerk_lim * dt;

  if (curr_latacc > latacc_max) {
    input.lateral.steering_tire_angle = calcSteerFromLatacc(current_speed_, latacc_max);
  } else if (curr_latacc < latacc_min) {
    input.lateral.steering_tire_angle = calcSteerFromLatacc(current_speed_, latacc_min);
  }

  const auto filtered_latacc = calcLatAcc(input, current_speed_);
  info.filtered.lat_jerk = (filtered_latacc - prev_latacc) / std::max(dt, 1.0e-10);
}

void VehicleCmdFilter::limitActualSteerDiff(
  const double current_steer_angle, AckermannControlCommand & input, GateFilterInfo & info) const
{
  const auto actual_steer_diff_lim = getSteerDiffLim();
  info.limit.steering_diff = actual_steer_diff_lim;

  auto ds = input.lateral.steering_tire_angle - current_steer_angle;
  info.raw.steering_diff = ds;
  ds = std::clamp(ds, -actual_steer_diff_lim, actual_steer_diff_lim);
  input.lateral.steering_tire_angle = current_steer_angle + ds;
  info.filtered.steering_diff = input.lateral.steering_tire_angle - current_steer_angle;
}

void VehicleCmdFilter::limitLateralSteer(
  AckermannControlCommand & input, GateFilterInfo & info) const
{
  const float steer_limit = getSteerLim();
  info.limit.steering = steer_limit;

  info.raw.steering = input.lateral.steering_tire_angle;
  input.lateral.steering_tire_angle =
    std::clamp(input.lateral.steering_tire_angle, -steer_limit, steer_limit);

  // TODO(Horibe): support steering greater than PI/2. Now the lateral acceleration
  // calculation does not support bigger steering value than PI/2 due to tan/atan calculation.
  if (std::abs(input.lateral.steering_tire_angle) > M_PI_2f) {
    std::cerr << "[vehicle_Cmd_gate] limitLateralSteer(): steering limit is set to pi/2 since the "
                 "current filtering logic can not handle the steering larger than pi/2. Please "
                 "check the steering angle limit."
              << std::endl;

    std::clamp(input.lateral.steering_tire_angle, -M_PI_2f, M_PI_2f);
  }
  info.filtered.steering = input.lateral.steering_tire_angle;
}

void VehicleCmdFilter::limitLateralSteerRate(
  const double dt, AckermannControlCommand & input, GateFilterInfo & info) const
{
  const float steer_rate_limit = getSteerRateLim();
  info.limit.steering_rate = steer_rate_limit;

  // for steering angle rate
  info.raw.steering_rate = input.lateral.steering_tire_rotation_rate;
  input.lateral.steering_tire_rotation_rate =
    std::clamp(input.lateral.steering_tire_rotation_rate, -steer_rate_limit, steer_rate_limit);
  info.filtered.steering_rate = input.lateral.steering_tire_rotation_rate;

  // for steering angle
  const float steer_diff_limit = steer_rate_limit * dt;
  float ds = input.lateral.steering_tire_angle - prev_cmd_.lateral.steering_tire_angle;
  ds = std::clamp(ds, -steer_diff_limit, steer_diff_limit);
  input.lateral.steering_tire_angle = prev_cmd_.lateral.steering_tire_angle + ds;
  info.filtered.steering = input.lateral.steering_tire_angle;
}

void VehicleCmdFilter::filterAll(
  const double dt, const double current_steer_angle, AckermannControlCommand & cmd,
  GateFilterInfo & info) const
{
  const auto cmd_orig = cmd;
  limitLateralSteer(cmd, info);
  limitLateralSteerRate(dt, cmd, info);
  limitLongitudinalWithJerk(dt, cmd, info);
  limitLongitudinalWithAcc(dt, cmd, info);
  limitLongitudinalWithVel(cmd, info);
  limitLateralWithLatJerk(dt, cmd, info);
  limitLateralWithLatAcc(dt, cmd, info);
  limitActualSteerDiff(current_steer_angle, cmd, info);

  updateIsActivated(info, cmd, cmd_orig);
  return;
}

void VehicleCmdFilter::updateIsActivated(
  GateFilterInfo & info, const AckermannControlCommand & c1, const AckermannControlCommand & c2,
  const double tol)
{
  info.is_activated_on_steering =
    std::abs(c1.lateral.steering_tire_angle - c2.lateral.steering_tire_angle) > tol;
  info.is_activated_on_steering_rate =
    std::abs(c1.lateral.steering_tire_rotation_rate - c2.lateral.steering_tire_rotation_rate) > tol;
  info.is_activated_on_speed = std::abs(c1.longitudinal.speed - c2.longitudinal.speed) > tol;
  info.is_activated_on_acceleration =
    std::abs(c1.longitudinal.acceleration - c2.longitudinal.acceleration) > tol;
  info.is_activated_on_jerk = std::abs(c1.longitudinal.jerk - c2.longitudinal.jerk) > tol;

  info.is_activated =
    (info.is_activated_on_steering || info.is_activated_on_steering_rate ||
     info.is_activated_on_speed || info.is_activated_on_acceleration || info.is_activated_on_jerk);

  return;
}

double VehicleCmdFilter::calcSteerFromLatacc(const double v, const double latacc) const
{
  const double v_sq = std::max(v * v, 0.001);
  return std::atan(latacc * param_.wheel_base / v_sq);
}

double VehicleCmdFilter::calcLatAcc(const AckermannControlCommand & cmd) const
{
  double v = cmd.longitudinal.speed;
  return v * v * std::tan(cmd.lateral.steering_tire_angle) / param_.wheel_base;
}

double VehicleCmdFilter::calcLatAcc(const AckermannControlCommand & cmd, const double v) const
{
  return v * v * std::tan(cmd.lateral.steering_tire_angle) / param_.wheel_base;
}

double VehicleCmdFilter::limitDiff(
  const double curr, const double prev, const double diff_lim) const
{
  double diff = std::max(std::min(curr - prev, diff_lim), -diff_lim);
  return prev + diff;
}

double VehicleCmdFilter::interpolateFromSpeed(const LimitArray & limits) const
{
  // Consider only for the positive velocities.
  const auto current = std::abs(current_speed_);
  const auto reference = param_.reference_speed_points;

  // If the speed is out of range of the reference, apply zero-order hold.
  if (current <= reference.front()) {
    return limits.front();
  }
  if (current >= reference.back()) {
    return limits.back();
  }

  // Apply linear interpolation
  for (size_t i = 0; i < reference.size() - 1; ++i) {
    if (reference.at(i) <= current && current <= reference.at(i + 1)) {
      auto ratio =
        (current - reference.at(i)) / std::max(reference.at(i + 1) - reference.at(i), 1.0e-5);
      ratio = std::clamp(ratio, 0.0, 1.0);
      const auto interp = limits.at(i) + ratio * (limits.at(i + 1) - limits.at(i));
      return interp;
    }
  }

  std::cerr << "VehicleCmdFilter::interpolateFromSpeed() interpolation logic is broken. Command "
               "filter is not working. Please check the code."
            << std::endl;
  return reference.back();
}

double VehicleCmdFilter::getLonAccLim() const
{
  return interpolateFromSpeed(param_.lon_acc_lim);
}
double VehicleCmdFilter::getLonJerkLim() const
{
  return interpolateFromSpeed(param_.lon_jerk_lim);
}
double VehicleCmdFilter::getLatAccLim() const
{
  return interpolateFromSpeed(param_.lat_acc_lim);
}
double VehicleCmdFilter::getLatJerkLim() const
{
  return interpolateFromSpeed(param_.lat_jerk_lim);
}
double VehicleCmdFilter::getSteerLim() const
{
  return interpolateFromSpeed(param_.steer_lim);
}
double VehicleCmdFilter::getSteerRateLim() const
{
  return interpolateFromSpeed(param_.steer_rate_lim);
}
double VehicleCmdFilter::getSteerDiffLim() const
{
  return interpolateFromSpeed(param_.actual_steer_diff_lim);
}

}  // namespace vehicle_cmd_gate
