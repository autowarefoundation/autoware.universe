/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pid.h"

PIDController::PIDController()
{
  error_integral_ = 0;
  prev_error_ = 0;
  is_first_time_ = true;
}

double PIDController::calculate(
  double error, double dt, bool enable_integration, std::vector<double> & pid_contributions)
{
  double ret_p = kp_ * error;
  ret_p = std::min(std::max(ret_p, min_ret_p_), max_ret_p_);

  if (enable_integration) {
    error_integral_ += error * dt;
    error_integral_ = std::min(std::max(error_integral_, min_ret_i_ / ki_), max_ret_i_ / ki_);
  }
  double ret_i = ki_ * error_integral_;

  double error_differential;
  if (is_first_time_) {
    error_differential = 0;
    is_first_time_ = false;
  } else {
    error_differential = (error - prev_error_) / dt;
  }
  double ret_d = kd_ * error_differential;
  ret_d = std::min(std::max(ret_d, min_ret_d_), max_ret_d_);

  prev_error_ = error;

  pid_contributions.at(0) = ret_p;
  pid_contributions.at(1) = ret_i;
  pid_contributions.at(2) = ret_d;

  double ret = ret_p + ret_i + ret_d;
  ret = std::min(std::max(ret, min_ret_), max_ret_);

  return ret;
}

void PIDController::setGains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setLimits(
  double max_ret, double min_ret, double max_ret_p, double min_ret_p, double max_ret_i,
  double min_ret_i, double max_ret_d, double min_ret_d)
{
  max_ret_ = max_ret;
  min_ret_ = min_ret;
  max_ret_p_ = max_ret_p;
  min_ret_p_ = min_ret_p;
  max_ret_d_ = max_ret_d;
  min_ret_d_ = min_ret_d;
  max_ret_i_ = max_ret_i;
  min_ret_i_ = min_ret_i;
}

void PIDController::reset()
{
  error_integral_ = 0;
  prev_error_ = 0;
  is_first_time_ = true;
}
