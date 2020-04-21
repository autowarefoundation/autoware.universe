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

#ifndef VELOCITY_CONTROLLER_PID
#define VELOCITY_CONTROLLER_PID

#include <algorithm>
#include <vector>

class PIDController
{
public:
  PIDController();
  ~PIDController() = default;

  double calculate(
    double error, double dt, bool is_integrated, std::vector<double> & pid_contributions);
  void setGains(double kp, double ki, double kd);
  void setLimits(
    double max_ret, double min_ret, double max_ret_p, double min_ret_p, double max_ret_i,
    double min_ret_i, double max_ret_d, double min_ret_d);
  void reset();

private:
  // parameters
  double kp_;
  double ki_;
  double kd_;
  double max_ret_p_;
  double min_ret_p_;
  double max_ret_i_;
  double min_ret_i_;
  double max_ret_d_;
  double min_ret_d_;
  double max_ret_;
  double min_ret_;
  // states
  double error_integral_;
  double prev_error_;
  bool is_first_time_;
};

#endif
