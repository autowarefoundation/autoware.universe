// Copyright 2023 Autoware Foundation
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

#ifndef EKF_LOCALIZER__SIMPLE_FILTER_BASE_HPP_
#define EKF_LOCALIZER__SIMPLE_FILTER_BASE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

class Simple1DFilter
{
public:
  Simple1DFilter()
  {
    initialized_ = false;
    x_ = 0;
    dev_ = 1e9;
    proc_dev_x_c_ = 0.0;
    return;
  };
  void init(const double init_obs, const double obs_dev, const rclcpp::Time time)
  {
    x_ = init_obs;
    dev_ = obs_dev;
    latest_time_ = time;
    initialized_ = true;
    return;
  };
  void update(const double obs, const double obs_dev, const rclcpp::Time time)
  {
    if (!initialized_) {
      init(obs, obs_dev, time);
      return;
    }

    // Prediction step (current stddev_)
    double dt = (time - latest_time_).seconds();
    double proc_dev_x_d = proc_dev_x_c_ * dt * dt;
    dev_ = dev_ + proc_dev_x_d;

    // Update step
    double kalman_gain = dev_ / (dev_ + obs_dev);
    x_ = x_ + kalman_gain * (obs - x_);
    dev_ = (1 - kalman_gain) * dev_;

    latest_time_ = time;
    return;
  };
  void set_proc_dev(const double proc_dev) { proc_dev_x_c_ = proc_dev; }
  double get_x() const { return x_; }

private:
  bool initialized_;
  double x_;
  double dev_;
  double proc_dev_x_c_;
  rclcpp::Time latest_time_;
};

#endif  // EKF_LOCALIZER__SIMPLE_FILTER_BASE_HPP_
