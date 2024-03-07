// Copyright 2024 Tier IV, Inc.
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
//
//
// Author: v1.0 Taekjin Lee
//

#include "multi_object_tracker/motion_model/motion_model_base.hpp"

MotionModel::MotionModel() : last_update_time_(rclcpp::Time(0, 0))
{
}

bool MotionModel::initialize(const rclcpp::Time & time, const Eigen::MatrixXd & X, const Eigen::MatrixXd & P)
{
  // initialize Kalman filter
  if (!ekf_.init(X, P)) return false;

  // set last update time
  last_update_time_ = time;

  // set initialized flag
  is_initialized_ = true;

  return true;
}

