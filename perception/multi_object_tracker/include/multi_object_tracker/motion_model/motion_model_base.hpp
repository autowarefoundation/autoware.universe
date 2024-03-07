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

#ifndef MULTI_OBJECT_TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_
#define MULTI_OBJECT_TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_

#include "multi_object_tracker/utils/utils.hpp"

#include <Eigen/Core>
#include <kalman_filter/kalman_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

class MotionModel
{
protected:
  rclcpp::Time last_update_time_;
  bool is_initialized_{false};
  KalmanFilter ekf_;

public:
  MotionModel();
  virtual ~MotionModel() = default;

  bool checkInitialized() const { return is_initialized_; }

  bool initialize(const rclcpp::Time & time, const Eigen::MatrixXd & X, const Eigen::MatrixXd & P);

  // virtual void initialize(const rclcpp::Time & time,
  //   const Eigen::VectorXd & state, const Eigen::MatrixXd & covariance) = 0;

  // virtual void predict(const rclcpp::Time & time) = 0;
};

#endif  // MULTI_OBJECT_TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_
