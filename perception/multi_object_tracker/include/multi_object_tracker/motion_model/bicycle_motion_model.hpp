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

#ifndef MULTI_OBJECT_TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
#define MULTI_OBJECT_TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_

#include "multi_object_tracker/utils/utils.hpp"

#include <Eigen/Core>
#include <kalman_filter/kalman_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/detected_object.hpp"

class BicycleMotionModel
{
private:
  // attributes
  rclcpp::Logger logger_;
  rclcpp::Time last_update_time_;

private:
  // state
  bool is_initialized_{false};
  KalmanFilter ekf_;

  // extended state
  double lf_;
  double lr_;

  // motion parameters
  struct MotionParams
  {
    double q_stddev_acc_long;
    double q_stddev_acc_lat;
    double q_stddev_yaw_rate_min;
    double q_stddev_yaw_rate_max;
    double q_cov_slip_rate_min;
    double q_cov_slip_rate_max;
    double q_max_slip_angle;
    double lf_ratio;
    double lr_ratio;
    double lf_min;
    double lr_min;
    double max_vel;
    double max_slip;
    double dt_max;
  } motion_params_;

public:
  BicycleMotionModel();

  enum IDX { X = 0, Y = 1, YAW = 2, VEL = 3, SLIP = 4 };
  const char DIM = 5;

  bool init(
    const rclcpp::Time & time, const Eigen::MatrixXd & X, const Eigen::MatrixXd & P,
    const double & length);
  bool init(
    const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov, const double & vel, const double & vel_cov,
    const double & slip, const double & slip_cov, const double & length);

  bool checkInitialized() const
  {
    // if the state is not initialized, return false
    if (!is_initialized_) {
      RCLCPP_WARN(logger_, "BicycleMotionModel is not initialized.");
      return false;
    }
    return true;
  }

  void setDefaultParams();

  void setMotionParams(
    const double & q_stddev_acc_long, const double & q_stddev_acc_lat,
    const double & q_stddev_yaw_rate_min, const double & q_stddev_yaw_rate_max,
    const double & q_stddev_slip_rate_min, const double & q_stddev_slip_rate_max,
    const double & q_max_slip_angle, const double & lf_ratio, const double & lf_min,
    const double & lr_ratio, const double & lr_min);

  void setMotionLimits(const double & max_vel, const double & max_slip);

  Eigen::MatrixXd getStateVector() const
  {
    Eigen::MatrixXd X_t(DIM, 1);
    ekf_.getX(X_t);
    return X_t;
  }

  double getStateElement(unsigned int idx) const { return ekf_.getXelement(idx); }

  bool updateStatePose(const double & x, const double & y, const std::array<double, 36> & pose_cov);

  bool updateStatePoseHead(
    const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov);

  bool updateStatePoseHeadVel(
    const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
    const double & vel, const std::array<double, 36> & twist_cov);

  bool adjustPosition(const double & x, const double & y);

  bool limitStates();

  bool updateExtendedState(const double & length);

  bool predictState(const rclcpp::Time & time);

  bool predictState(const double dt, KalmanFilter & ekf) const;

  bool getPredictedState(const rclcpp::Time & time, Eigen::MatrixXd & X, Eigen::MatrixXd & P) const;

  bool getPredictedState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const;
};

#endif  // MULTI_OBJECT_TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
