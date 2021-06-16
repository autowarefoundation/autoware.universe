// Copyright 2020 Tier IV, Inc.
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

#include <bits/stdc++.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware_utils/autoware_utils.hpp>
#include "multi_object_tracker/utils/utils.hpp"
#include "multi_object_tracker/tracker/model/unknown_tracker.hpp"

UnknownTracker::UnknownTracker(
  const rclcpp::Time & time, const autoware_perception_msgs::msg::DynamicObject & object)
: Tracker(time, object.semantic.type),
  logger_(rclcpp::get_logger("UnknownTracker")),
  last_update_time_(time)
{
  object_ = object;

  // initialize params
  use_measurement_covariance_ = false;
  float process_noise_stddev_pos_x = 0.0;                                     // [m/s]
  float process_noise_stddev_pos_y = 0.0;                                     // [m/s]
  float process_noise_stddev_vx = autoware_utils::kmph2mps(0.1);              // [m/(s*s)]
  float process_noise_stddev_vy = autoware_utils::kmph2mps(0.1);              // [m/(s*s)]
  float measurement_noise_stddev_pos_x = 0.4;                                 // [m]
  float measurement_noise_stddev_pos_y = 0.4;                                 // [m]
  float initial_measurement_noise_stddev_pos_x = 1.0;                         // [m/s]
  float initial_measurement_noise_stddev_pos_y = 1.0;                         // [m/s]
  float initial_measurement_noise_stddev_vx = autoware_utils::kmph2mps(0.1);  // [m/(s*s)]
  float initial_measurement_noise_stddev_vy = autoware_utils::kmph2mps(0.1);  // [m/(s*s)]
  process_noise_covariance_pos_x_ = std::pow(process_noise_stddev_pos_x, 2.0);
  process_noise_covariance_pos_y_ = std::pow(process_noise_stddev_pos_y, 2.0);
  process_noise_covariance_vx_ = std::pow(process_noise_stddev_vx, 2.0);
  process_noise_covariance_vy_ = std::pow(process_noise_stddev_vy, 2.0);
  measurement_noise_covariance_pos_x_ = std::pow(measurement_noise_stddev_pos_x, 2.0);
  measurement_noise_covariance_pos_y_ = std::pow(measurement_noise_stddev_pos_y, 2.0);
  initial_measurement_noise_covariance_pos_x_ =
    std::pow(initial_measurement_noise_stddev_pos_x, 2.0);
  initial_measurement_noise_covariance_pos_y_ =
    std::pow(initial_measurement_noise_stddev_pos_y, 2.0);
  initial_measurement_noise_covariance_vx_ = std::pow(initial_measurement_noise_stddev_vx, 2.0);
  initial_measurement_noise_covariance_vy_ = std::pow(initial_measurement_noise_stddev_vy, 2.0);
  max_vx_ = autoware_utils::kmph2mps(5);  // [m/s]
  max_vy_ = autoware_utils::kmph2mps(5);  // [m/s]

  // initialize X matrix
  Eigen::MatrixXd X(dim_x_, 1);
  X(IDX::X) = object.state.pose_covariance.pose.position.x;
  X(IDX::Y) = object.state.pose_covariance.pose.position.y;
  if (object.state.twist_reliable) {
    X(IDX::VX) = object.state.twist_covariance.twist.linear.x;
    X(IDX::VY) = object.state.twist_covariance.twist.linear.y;
  } else {
    X(IDX::VX) = 0.0;
    X(IDX::VY) = 0.0;
  }

  // initialize P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
  if (
    !use_measurement_covariance_ ||
    object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X] == 0.0 ||
    object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y] == 0.0)
  {
    // Rotate the covariance matrix according to the vehicle yaw
    // because initial_measurement_noise_covariance_pos_x and y
    // are in the vehicle coordinate system.
    P(IDX::X, IDX::X) = initial_measurement_noise_covariance_pos_x_;
    P(IDX::X, IDX::Y) = 0.0;
    P(IDX::Y, IDX::Y) = initial_measurement_noise_covariance_pos_y_;
    P(IDX::Y, IDX::X) = P(IDX::X, IDX::Y);
    P(IDX::VX, IDX::VX) = initial_measurement_noise_covariance_vx_;
    P(IDX::VY, IDX::VY) = initial_measurement_noise_covariance_vy_;
  } else {
    P(IDX::X, IDX::X) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X];
    P(IDX::X, IDX::Y) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_Y];
    P(IDX::Y, IDX::Y) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    P(IDX::Y, IDX::X) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_X];
    if (object.state.twist_reliable) {
      P(IDX::VX, IDX::VX) = object.state.twist_covariance.covariance[utils::MSG_COV_IDX::X_X];
      P(IDX::VY, IDX::VY) = object.state.twist_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    } else {
      P(IDX::VX, IDX::VX) = initial_measurement_noise_covariance_vx_;
      P(IDX::VY, IDX::VY) = initial_measurement_noise_covariance_vy_;
    }
  }

  ekf_.init(X, P);
}

bool UnknownTracker::predict(const rclcpp::Time & time)
{
  const double dt = (time - last_update_time_).seconds();
  bool ret = predict(dt, ekf_);
  if (ret) {last_update_time_ = time;}
  return ret;
}

bool UnknownTracker::predict(const double dt, KalmanFilter & ekf)
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * dt
   * y_{k+1}   = y_k + vx_k * dt
   * vx_{k+1}  = vx_k
   * vy_{k+1}  = vy_k
   *
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, dt,  0]
   *     [ 0, 1,  0, dt]
   *     [ 0, 0,  1,  0]
   *     [ 0, 0,  0,  1]
   */

  // X t
  Eigen::MatrixXd X_t(dim_x_, 1);  // predicted state
  ekf.getX(X_t);

  // X t+1
  Eigen::MatrixXd X_next_t(dim_x_, 1);  // predicted state
  X_next_t(IDX::X) = X_t(IDX::X) + X_t(IDX::VX) * dt;
  X_next_t(IDX::Y) = X_t(IDX::Y) + X_t(IDX::VY) * dt;
  X_next_t(IDX::VX) = X_t(IDX::VX);
  X_next_t(IDX::VY) = X_t(IDX::VY);

  // A
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(IDX::X, IDX::VX) = dt;
  A(IDX::Y, IDX::VY) = dt;

  // Q
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
  // Rotate the covariance matrix according to the vehicle yaw
  // because process_noise_covariance_pos_x and y are in the vehicle coordinate system.
  Q(IDX::X, IDX::X) = process_noise_covariance_pos_x_ * dt * dt;
  Q(IDX::X, IDX::Y) = 0.0;
  Q(IDX::Y, IDX::Y) = process_noise_covariance_pos_y_ * dt * dt;
  Q(IDX::Y, IDX::X) = Q(IDX::X, IDX::Y);
  Q(IDX::VX, IDX::VX) = process_noise_covariance_vx_ * dt * dt;
  Q(IDX::VY, IDX::VY) = process_noise_covariance_vy_ * dt * dt;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
  Eigen::MatrixXd u = Eigen::MatrixXd::Zero(dim_x_, 1);

  if (!ekf_.predict(X_next_t, A, Q)) {RCLCPP_WARN(logger_, "Pedestrian : Cannot predict");}

  return true;
}

bool UnknownTracker::measureWithPose(const autoware_perception_msgs::msg::DynamicObject & object)
{
  constexpr int dim_y = 2;  // pos x, pos y depending on Pose output

  /* Set measurement matrix */
  Eigen::MatrixXd Y(dim_y, 1);
  Y << object.state.pose_covariance.pose.position.x, object.state.pose_covariance.pose.position.y;

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::X) = 1.0;  // for pos x
  C(1, IDX::Y) = 1.0;  // for pos y

  /* Set measurement noise covariance */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (
    !use_measurement_covariance_ ||
    object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X] == 0.0 ||
    object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y] == 0.0)
  {
    R(0, 0) = measurement_noise_covariance_pos_x_;  // x - x
    R(0, 1) = 0.0;                                  // x - y
    R(1, 1) = measurement_noise_covariance_pos_y_;  // y - y
    R(1, 0) = R(0, 1);                              // y - x
  } else {
    R(0, 0) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X];
    R(0, 1) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_Y];
    R(1, 0) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_X];
    R(1, 1) = object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
  }
  if (!ekf_.update(Y, C, R)) {RCLCPP_WARN(logger_, "Pedestrian : Cannot update");}

  // limit vx, vy
  {
    Eigen::MatrixXd X_t(dim_x_, 1);
    Eigen::MatrixXd P_t(dim_x_, dim_x_);
    ekf_.getX(X_t);
    ekf_.getP(P_t);
    if (!(-max_vx_ <= X_t(IDX::VX) && X_t(IDX::VX) <= max_vx_)) {
      X_t(IDX::VX) = X_t(IDX::VX) < 0 ? -max_vx_ : max_vx_;
    }
    if (!(-max_vy_ <= X_t(IDX::VY) && X_t(IDX::VY) <= max_vy_)) {
      X_t(IDX::VY) = X_t(IDX::VY) < 0 ? -max_vy_ : max_vy_;
    }
    ekf_.init(X_t, P_t);
  }
  return true;
}

bool UnknownTracker::measure(
  const autoware_perception_msgs::msg::DynamicObject & object, const rclcpp::Time & time)
{
  object_ = object;

  if (0.01 /*10msec*/ < std::fabs((time - last_update_time_).seconds())) {
    RCLCPP_WARN(
      logger_,
      "Pedestrian : There is a large gap between predicted time and measurement time. (%f)",
      (time - last_update_time_).seconds());
  }

  measureWithPose(object);

  return true;
}

bool UnknownTracker::getEstimatedDynamicObject(
  const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObject & object)
{
  object = object_;
  object.id = getUUID();
  object.semantic.type = getType();

  // predict state
  KalmanFilter tmp_ekf_for_no_update = ekf_;
  const double dt = (time - last_update_time_).seconds();
  if (0.001 /*1msec*/ < dt) {predict(dt, tmp_ekf_for_no_update);}
  Eigen::MatrixXd X_t(dim_x_, 1);     // predicted state
  Eigen::MatrixXd P(dim_x_, dim_x_);  // predicted state
  tmp_ekf_for_no_update.getX(X_t);
  tmp_ekf_for_no_update.getP(P);

  // set position
  object.state.pose_covariance.pose.position.x = X_t(IDX::X);
  object.state.pose_covariance.pose.position.y = X_t(IDX::Y);

  // set covariance
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::X, IDX::X);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  object.state.pose_covariance.covariance[utils::MSG_COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  object.state.twist_covariance.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::VX, IDX::VX);
  object.state.twist_covariance.covariance[utils::MSG_COV_IDX::Y_Y] = P(IDX::VY, IDX::VY);

  // set velocity
  object.state.twist_covariance.twist.linear.x = X_t(IDX::VX);
  object.state.twist_covariance.twist.linear.y = X_t(IDX::VY);

  object.state.twist_reliable = true;

  return true;
}
