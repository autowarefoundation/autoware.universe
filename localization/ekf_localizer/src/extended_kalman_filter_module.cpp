// Copyright 2018-2019 Autoware Foundation
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

#include "ekf_localizer/extended_kalman_filter_module.hpp"

#include "ekf_localizer/covariance.hpp"
#include "ekf_localizer/mahalanobis.hpp"
#include "ekf_localizer/matrix_types.hpp"
#include "ekf_localizer/measurement.hpp"
#include "ekf_localizer/numeric.hpp"
#include "ekf_localizer/state_transition.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/msg_covariance.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

ExtendedKalmanFilterModule::ExtendedKalmanFilterModule(const HyperParameters params)
: dim_x_(6),  // x, y, yaw, yaw_bias, vx, wz
  params_(params)
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;  // for x & y
  P(IDX::YAW, IDX::YAW) = 50.0;                                            // for yaw
  if (params_.enable_yaw_bias_estimation) {
    P(IDX::YAWB, IDX::YAWB) = 50.0;  // for yaw bias
  }
  P(IDX::VX, IDX::VX) = 1000.0;  // for vx
  P(IDX::WZ, IDX::WZ) = 50.0;    // for wz

  ekf_.init(X, P, params_.extend_state_step);
}

void ExtendedKalmanFilterModule::initialize(
  PoseWithCovariance & initial_pose, geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  X(IDX::X) = initial_pose.pose.pose.position.x + transform.transform.translation.x;
  X(IDX::Y) = initial_pose.pose.pose.position.y + transform.transform.translation.y;
  X(IDX::YAW) =
    tf2::getYaw(initial_pose.pose.pose.orientation) + tf2::getYaw(transform.transform.rotation);
  X(IDX::YAWB) = 0.0;
  X(IDX::VX) = 0.0;
  X(IDX::WZ) = 0.0;

  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  P(IDX::X, IDX::X) = initial_pose.pose.covariance[COV_IDX::X_X];
  P(IDX::Y, IDX::Y) = initial_pose.pose.covariance[COV_IDX::Y_Y];
  P(IDX::YAW, IDX::YAW) = initial_pose.pose.covariance[COV_IDX::YAW_YAW];

  if (params_.enable_yaw_bias_estimation) {
    P(IDX::YAWB, IDX::YAWB) = 0.0001;
  }
  P(IDX::VX, IDX::VX) = 0.01;
  P(IDX::WZ, IDX::WZ) = 0.01;

  ekf_.init(X, P, params_.extend_state_step);
}

geometry_msgs::msg::PoseStamped ExtendedKalmanFilterModule::getCurrentPose(
  const rclcpp::Time & current_time, bool get_biased_yaw) const
{
  const double x = ekf_.getXelement(IDX::X);
  const double y = ekf_.getXelement(IDX::Y);
  const double z = z_filter_.get_x();
  const double biased_yaw = ekf_.getXelement(IDX::YAW);
  const double yaw_bias = ekf_.getXelement(IDX::YAWB);
  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();
  const double yaw = biased_yaw + yaw_bias;
  geometry_msgs::msg::PoseStamped current_ekf_pose;
  current_ekf_pose.header.frame_id = params_.pose_frame_id;
  current_ekf_pose.header.stamp = current_time;
  current_ekf_pose.pose.position = tier4_autoware_utils::createPoint(x, y, z);
  if (get_biased_yaw) {
    current_ekf_pose.pose.orientation =
      tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, biased_yaw);
  } else {
    current_ekf_pose.pose.orientation =
      tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, yaw);
  }
  return current_ekf_pose;
}

geometry_msgs::msg::TwistStamped ExtendedKalmanFilterModule::getCurrentTwist(
  const rclcpp::Time & current_time) const
{
  const double vx = ekf_.getXelement(IDX::VX);
  const double wz = ekf_.getXelement(IDX::WZ);

  geometry_msgs::msg::TwistStamped current_ekf_twist;
  current_ekf_twist.header.frame_id = "base_link";
  current_ekf_twist.header.stamp = current_time;
  current_ekf_twist.twist.linear.x = vx;
  current_ekf_twist.twist.angular.z = wz;
  return current_ekf_twist;
}

std::array<double, 36> ExtendedKalmanFilterModule::getCurrentPoseCovariance() const
{
  return ekfCovarianceToPoseMessageCovariance(ekf_.getLatestP());
}

std::array<double, 36> ExtendedKalmanFilterModule::getCurrentTwistCovariance() const
{
  return ekfCovarianceToTwistMessageCovariance(ekf_.getLatestP());
}

double ExtendedKalmanFilterModule::getYawBias() const
{
  return ekf_.getLatestX()(IDX::YAWB);
}

EKFDiagnosticInfo ExtendedKalmanFilterModule::getPoseDiagInfo() const
{
  return pose_diag_info_;
}

EKFDiagnosticInfo ExtendedKalmanFilterModule::getTwistDiagInfo() const
{
  return twist_diag_info_;
}

void ExtendedKalmanFilterModule::predictWithDelay(const double dt)
{
  const Eigen::MatrixXd X_curr = ekf_.getLatestX();
  const Eigen::MatrixXd P_curr = ekf_.getLatestP();

  const double proc_cov_vx_d = std::pow(params_.proc_stddev_vx_c * dt, 2.0);
  const double proc_cov_wz_d = std::pow(params_.proc_stddev_wz_c * dt, 2.0);
  const double proc_cov_yaw_d = std::pow(params_.proc_stddev_yaw_c * dt, 2.0);

  const Vector6d X_next = predictNextState(X_curr, dt);
  const Matrix6d A = createStateTransitionMatrix(X_curr, dt);
  const Matrix6d Q = processNoiseCovariance(proc_cov_yaw_d, proc_cov_vx_d, proc_cov_wz_d);
  ekf_.predictWithDelay(X_next, A, Q);
}

void ExtendedKalmanFilterModule::measurementUpdatePoseQueue(
  AgedObjectQueue<PoseWithCovariance::SharedPtr> & pose_queue, const double dt,
  const rclcpp::Time & current_stamp)
{
  pose_diag_info_.queue_size = pose_queue.size();
  pose_diag_info_.is_passed_delay_gate = true;
  pose_diag_info_.delay_time = 0.0;
  pose_diag_info_.delay_time_threshold = 0.0;
  pose_diag_info_.is_passed_mahalanobis_gate = true;
  pose_diag_info_.mahalanobis_distance = 0.0;

  bool pose_is_updated = false;

  if (!pose_queue.empty()) {
    // save the initial size because the queue size can change in the loop
    const size_t n = pose_queue.size();
    for (size_t i = 0; i < n; ++i) {
      const auto pose = pose_queue.pop_increment_age();
      bool is_updated = measurementUpdatePose(*pose, dt, current_stamp);
      if (is_updated) {
        pose_is_updated = true;
      }
    }
  }
  pose_diag_info_.no_update_count = pose_is_updated ? 0 : (pose_diag_info_.no_update_count + 1);
}

void ExtendedKalmanFilterModule::measurementUpdateTwistQueue(
  AgedObjectQueue<TwistWithCovariance::SharedPtr> & twist_queue, const double dt,
  const rclcpp::Time & current_stamp)
{
  twist_diag_info_.queue_size = twist_queue.size();
  twist_diag_info_.is_passed_delay_gate = true;
  twist_diag_info_.delay_time = 0.0;
  twist_diag_info_.delay_time_threshold = 0.0;
  twist_diag_info_.is_passed_mahalanobis_gate = true;
  twist_diag_info_.mahalanobis_distance = 0.0;

  bool twist_is_updated = false;

  if (!twist_queue.empty()) {
    // save the initial size because the queue size can change in the loop
    const size_t n = twist_queue.size();
    for (size_t i = 0; i < n; ++i) {
      const auto twist = twist_queue.pop_increment_age();
      bool is_updated = measurementUpdateTwist(*twist, dt, current_stamp);
      if (is_updated) {
        twist_is_updated = true;
      }
    }
  }
  twist_diag_info_.no_update_count = twist_is_updated ? 0 : (twist_diag_info_.no_update_count + 1);
}

bool ExtendedKalmanFilterModule::measurementUpdatePose(
  const PoseWithCovariance & pose, const double dt, const rclcpp::Time & t_curr)
{
  // if (pose.header.frame_id != params_.pose_frame_id) {
  //   warning_.warnThrottle(
  //     fmt::format(
  //       "pose frame_id is %s, but pose_frame is set as %s. They must be same.",
  //       pose.header.frame_id.c_str(), params_.pose_frame_id.c_str()),
  //     2000);
  // }
  const Eigen::MatrixXd X_curr = ekf_.getLatestX();

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output

  /* Calculate delay step */
  double delay_time = (t_curr - pose.header.stamp).seconds() + params_.pose_additional_delay;
  // if (delay_time < 0.0) {
  //   warning_.warnThrottle(poseDelayTimeWarningMessage(delay_time), 1000);
  // }

  delay_time = std::max(delay_time, 0.0);

  const int delay_step = std::roundf(delay_time / dt);

  pose_diag_info_.delay_time = std::max(delay_time, pose_diag_info_.delay_time);
  pose_diag_info_.delay_time_threshold = params_.extend_state_step * dt;
  if (delay_step >= params_.extend_state_step) {
    pose_diag_info_.is_passed_delay_gate = false;
    // warning_.warnThrottle(
    //   poseDelayStepWarningMessage(delay_time, params_.extend_state_step, dt), 2000);
    return false;
  }

  /* Set yaw */
  double yaw = tf2::getYaw(pose.pose.pose.orientation);
  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << pose.pose.pose.position.x, pose.pose.pose.position.y, yaw;

  if (hasNan(y) || hasInf(y)) {
    // warning_.warn(
    //   "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
    return false;
  }

  /* Gate */
  const Eigen::Vector3d y_ekf(
    ekf_.getXelement(delay_step * dim_x_ + IDX::X), ekf_.getXelement(delay_step * dim_x_ + IDX::Y),
    ekf_yaw);
  const Eigen::MatrixXd P_curr = ekf_.getLatestP();
  const Eigen::MatrixXd P_y = P_curr.block(0, 0, dim_y, dim_y);

  const double distance = mahalanobis(y_ekf, y, P_y);
  pose_diag_info_.mahalanobis_distance = std::max(distance, pose_diag_info_.mahalanobis_distance);
  if (distance > params_.pose_gate_dist) {
    pose_diag_info_.is_passed_mahalanobis_gate = false;
    // warning_.warnThrottle(mahalanobisWarningMessage(distance, params_.pose_gate_dist), 2000);
    // warning_.warnThrottle("Ignore the measurement data.", 2000);
    return false;
  }

  const Eigen::Matrix<double, 3, 6> C = poseMeasurementMatrix();
  const Eigen::Matrix3d R =
    poseMeasurementCovariance(pose.pose.covariance, params_.pose_smoothing_steps);

  ekf_.updateWithDelay(y, C, R, delay_step);

  // Considering change of z value due to measurement pose delay
  const auto rpy = tier4_autoware_utils::getRPY(pose.pose.pose.orientation);
  const double dz_delay = ekf_.getXelement(IDX::VX) * delay_time * std::sin(-rpy.y);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_z_delay;
  pose_with_z_delay = pose;
  pose_with_z_delay.pose.pose.position.z += dz_delay;

  updateSimple1DFilters(pose_with_z_delay, params_.pose_smoothing_steps);

  return true;
}

bool ExtendedKalmanFilterModule::measurementUpdateTwist(
  const TwistWithCovariance & twist, const double dt, const rclcpp::Time & t_curr)
{
  if (twist.header.frame_id != "base_link") {
    // RCLCPP_WARN_THROTTLE(
    //   get_logger(), *get_clock(), std::chrono::milliseconds(2000).count(),
    //   "twist frame_id must be base_link");
  }

  const Eigen::MatrixXd X_curr = ekf_.getLatestX();

  constexpr int dim_y = 2;  // vx, wz

  /* Calculate delay step */
  double delay_time = (t_curr - twist.header.stamp).seconds() + params_.twist_additional_delay;
  // if (delay_time < 0.0) {
  //   warning_.warnThrottle(twistDelayTimeWarningMessage(delay_time), 1000);
  // }
  delay_time = std::max(delay_time, 0.0);

  const int delay_step = std::roundf(delay_time / dt);

  twist_diag_info_.delay_time = std::max(delay_time, twist_diag_info_.delay_time);
  twist_diag_info_.delay_time_threshold = params_.extend_state_step * dt;
  if (delay_step >= params_.extend_state_step) {
    twist_diag_info_.is_passed_delay_gate = false;
    // warning_.warnThrottle(
    //   twistDelayStepWarningMessage(delay_time, params_.extend_state_step, dt), 2000);
    return false;
  }

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << twist.twist.twist.linear.x, twist.twist.twist.angular.z;

  if (hasNan(y) || hasInf(y)) {
    // warning_.warn(
    //   "[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
    return false;
  }

  const Eigen::Vector2d y_ekf(
    ekf_.getXelement(delay_step * dim_x_ + IDX::VX),
    ekf_.getXelement(delay_step * dim_x_ + IDX::WZ));
  const Eigen::MatrixXd P_curr = ekf_.getLatestP();
  const Eigen::MatrixXd P_y = P_curr.block(4, 4, dim_y, dim_y);

  const double distance = mahalanobis(y_ekf, y, P_y);
  twist_diag_info_.mahalanobis_distance = std::max(distance, twist_diag_info_.mahalanobis_distance);
  if (distance > params_.twist_gate_dist) {
    twist_diag_info_.is_passed_mahalanobis_gate = false;
    // warning_.warnThrottle(mahalanobisWarningMessage(distance, params_.twist_gate_dist), 2000);
    // warning_.warnThrottle("Ignore the measurement data.", 2000);
    return false;
  }

  const Eigen::Matrix<double, 2, 6> C = twistMeasurementMatrix();
  const Eigen::Matrix2d R =
    twistMeasurementCovariance(twist.twist.covariance, params_.twist_smoothing_steps);

  ekf_.updateWithDelay(y, C, R, delay_step);

  return true;
}
