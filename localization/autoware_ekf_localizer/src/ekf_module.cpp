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

#include "autoware/ekf_localizer/ekf_module.hpp"

#include "autoware/ekf_localizer/covariance.hpp"
#include "autoware/ekf_localizer/mahalanobis.hpp"
#include "autoware/ekf_localizer/matrix_types.hpp"
#include "autoware/ekf_localizer/measurement.hpp"
#include "autoware/ekf_localizer/numeric.hpp"
#include "autoware/ekf_localizer/state_transition.hpp"
#include "autoware/ekf_localizer/warning_message.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/msg_covariance.hpp>

#include <fmt/core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <utility>

namespace autoware::ekf_localizer
{

// clang-format off
#define DEBUG_PRINT_MAT(X) {if (params_.show_debug_info) {std::cout << #X << ": " << X << std::endl;}} // NOLINT
// clang-format on

EKFModule::EKFModule(std::shared_ptr<Warning> warning, const HyperParameters & params)
: warning_(std::move(warning)),
  dim_x_(6),  // x, y, yaw, yaw_bias, vx, wz
  accumulated_delay_times_(params.extend_state_step, 1.0E15),
  params_(params),
  last_angular_velocity_(0.0, 0.0, 0.0)
{
  Eigen::MatrixXd x = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd p = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;  // for x & y
  p(IDX::YAW, IDX::YAW) = 50.0;                                            // for yaw
  if (params_.enable_yaw_bias_estimation) {
    p(IDX::YAWB, IDX::YAWB) = 50.0;  // for yaw bias
  }
  p(IDX::VX, IDX::VX) = 1000.0;  // for vx
  p(IDX::WZ, IDX::WZ) = 50.0;    // for wz

  kalman_filter_.init(x, p, static_cast<int>(params_.extend_state_step));
  z_filter_.set_proc_var(params_.z_filter_proc_dev * params_.z_filter_proc_dev);
  roll_filter_.set_proc_var(params_.roll_filter_proc_dev * params_.roll_filter_proc_dev);
  pitch_filter_.set_proc_var(params_.pitch_filter_proc_dev * params_.pitch_filter_proc_dev);
}

void EKFModule::initialize(
  const PoseWithCovariance & initial_pose, const geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::MatrixXd x(dim_x_, 1);
  Eigen::MatrixXd p = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  x(IDX::X) = initial_pose.pose.pose.position.x + transform.transform.translation.x;
  x(IDX::Y) = initial_pose.pose.pose.position.y + transform.transform.translation.y;
  x(IDX::YAW) =
    tf2::getYaw(initial_pose.pose.pose.orientation) + tf2::getYaw(transform.transform.rotation);
  x(IDX::YAWB) = 0.0;
  x(IDX::VX) = 0.0;
  x(IDX::WZ) = 0.0;

  using COV_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  p(IDX::X, IDX::X) = initial_pose.pose.covariance[COV_IDX::X_X];
  p(IDX::Y, IDX::Y) = initial_pose.pose.covariance[COV_IDX::Y_Y];
  p(IDX::YAW, IDX::YAW) = initial_pose.pose.covariance[COV_IDX::YAW_YAW];

  if (params_.enable_yaw_bias_estimation) {
    p(IDX::YAWB, IDX::YAWB) = 0.0001;
  }
  p(IDX::VX, IDX::VX) = 0.01;
  p(IDX::WZ, IDX::WZ) = 0.01;

  kalman_filter_.init(x, p, static_cast<int>(params_.extend_state_step));

  const double z = initial_pose.pose.pose.position.z;

  const auto rpy = autoware::universe_utils::getRPY(initial_pose.pose.pose.orientation);

  const double z_var = initial_pose.pose.covariance[COV_IDX::Z_Z];
  const double roll_var = initial_pose.pose.covariance[COV_IDX::ROLL_ROLL];
  const double pitch_var = initial_pose.pose.covariance[COV_IDX::PITCH_PITCH];

  z_filter_.init(z, z_var);
  roll_filter_.init(rpy.x, roll_var);
  pitch_filter_.init(rpy.y, pitch_var);
}

geometry_msgs::msg::PoseStamped EKFModule::get_current_pose(
  const rclcpp::Time & current_time, bool get_biased_yaw) const
{
  const double z = z_filter_.get_x();
  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();

  const double x = kalman_filter_.getXelement(IDX::X);
  const double y = kalman_filter_.getXelement(IDX::Y);
  /*
    getXelement(IDX::YAW) is surely `biased_yaw`.
    Please note how `yaw` and `yaw_bias` are used in the state transition model and
    how the observed pose is handled in the measurement pose update.
  */
  const double biased_yaw = kalman_filter_.getXelement(IDX::YAW);
  const double yaw_bias = kalman_filter_.getXelement(IDX::YAWB);
  const double yaw = biased_yaw + yaw_bias;

  Pose current_ekf_pose;
  current_ekf_pose.header.frame_id = params_.pose_frame_id;
  current_ekf_pose.header.stamp = current_time;
  current_ekf_pose.pose.position = autoware::universe_utils::createPoint(x, y, z);
  if (get_biased_yaw) {
    current_ekf_pose.pose.orientation =
      autoware::universe_utils::createQuaternionFromRPY(roll, pitch, biased_yaw);
  } else {
    current_ekf_pose.pose.orientation =
      autoware::universe_utils::createQuaternionFromRPY(roll, pitch, yaw);
  }
  return current_ekf_pose;
}

geometry_msgs::msg::TwistStamped EKFModule::get_current_twist(
  const rclcpp::Time & current_time) const
{
  const double vx = kalman_filter_.getXelement(IDX::VX);
  const double wz = kalman_filter_.getXelement(IDX::WZ);

  Twist current_ekf_twist;
  current_ekf_twist.header.frame_id = "base_link";
  current_ekf_twist.header.stamp = current_time;
  current_ekf_twist.twist.linear.x = vx;
  current_ekf_twist.twist.angular.z = wz;
  return current_ekf_twist;
}

std::array<double, 36> EKFModule::get_current_pose_covariance() const
{
  std::array<double, 36> cov =
    ekf_covariance_to_pose_message_covariance(kalman_filter_.getLatestP());

  using COV_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  cov[COV_IDX::Z_Z] = z_filter_.get_var();
  cov[COV_IDX::ROLL_ROLL] = roll_filter_.get_var();
  cov[COV_IDX::PITCH_PITCH] = pitch_filter_.get_var();

  return cov;
}

std::array<double, 36> EKFModule::get_current_twist_covariance() const
{
  return ekf_covariance_to_twist_message_covariance(kalman_filter_.getLatestP());
}

double EKFModule::get_yaw_bias() const
{
  return kalman_filter_.getLatestX()(IDX::YAWB);
}

size_t EKFModule::find_closest_delay_time_index(double target_value) const
{
  // If target_value is too large, return last index + 1
  if (target_value > accumulated_delay_times_.back()) {
    return accumulated_delay_times_.size();
  }

  auto lower = std::lower_bound(
    accumulated_delay_times_.begin(), accumulated_delay_times_.end(), target_value);

  // If the lower bound is the first element, return its index.
  // If the lower bound is beyond the last element, return the last index.
  // If else, take the closest element.
  if (lower == accumulated_delay_times_.begin()) {
    return 0;
  }
  if (lower == accumulated_delay_times_.end()) {
    return accumulated_delay_times_.size() - 1;
  }
  // Compare the target with the lower bound and the previous element.
  auto prev = lower - 1;
  bool is_closer_to_prev = (target_value - *prev) < (*lower - target_value);

  // Return the index of the closer element.
  return is_closer_to_prev ? std::distance(accumulated_delay_times_.begin(), prev)
                           : std::distance(accumulated_delay_times_.begin(), lower);
}

void EKFModule::accumulate_delay_time(const double dt)
{
  // Shift the delay times to the right.
  std::copy_backward(
    accumulated_delay_times_.begin(), accumulated_delay_times_.end() - 1,
    accumulated_delay_times_.end());

  // Add a new element (=0) and, and add delay time to the previous elements.
  accumulated_delay_times_.front() = 0.0;
  for (size_t i = 1; i < accumulated_delay_times_.size(); ++i) {
    accumulated_delay_times_[i] += dt;
  }
}

void EKFModule::predict_with_delay(const double dt)
{
  const Eigen::MatrixXd x_curr = kalman_filter_.getLatestX();
  const Eigen::MatrixXd p_curr = kalman_filter_.getLatestP();

  const double proc_cov_vx_d = std::pow(params_.proc_stddev_vx_c * dt, 2.0);
  const double proc_cov_wz_d = std::pow(params_.proc_stddev_wz_c * dt, 2.0);
  const double proc_cov_yaw_d = std::pow(params_.proc_stddev_yaw_c * dt, 2.0);

  const Vector6d x_next = predict_next_state(x_curr, dt);
  const Matrix6d a = create_state_transition_matrix(x_curr, dt);
  const Matrix6d q = process_noise_covariance(proc_cov_yaw_d, proc_cov_vx_d, proc_cov_wz_d);
  kalman_filter_.predictWithDelay(x_next, a, q);
  ekf_dt_ = dt;
}

bool EKFModule::measurement_update_pose(
  const PoseWithCovariance & pose, const rclcpp::Time & t_curr, EKFDiagnosticInfo & pose_diag_info)
{
  if (pose.header.frame_id != params_.pose_frame_id) {
    warning_->warn_throttle(
      fmt::format(
        "pose frame_id is %s, but pose_frame is set as %s. They must be same.",
        pose.header.frame_id.c_str(), params_.pose_frame_id.c_str()),
      2000);
  }
  const Eigen::MatrixXd x_curr = kalman_filter_.getLatestX();
  DEBUG_PRINT_MAT(x_curr.transpose());

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output

  /* Calculate delay step */
  double delay_time = (t_curr - pose.header.stamp).seconds() + params_.pose_additional_delay;
  if (delay_time < 0.0) {
    warning_->warn_throttle(pose_delay_time_warning_message(delay_time), 1000);
  }

  delay_time = std::max(delay_time, 0.0);

  const size_t delay_step = find_closest_delay_time_index(delay_time);

  pose_diag_info.delay_time = std::max(delay_time, pose_diag_info.delay_time);
  pose_diag_info.delay_time_threshold = accumulated_delay_times_.back();
  if (delay_step >= params_.extend_state_step) {
    pose_diag_info.is_passed_delay_gate = false;
    warning_->warn_throttle(
      pose_delay_step_warning_message(
        pose_diag_info.delay_time, pose_diag_info.delay_time_threshold),
      2000);
    return false;
  }

  /* Since the kalman filter cannot handle the rotation angle directly,
    offset the yaw angle so that the difference from the yaw angle that ekf holds internally
    is less than 2 pi. */
  double yaw = tf2::getYaw(pose.pose.pose.orientation);
  const double ekf_yaw = kalman_filter_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error = normalize_yaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << pose.pose.pose.position.x, pose.pose.pose.position.y, yaw;

  if (has_nan(y) || has_inf(y)) {
    warning_->warn(
      "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
    return false;
  }

  /* Gate */
  const Eigen::Vector3d y_ekf(
    kalman_filter_.getXelement(delay_step * dim_x_ + IDX::X),
    kalman_filter_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw);
  const Eigen::MatrixXd p_curr = kalman_filter_.getLatestP();
  const Eigen::MatrixXd p_y = p_curr.block(0, 0, dim_y, dim_y);

  const double distance = mahalanobis(y_ekf, y, p_y);
  pose_diag_info.mahalanobis_distance = std::max(distance, pose_diag_info.mahalanobis_distance);
  if (distance > params_.pose_gate_dist) {
    pose_diag_info.is_passed_mahalanobis_gate = false;
    warning_->warn_throttle(mahalanobis_warning_message(distance, params_.pose_gate_dist), 2000);
    warning_->warn_throttle("Ignore the measurement data.", 2000);
    return false;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  const Eigen::Matrix<double, 3, 6> c = pose_measurement_matrix();
  const Eigen::Matrix3d r =
    pose_measurement_covariance(pose.pose.covariance, params_.pose_smoothing_steps);

  kalman_filter_.updateWithDelay(y, c, r, static_cast<int>(delay_step));

  // Update Simple 1D filter with considering change of roll, pitch and height (position z)
  // values due to measurement pose delay
  auto pose_with_rph_delay_compensation =
    compensate_rph_with_delay(pose, last_angular_velocity_, delay_time);
  update_simple_1d_filters(pose_with_rph_delay_compensation, params_.pose_smoothing_steps);

  // debug
  const Eigen::MatrixXd x_result = kalman_filter_.getLatestX();
  DEBUG_PRINT_MAT(x_result.transpose());
  DEBUG_PRINT_MAT((x_result - x_curr).transpose());

  return true;
}

geometry_msgs::msg::PoseWithCovarianceStamped EKFModule::compensate_rph_with_delay(
  const PoseWithCovariance & pose, tf2::Vector3 last_angular_velocity, const double delay_time)
{
  tf2::Quaternion delta_orientation;
  if (last_angular_velocity.length() > 0.0) {
    delta_orientation.setRotation(
      last_angular_velocity.normalized(), last_angular_velocity.length() * delay_time);
  } else {
    delta_orientation.setValue(0.0, 0.0, 0.0, 1.0);
  }

  tf2::Quaternion prev_orientation = tf2::Quaternion(
    pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z,
    pose.pose.pose.orientation.w);

  tf2::Quaternion curr_orientation;
  curr_orientation = prev_orientation * delta_orientation;
  curr_orientation.normalize();

  PoseWithCovariance pose_with_delay;
  pose_with_delay = pose;
  pose_with_delay.header.stamp =
    rclcpp::Time(pose.header.stamp) + rclcpp::Duration::from_seconds(delay_time);
  pose_with_delay.pose.pose.orientation.x = curr_orientation.x();
  pose_with_delay.pose.pose.orientation.y = curr_orientation.y();
  pose_with_delay.pose.pose.orientation.z = curr_orientation.z();
  pose_with_delay.pose.pose.orientation.w = curr_orientation.w();

  const auto rpy = autoware::universe_utils::getRPY(pose_with_delay.pose.pose.orientation);
  const double delta_z = kalman_filter_.getXelement(IDX::VX) * delay_time * std::sin(-rpy.y);
  pose_with_delay.pose.pose.position.z += delta_z;

  return pose_with_delay;
}

bool EKFModule::measurement_update_twist(
  const TwistWithCovariance & twist, const rclcpp::Time & t_curr,
  EKFDiagnosticInfo & twist_diag_info)
{
  if (twist.header.frame_id != "base_link") {
    warning_->warn_throttle("twist frame_id must be base_link", 2000);
  }

  last_angular_velocity_ = tf2::Vector3(0.0, 0.0, 0.0);

  const Eigen::MatrixXd x_curr = kalman_filter_.getLatestX();
  DEBUG_PRINT_MAT(x_curr.transpose());

  constexpr int dim_y = 2;  // vx, wz

  /* Calculate delay step */
  double delay_time = (t_curr - twist.header.stamp).seconds() + params_.twist_additional_delay;
  if (delay_time < 0.0) {
    warning_->warn_throttle(twist_delay_time_warning_message(delay_time), 1000);
  }
  delay_time = std::max(delay_time, 0.0);

  const size_t delay_step = find_closest_delay_time_index(delay_time);

  twist_diag_info.delay_time = std::max(delay_time, twist_diag_info.delay_time);
  twist_diag_info.delay_time_threshold = accumulated_delay_times_.back();
  if (delay_step >= params_.extend_state_step) {
    twist_diag_info.is_passed_delay_gate = false;
    warning_->warn_throttle(
      twist_delay_step_warning_message(
        twist_diag_info.delay_time, twist_diag_info.delay_time_threshold),
      2000);
    return false;
  }

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << twist.twist.twist.linear.x, twist.twist.twist.angular.z;

  if (has_nan(y) || has_inf(y)) {
    warning_->warn(
      "[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
    return false;
  }

  const Eigen::Vector2d y_ekf(
    kalman_filter_.getXelement(delay_step * dim_x_ + IDX::VX),
    kalman_filter_.getXelement(delay_step * dim_x_ + IDX::WZ));
  const Eigen::MatrixXd p_curr = kalman_filter_.getLatestP();
  const Eigen::MatrixXd p_y = p_curr.block(4, 4, dim_y, dim_y);

  const double distance = mahalanobis(y_ekf, y, p_y);
  twist_diag_info.mahalanobis_distance = std::max(distance, twist_diag_info.mahalanobis_distance);
  if (distance > params_.twist_gate_dist) {
    twist_diag_info.is_passed_mahalanobis_gate = false;
    warning_->warn_throttle(mahalanobis_warning_message(distance, params_.twist_gate_dist), 2000);
    warning_->warn_throttle("Ignore the measurement data.", 2000);
    return false;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  const Eigen::Matrix<double, 2, 6> c = twist_measurement_matrix();
  const Eigen::Matrix2d r =
    twist_measurement_covariance(twist.twist.covariance, params_.twist_smoothing_steps);

  kalman_filter_.updateWithDelay(y, c, r, static_cast<int>(delay_step));

  last_angular_velocity_ = tf2::Vector3(
    twist.twist.twist.angular.x, twist.twist.twist.angular.y, twist.twist.twist.angular.z);

  // debug
  const Eigen::MatrixXd x_result = kalman_filter_.getLatestX();
  DEBUG_PRINT_MAT(x_result.transpose());
  DEBUG_PRINT_MAT((x_result - x_curr).transpose());

  return true;
}

void EKFModule::update_simple_1d_filters(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose, const size_t smoothing_step)
{
  double z = pose.pose.pose.position.z;

  const auto rpy = autoware::universe_utils::getRPY(pose.pose.pose.orientation);

  using COV_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  double z_var = pose.pose.covariance[COV_IDX::Z_Z] * static_cast<double>(smoothing_step);
  double roll_var = pose.pose.covariance[COV_IDX::ROLL_ROLL] * static_cast<double>(smoothing_step);
  double pitch_var =
    pose.pose.covariance[COV_IDX::PITCH_PITCH] * static_cast<double>(smoothing_step);

  z_filter_.update(z, z_var, ekf_dt_);
  roll_filter_.update(rpy.x, roll_var, ekf_dt_);
  pitch_filter_.update(rpy.y, pitch_var, ekf_dt_);
}

}  // namespace autoware::ekf_localizer
