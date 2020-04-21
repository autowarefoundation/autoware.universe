/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not enable this file except in compliance with the License.
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

#include "velocity_controller.h"

VelocityController::VelocityController()
: nh_(""),
  pnh_("~"),
  tf_listener_(tf_buffer_),
  is_smooth_stop_(false),
  is_emergency_stop_(false),
  prev_acc_cmd_(0.0)
{
  // parameters
  pnh_.param("show_debug_info", show_debug_info_, bool(false));

  // parameters timer
  pnh_.param("control_rate", control_rate_, double(30.0));

  // parameters to enable functions
  pnh_.param("enable_smooth_stop", enable_smooth_stop_, bool(true));
  pnh_.param("enable_overshoot_emergency", enable_overshoot_emergency_, bool(true));
  pnh_.param("enable_slope_compensation", enable_slope_compensation_, bool(true));

  // parameters to find a closest waypoint
  pnh_.param("closest_waypoint_distance_threshold", closest_dist_thr_, double(3.0));
  pnh_.param("closest_waypoint_angle_threshold", closest_angle_thr_, double(M_PI_4));

  // parameters for stop state
  pnh_.param("stop_state_vel", stop_state_vel_, double(0.0));                          // [m/s]
  pnh_.param("stop_state_acc", stop_state_acc_, double(-2.0));                         // [m/s^2]
  pnh_.param("stop_state_entry_ego_speed", stop_state_entry_ego_speed_, double(0.2));  // [m/s]
  pnh_.param(
    "stop_state_entry_target_speed", stop_state_entry_target_speed_, double(0.1));  // [m/s]

  // parameters for delay compensation
  pnh_.param("delay_compensation_time", delay_compensation_time_, double(0.17));  // [sec]

  // parameters for emergency stop by this controller
  pnh_.param("emergency_stop_acc", emergency_stop_acc_, double(-2.0));             // [m/s^2]
  pnh_.param("emergency_stop_jerk", emergency_stop_jerk_, double(-1.5));           // [m/s^3]
  pnh_.param("emergency_overshoot_dist", emergency_overshoot_dist_, double(1.5));  // [m]

  // parameters for smooth stop
  pnh_.param(
    "smooth_stop/exit_ego_speed", smooth_stop_param_.exit_ego_speed, double(2.0));  // [m/s]
  pnh_.param(
    "smooth_stop/exit_target_speed", smooth_stop_param_.exit_target_speed, double(2.0));  // [m/s]
  pnh_.param(
    "smooth_stop/entry_ego_speed", smooth_stop_param_.entry_ego_speed, double(1.0));  // [m/s]
  pnh_.param(
    "smooth_stop/entry_target_speed", smooth_stop_param_.entry_target_speed, double(1.0));  // [m/s]
  pnh_.param(
    "smooth_stop/weak_brake_time", smooth_stop_param_.weak_brake_time, double(3.0));  // [sec]
  pnh_.param(
    "smooth_stop/weak_brake_acc", smooth_stop_param_.weak_brake_acc, double(-0.4));  // [m/s^2]
  pnh_.param(
    "smooth_stop/increasing_brake_time", smooth_stop_param_.increasing_brake_time,
    double(3.0));  // [sec]
  pnh_.param(
    "smooth_stop/increasing_brake_gradient", smooth_stop_param_.increasing_brake_gradient,
    double(-0.05));  // [m/s^3]
  pnh_.param(
    "smooth_stop/stop_brake_time", smooth_stop_param_.stop_brake_time, double(2.0));  // [sec]
  pnh_.param(
    "smooth_stop/stop_brake_acc", smooth_stop_param_.stop_brake_acc, double(-1.7));  // [m/s^2]
  pnh_.param("smooth_stop/stop_dist_", smooth_stop_param_.stop_dist_, double(3.0));  // [m/s^2]

  // parameters for acceleration limit
  pnh_.param("max_acc", max_acc_, double(2.0));   // [m/s^2]
  pnh_.param("min_acc", min_acc_, double(-5.0));  // [m/s^2]

  // parameters for jerk limit
  pnh_.param("max_jerk", max_jerk_, double(2.0));   // [m/s^3]
  pnh_.param("min_jerk", min_jerk_, double(-5.0));  // [m/s^3]

  // parameters for slope compensation
  pnh_.param("max_pitch_rad", max_pitch_rad_, double(0.1));   // [rad]
  pnh_.param("min_pitch_rad", min_pitch_rad_, double(-0.1));  // [rad]

  pnh_.param(
    "pid_controller/current_vel_threshold_pid_integration", current_vel_threshold_pid_integrate_,
    double(0.5));  // [m/s]

  // subscriber, publisher and timer
  sub_current_vel_ =
    pnh_.subscribe("current_velocity", 1, &VelocityController::callbackCurrentVelocity, this);
  sub_trajectory_ =
    pnh_.subscribe("current_trajectory", 1, &VelocityController::callbackTrajectory, this);
  pub_control_cmd_ = pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("control_cmd", 1);
  pub_debug_ = pnh_.advertise<std_msgs::Float32MultiArray>("debug_values", 1);
  timer_control_ = nh_.createTimer(
    ros::Duration(1.0 / control_rate_), &VelocityController::callbackTimerControl, this);

  // initialize PID gain
  double kp, ki, kd;
  pnh_.param("pid_controller/kp", kp, double(0.0));
  pnh_.param("pid_controller/ki", ki, double(0.0));
  pnh_.param("pid_controllerd/kd", kd, double(0.0));
  pid_vel_.setGains(kp, ki, kd);

  // initialize PID limits
  double max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d;
  pnh_.param("pid_controller/max_out", max_pid, double(0.0));     // [m/s^2]
  pnh_.param("pid_controller/min_out", min_pid, double(0.0));     // [m/s^2]
  pnh_.param("pid_controller/max_p_effort", max_p, double(0.0));  // [m/s^2]
  pnh_.param("pid_controller/min_p_effort", min_p, double(0.0));  // [m/s^2]
  pnh_.param("pid_controller/max_i_effort", max_i, double(0.0));  // [m/s^2]
  pnh_.param("pid_controller/min_i_effort", min_i, double(0.0));  // [m/s^2]
  pnh_.param("pid_controller/max_d_effort", max_d, double(0.0));  // [m/s^2]
  pnh_.param("pid_controller/min_d_effort", min_d, double(0.0));  // [m/s^2]
  pid_vel_.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);

  // set lowpass filter
  double lpf_vel_error_gain;
  pnh_.param("pid_controller/lpf_vel_error_gain", lpf_vel_error_gain, double(0.9));
  lpf_vel_error_.init(lpf_vel_error_gain);

  double lpf_pitch_gain;
  pnh_.param("lpf_pitch_gain", lpf_pitch_gain, double(0.95));
  lpf_pitch_.init(lpf_pitch_gain);

  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);

  /* wait to get vehicle position */
  while (ros::ok()) {
    if (!updateCurrentPose(5.0)) {
      ROS_INFO("waiting map to base_link at initialize.");
    } else {
      break;
    }
  }
}

void VelocityController::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  current_vel_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
}

void VelocityController::callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & msg)
{
  trajectory_ptr_ = std::make_shared<autoware_planning_msgs::Trajectory>(*msg);
}

bool VelocityController::getCurretPoseFromTF(
  const double timeout_sec, geometry_msgs::PoseStamped & ps)
{
  geometry_msgs::TransformStamped transform;
  try {
    transform =
      tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(timeout_sec));
  } catch (tf2::TransformException & ex) {
    ROS_WARN_DELAYED_THROTTLE(3.0, "cannot get map to base_link transform. %s", ex.what());
    return false;
  }
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  return true;
}

bool VelocityController::updateCurrentPose(const double timeout_sec)
{
  geometry_msgs::PoseStamped ps;
  if (!getCurretPoseFromTF(timeout_sec, ps)) {
    return false;
  }
  current_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(ps);
  return true;
}

void VelocityController::callbackTimerControl(const ros::TimerEvent & event)
{
  const bool is_pose_updated = updateCurrentPose(0.0);

  /* gurad */
  if (!is_pose_updated || !current_pose_ptr_ || !current_vel_ptr_ || !trajectory_ptr_) {
    ROS_INFO_COND(
      show_debug_info_,
      "Waiting topics, Publish stop command. pose_update: %d, pose: %d, vel: %d, trajectory: %d",
      is_pose_updated, current_pose_ptr_ != nullptr, current_vel_ptr_ != nullptr,
      trajectory_ptr_ != nullptr);
    controller_mode_ = ControlMode::INIT;
    publishCtrlCmd(stop_state_vel_, stop_state_acc_);
    return;
  }

  /* calculate command velocity & acceleration */
  const CtrlCmd ctrl_cmd = calcCtrlCmd();

  /* publish control command */
  publishCtrlCmd(ctrl_cmd.vel, ctrl_cmd.acc);
  debug_values_.data.at(DBGVAL::FLAG_SMOOTH_STOP) = is_smooth_stop_;
  debug_values_.data.at(DBGVAL::FLAG_EMERGENCY_STOP) = is_emergency_stop_;

  /* reset parameters depending on the current control mode */
  resetHandling(controller_mode_);
}

CtrlCmd VelocityController::calcCtrlCmd()
{
  /* initialize parameters */
  const double dt = getDt();

  /* -- find a closest waypoint index --
   *
   * If the closest is not found (when the threshold is exceeded), it is treated as an emergency stop.
   *
   * Outout velocity : "0" with maximum acceleration constraint
   * Output acceleration : "emergency_stop_acc_" with maximum jerk constraint
   *
   */
  int closest_idx;
  if (!vcutils::calcClosestWithThr(
        *trajectory_ptr_, current_pose_ptr_->pose, closest_angle_thr_, closest_dist_thr_,
        closest_idx)) {
    ROS_ERROR_DELAYED_THROTTLE(
      5.0,
      "calcClosestWithThr: closest not found. Emergency Stop! (dist_thr = %3.3f [m], angle_thr = "
      "%3.3f [rad])",
      closest_dist_thr_, closest_angle_thr_);
    double vel_cmd =
      applyRateFilter(0.0, prev_vel_cmd_, dt, std::fabs(emergency_stop_acc_), emergency_stop_acc_);
    double acc_cmd = applyRateFilter(
      emergency_stop_acc_, prev_acc_cmd_, dt, std::fabs(emergency_stop_jerk_),
      emergency_stop_jerk_);
    controller_mode_ = ControlMode::ERROR;
    ROS_INFO_COND(show_debug_info_, "[closest error]. vel: %3.3f, acc: %3.3f", vel_cmd, acc_cmd);
    return CtrlCmd{vel_cmd, acc_cmd};
  }

  double current_vel = current_vel_ptr_->twist.linear.x;
  int target_idx = DelayCompensator::getTrajectoryPointIndexAfterTimeDelay(
    *trajectory_ptr_, closest_idx, delay_compensation_time_, current_vel);
  double target_vel =
    calcInterpolatedTargetVelocity(*trajectory_ptr_, *current_pose_ptr_, current_vel, target_idx);
  double target_acc = DelayCompensator::getAccelerationAfterTimeDelay(
    *trajectory_ptr_, closest_idx, delay_compensation_time_, current_vel);

  /* shift check */
  const Shift shift = getCurrentShift(target_vel);
  if (shift != prev_shift_) pid_vel_.reset();
  prev_shift_ = shift;

  const double pitch_filtered = lpf_pitch_.filter(getPitch(current_pose_ptr_->pose.orientation));
  const double stop_dist = calcStopDistance(*trajectory_ptr_, closest_idx);
  writeDebugValues(dt, current_vel, target_vel, target_acc, shift, pitch_filtered, closest_idx);

  /* ===== STOPPED =====
   *
   * If the current velocity and target velocity is almost zero,
   * and the smooth stop is not working, enter the stop state.
   *
   * Outout velocity : "stop_state_vel_" (assumed to be zero, depending on the vehicle interface)
   * Output acceleration : "stop_state_acc_" with max_jerk limit. (depending on the vehicle interface)
   *
   */
  if (checkIsStopped(current_vel, target_vel)) {
    double acc_cmd = calcFilteredAcc(stop_state_acc_, pitch_filtered, dt, shift);
    controller_mode_ = ControlMode::STOPPED;
    ROS_INFO_COND(show_debug_info_, "[Stopped]. vel: %3.3f, acc: %3.3f", stop_state_vel_, acc_cmd);
    return CtrlCmd{stop_state_vel_, acc_cmd};
  }

  /* ===== EMERGENCY STOP =====
   *
   * If the emergency flag is true, enter the emergency state.
   * The condition of the energency is checked in checkEmergency() function.
   * The flag is reset when the vehicle is stopped.
   *
   * Outout velocity : "0" with maximum acceleration constraint
   * Output acceleration : "emergency_stop_acc_" with max_jerk limit.
   *
   */
  is_emergency_stop_ = checkEmergency(closest_idx, target_vel);
  if (is_emergency_stop_) {
    double vel_cmd =
      applyRateFilter(0.0, prev_vel_cmd_, dt, std::fabs(emergency_stop_acc_), emergency_stop_acc_);
    double acc_cmd = applyRateFilter(
      emergency_stop_acc_, prev_acc_cmd_, dt, std::fabs(emergency_stop_jerk_),
      emergency_stop_jerk_);
    controller_mode_ = ControlMode::EMERGENCY_STOP;
    ROS_ERROR_COND(show_debug_info_, "[Emergency stop] vel: %3.3f, acc: %3.3f", 0.0, acc_cmd);
    return CtrlCmd{vel_cmd, acc_cmd};
  }

  /* ===== SMOOTH STOP =====
   *
   * If the vehicle veloicity & target velocity is low ehough, and there is a stop point nearby the ego vehicle,
   * enter the smooth stop state.
   *
   * Outout velocity : "target_vel" from the reference trajectory
   * Output acceleration : "emergency_stop_acc_" with max_jerk limit.
   *
   */
  is_smooth_stop_ = checkSmoothStop(closest_idx, target_vel);
  if (is_smooth_stop_) {
    if (!start_time_smooth_stop_) {
      start_time_smooth_stop_ = std::make_shared<ros::Time>(ros::Time::now());
    }
    double smooth_stop_acc_cmd = calcSmoothStopAcc();
    double acc_cmd = calcFilteredAcc(smooth_stop_acc_cmd, pitch_filtered, dt, shift);
    controller_mode_ = ControlMode::SMOOTH_STOP;
    ROS_WARN_COND(
      show_debug_info_, "[smooth stop]: Smooth stopping. vel: %3.3f, acc: %3.3f", target_vel,
      acc_cmd);
    return CtrlCmd{target_vel, acc_cmd};
  }

  /* ===== FEEDBACK CONTROL =====
   *
   * Execute PID feedback control.
   *
   * Outout velocity : "target_vel" from the reference trajectory
   * Output acceleration : calculated by PID controller with max_acceleration & max_jerk limit.
   *
   */
  double feedback_acc_cmd = applyVelocityFeedback(target_acc, target_vel, dt, current_vel);
  double acc_cmd = calcFilteredAcc(feedback_acc_cmd, pitch_filtered, dt, shift);
  controller_mode_ = ControlMode::PID_CONTROL;
  ROS_INFO_COND(
    show_debug_info_,
    "[feedback control]  vel: %3.3f, acc: %3.3f, dt: %3.3f, vcurr: %3.3f, vref: %3.3f "
    "feedback_acc_cmd: "
    "%3.3f, shift: %d",
    target_vel, acc_cmd, dt, current_vel, target_vel, feedback_acc_cmd, shift);
  return CtrlCmd{target_vel, acc_cmd};
}

void VelocityController::resetHandling(const ControlMode control_mode)
{
  if (control_mode == ControlMode::EMERGENCY_STOP) {
    pid_vel_.reset();
    lpf_vel_error_.reset();
    resetSmoothStop();
    if (std::fabs(current_vel_ptr_->twist.linear.x) < stop_state_entry_ego_speed_) {
      is_emergency_stop_ = false;
    }
  } else if (control_mode == ControlMode::ERROR) {
    pid_vel_.reset();
    resetSmoothStop();
  } else if (control_mode == ControlMode::STOPPED) {
    pid_vel_.reset();
    lpf_vel_error_.reset();
    resetSmoothStop();
    resetEmergencyStop();
  } else if (control_mode == ControlMode::SMOOTH_STOP) {
    pid_vel_.reset();
    lpf_vel_error_.reset();
  } else if (control_mode == ControlMode::PID_CONTROL) {
    resetSmoothStop();
  }
}

void VelocityController::publishCtrlCmd(const double vel, const double acc)
{
  prev_acc_cmd_ = acc;
  prev_vel_cmd_ = vel;

  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "base_link";
  cmd.control.velocity = vel;
  cmd.control.acceleration = acc;
  pub_control_cmd_.publish(cmd);

  // debug
  debug_values_.data.at(DBGVAL::CTRL_MODE) = static_cast<double>(controller_mode_);
  debug_values_.data.at(DBGVAL::ACCCMD_PUBLISHED) = acc;
  pub_debug_.publish(debug_values_);
  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);
}

bool VelocityController::checkSmoothStop(const int closest, const double target_vel) const
{
  if (!enable_smooth_stop_) {
    return false;
  }

  if (
    std::fabs(current_vel_ptr_->twist.linear.x) > smooth_stop_param_.entry_ego_speed ||
    std::fabs(target_vel) > smooth_stop_param_.entry_target_speed) {
    return false;
  }

  const double stop_dist = calcStopDistance(*trajectory_ptr_, closest);
  if (std::fabs(stop_dist) < smooth_stop_param_.stop_dist_) {
    return true;  // stop point is found around ego position.
  }
  return false;
}

bool VelocityController::checkIsStopped(double current_vel, double target_vel) const
{
  if (is_smooth_stop_) return false;  // stopping.

  if (
    std::fabs(current_vel) < stop_state_entry_ego_speed_ &&
    std::fabs(target_vel) < stop_state_entry_target_speed_) {
    return true;
  } else {
    return false;
  }
}

bool VelocityController::checkEmergency(int closest, double target_vel) const
{
  // already in emergency.
  if (is_emergency_stop_) {
    return true;
  }

  // velocity is getting high when smoth stopping.
  if (
    is_smooth_stop_ &&
    (std::fabs(current_vel_ptr_->twist.linear.x) > smooth_stop_param_.exit_ego_speed)) {
    return true;
  }

  // overshoot stop line.
  if (enable_overshoot_emergency_) {
    double stop_dist = calcStopDistance(*trajectory_ptr_, closest);
    if (stop_dist < -emergency_overshoot_dist_) {
      return true;
    }
  }
  return false;
}

double VelocityController::calcFilteredAcc(
  const double raw_acc, const double pitch, const double dt, const Shift shift) const
{
  double acc_max_filtered = applyLimitFilter(raw_acc, max_acc_, min_acc_);
  debug_values_.data.at(DBGVAL::ACCCMD_ACC_LIMITED) = acc_max_filtered;

  double acc_jerk_filtered =
    applyRateFilter(acc_max_filtered, prev_acc_cmd_, dt, max_jerk_, min_jerk_);
  debug_values_.data.at(DBGVAL::ACCCMD_JERK_LIMITED) = acc_jerk_filtered;

  double acc_slope_filtered = applySlopeCompensation(acc_jerk_filtered, pitch, shift);
  debug_values_.data.at(DBGVAL::ACCCMD_SLOPE_APPLIED) = acc_slope_filtered;
  return acc_slope_filtered;
}

double VelocityController::getDt()
{
  double dt;
  if (!prev_control_time_) {
    dt = 1.0 / control_rate_;
    prev_control_time_ = std::make_shared<ros::Time>(ros::Time::now());
  } else {
    dt = (ros::Time::now() - *prev_control_time_).toSec();
    *prev_control_time_ = ros::Time::now();
  }
  const double max_dt = 1.0 / control_rate_ * 2.0;
  const double min_dt = 1.0 / control_rate_ * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

enum VelocityController::Shift VelocityController::getCurrentShift(const double target_vel) const
{
  const double ep = 1.0e-5;
  return target_vel > ep ? Shift::Forward : (target_vel < -ep ? Shift::Reverse : prev_shift_);
}

double VelocityController::getPitch(const geometry_msgs::Quaternion & quaternion) const
{
  Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  Eigen::Vector3d v = q.toRotationMatrix() * Eigen::Vector3d::UnitX();
  double den = std::max(std::sqrt(v.x() * v.x() + v.y() * v.y()), 1.0E-8 /* avoid 0 divide */);
  double pitch = (-1.0) * std::atan2(v.z(), den);
  return pitch;
}

double VelocityController::calcSmoothStopAcc()
{
  const double elapsed_time = (ros::Time::now() - *start_time_smooth_stop_).toSec();

  double acc_cmd;
  if (elapsed_time < smooth_stop_param_.weak_brake_time) {
    acc_cmd = smooth_stop_param_.weak_brake_acc;
    ROS_INFO_COND(show_debug_info_, "[VC Smooth Stop] weak breaking! acc = %f", acc_cmd);
  } else if (
    elapsed_time <
    (smooth_stop_param_.weak_brake_time + smooth_stop_param_.increasing_brake_time)) {
    const double dt = elapsed_time - smooth_stop_param_.weak_brake_time;
    acc_cmd = smooth_stop_param_.weak_brake_acc + smooth_stop_param_.increasing_brake_gradient * dt;
    ROS_INFO_COND(show_debug_info_, "[VC Smooth Stop] break increasing! acc = %f", acc_cmd);
  } else if (
    elapsed_time < (smooth_stop_param_.weak_brake_time + smooth_stop_param_.increasing_brake_time +
                    smooth_stop_param_.stop_brake_time)) {
    acc_cmd = smooth_stop_param_.stop_brake_acc;
    ROS_INFO_COND(show_debug_info_, "[VC Smooth Stop] stop breaking! acc = %f", acc_cmd);
  } else {
    acc_cmd = smooth_stop_param_.stop_brake_acc;
    ROS_INFO_COND(show_debug_info_, "[VC Smooth Stop] finish smooth stopping! acc = %f", acc_cmd);
    resetSmoothStop();
  }

  return acc_cmd;
}

double VelocityController::calcStopDistance(
  const autoware_planning_msgs::Trajectory & trajectory, const int origin) const
{
  const double zero_velocity = 0.01;
  const double origin_velocity = trajectory.points.at(origin).twist.linear.x;
  double stop_dist = 0.0;

  // search forward
  if (std::fabs(origin_velocity) > zero_velocity) {
    for (int i = origin + 1; i < (int)trajectory.points.size() - 1; ++i) {
      stop_dist +=
        vcutils::calcDistance2D(trajectory.points.at(i).pose, trajectory.points.at(i - 1).pose);
      if (std::fabs(trajectory.points.at(i).twist.linear.x) < zero_velocity) {
        break;
      }
    }
    return stop_dist;
  }

  // search backward
  for (int i = origin - 1; 0 < i; --i) {
    if (std::fabs(trajectory.points.at(i).twist.linear.x) > zero_velocity) {
      break;
    }
    stop_dist -=
      vcutils::calcDistance2D(trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose);
  }
  return stop_dist;
}

double VelocityController::calcInterpolatedTargetVelocity(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::PoseStamped & curr_pose,
  const double curr_vel, const int closest) const
{
  const double closest_vel = traj.points.at(closest).twist.linear.x;

  if (traj.points.size() < 2) {
    return closest_vel;
  }

  /* If the current position is at the edge of the reference trajectory, enable the edge velocity. Else, calc secondary
   * closest index for interpolation */
  int closest_second;
  geometry_msgs::Point rel_pos =
    vcutils::transformToRelativeCoordinate2D(curr_pose.pose.position, traj.points.at(closest).pose);
  if (closest == 0) {
    if (rel_pos.x * curr_vel <= 0.0) {
      return closest_vel;
    }
    closest_second = 1;
  } else if (closest == (int)traj.points.size() - 1) {
    if (rel_pos.x * curr_vel >= 0.0) {
      return closest_vel;
    }
    closest_second = traj.points.size() - 2;
  } else {
    const double dist1 =
      vcutils::calcDistSquared2D(traj.points.at(closest).pose, traj.points.at(closest - 1).pose);
    const double dist2 =
      vcutils::calcDistSquared2D(traj.points.at(closest).pose, traj.points.at(closest + 1).pose);
    closest_second = dist1 < dist2 ? closest - 1 : closest + 1;
  }

  /* apply linear interpolation */
  const double dist_c1 = vcutils::calcDistance2D(curr_pose.pose, traj.points.at(closest).pose);
  const double dist_c2 =
    vcutils::calcDistance2D(curr_pose.pose, traj.points.at(closest_second).pose);
  const double v1 = traj.points.at(closest).twist.linear.x;
  const double v2 = traj.points.at(closest_second).twist.linear.x;
  const double vel_interp = (dist_c1 * v2 + dist_c2 * v1) / (dist_c1 + dist_c2);

  return vel_interp;
}

double VelocityController::applyLimitFilter(
  const double input_val, const double max_val, const double min_val) const
{
  const double limitted_val = std::min(std::max(input_val, min_val), max_val);
  return limitted_val;
}

double VelocityController::applyRateFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val) const
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + (diff * dt);
  return filtered_val;
}

double VelocityController::applySlopeCompensation(
  const double input_acc, const double pitch, const Shift shift) const
{
  if (!enable_slope_compensation_) {
    return input_acc;
  }
  constexpr double gravity = 9.80665;
  const double pitch_limited = std::min(std::max(pitch, min_pitch_rad_), max_pitch_rad_);
  double sign = (shift == Shift::Forward) ? -1 : (shift == Shift::Reverse ? 1 : 0);
  double compensated_acc = input_acc + sign * gravity * std::sin(pitch_limited);
  return compensated_acc;
}

double VelocityController::applyVelocityFeedback(
  const double target_acc, const double target_vel, const double dt, const double current_vel)
{
  const bool enable_integration =
    std::fabs(current_vel) < current_vel_threshold_pid_integrate_ ? false : true;
  const double error_vel_filtered =
    lpf_vel_error_.filter(std::fabs(target_vel) - std::fabs(current_vel));

  std::vector<double> pid_contributions(3);
  const double pid_acc =
    pid_vel_.calculate(error_vel_filtered, dt, enable_integration, pid_contributions);
  const double feedbacked_acc = target_acc + pid_acc;

  debug_values_.data.at(DBGVAL::ACCCMD_PID_APPLIED) = feedbacked_acc;
  debug_values_.data.at(DBGVAL::ERROR_V_FILTERED) = error_vel_filtered;
  debug_values_.data.at(DBGVAL::ACCCMD_FB_P_CONTRIBUTION) = pid_contributions.at(0);  // P
  debug_values_.data.at(DBGVAL::ACCCMD_FB_I_CONTRIBUTION) = pid_contributions.at(1);  // I
  debug_values_.data.at(DBGVAL::ACCCMD_FB_D_CONTRIBUTION) = pid_contributions.at(2);  // D

  return feedbacked_acc;
}

void VelocityController::resetSmoothStop()
{
  is_smooth_stop_ = false;
  start_time_smooth_stop_ = nullptr;
}

void VelocityController::resetEmergencyStop() { is_emergency_stop_ = false; }

void VelocityController::writeDebugValues(
  const double dt, const double current_vel, const double target_vel, const double target_acc,
  const Shift shift, const double pitch, const int32_t closest)
{
  constexpr double rad2deg = 180.0 / 3.141592;
  const double raw_pitch = getPitch(current_pose_ptr_->pose.orientation);
  debug_values_.data.at(DBGVAL::DT) = dt;
  debug_values_.data.at(DBGVAL::CURR_V) = current_vel;
  debug_values_.data.at(DBGVAL::TARGET_V) = target_vel;
  debug_values_.data.at(DBGVAL::TARGET_ACC) = target_acc;
  debug_values_.data.at(DBGVAL::CLOSEST_V) = trajectory_ptr_->points.at(closest).twist.linear.x;
  debug_values_.data.at(DBGVAL::CLOSEST_ACC) = trajectory_ptr_->points.at(closest).accel.linear.x;
  debug_values_.data.at(DBGVAL::SHIFT) = static_cast<double>(shift);
  debug_values_.data.at(DBGVAL::PITCH_LPFED_RAD) = pitch;
  debug_values_.data.at(DBGVAL::PITCH_LPFED_DEG) = pitch * rad2deg;
  debug_values_.data.at(DBGVAL::PITCH_RAW_RAD) = raw_pitch;
  debug_values_.data.at(DBGVAL::PITCH_RAW_DEG) = raw_pitch * rad2deg;
  debug_values_.data.at(DBGVAL::ERROR_V) = target_vel - current_vel;
}
