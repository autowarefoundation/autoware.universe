
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

#include "simple_planning_simulator/vehicle_model/sim_model_time_delay.hpp"

/*
 *
 * SimModelTimeDelayTwist
 *
 */

SimModelTimeDelayTwist::SimModelTimeDelayTwist(
  double vx_lim, double wz_lim, double vx_rate_lim, double wz_rate_lim, double dt, double vx_delay,
  double vx_time_constant, double wz_delay, double wz_time_constant)
: SimModelInterface(5 /* dim x */, 2 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  wz_lim_(wz_lim),
  wz_rate_lim_(wz_rate_lim),
  vx_delay_(vx_delay),
  vx_time_constant_(std::max(vx_time_constant, MIN_TIME_CONSTANT)),
  wz_delay_(wz_delay),
  wz_time_constant_(std::max(wz_time_constant, MIN_TIME_CONSTANT))
{
  if (vx_time_constant < MIN_TIME_CONSTANT) {
    ROS_WARN("Settings vx_time_constant is too small, replace it by %f", MIN_TIME_CONSTANT);
  }
  if (wz_time_constant < MIN_TIME_CONSTANT) {
    ROS_WARN("Settings wz_time_constant is too small, replace it by %f", MIN_TIME_CONSTANT);
  }
  initializeInputQueue(dt);
};

double SimModelTimeDelayTwist::getX() { return state_(IDX::X); };
double SimModelTimeDelayTwist::getY() { return state_(IDX::Y); };
double SimModelTimeDelayTwist::getYaw() { return state_(IDX::YAW); };
double SimModelTimeDelayTwist::getVx() { return state_(IDX::VX); };
double SimModelTimeDelayTwist::getWz() { return state_(IDX::WZ); };
double SimModelTimeDelayTwist::getSteer() { return 0.0; };
void SimModelTimeDelayTwist::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  vx_input_queue_.push_back(input_(IDX_U::VX_DES));
  delayed_input(IDX_U::VX_DES) = vx_input_queue_.front();
  vx_input_queue_.pop_front();
  wz_input_queue_.push_back(input_(IDX_U::WZ_DES));
  delayed_input(IDX_U::WZ_DES) = wz_input_queue_.front();
  wz_input_queue_.pop_front();

  updateRungeKutta(dt, delayed_input);
};
void SimModelTimeDelayTwist::initializeInputQueue(const double & dt)
{
  size_t vx_input_queue_size = static_cast<size_t>(round(vx_delay_ / dt));
  for (size_t i = 0; i < vx_input_queue_size; i++) {
    vx_input_queue_.push_back(0.0);
  }
  size_t wz_input_queue_size = static_cast<size_t>(round(wz_delay_ / dt));
  for (size_t i = 0; i < wz_input_queue_size; i++) {
    wz_input_queue_.push_back(0.0);
  }
}

Eigen::VectorXd SimModelTimeDelayTwist::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double vx = state(IDX::VX);
  const double wz = state(IDX::WZ);
  const double yaw = state(IDX::YAW);
  const double delay_input_vx = input(IDX_U::VX_DES);
  const double delay_input_wz = input(IDX_U::WZ_DES);
  const double delay_vx_des = std::max(std::min(delay_input_vx, vx_lim_), -vx_lim_);
  const double delay_wz_des = std::max(std::min(delay_input_wz, wz_lim_), -wz_lim_);
  double vx_rate = -(vx - delay_vx_des) / vx_time_constant_;
  double wz_rate = -(wz - delay_wz_des) / wz_time_constant_;
  vx_rate = std::min(vx_rate_lim_, std::max(-vx_rate_lim_, vx_rate));
  wz_rate = std::min(wz_rate_lim_, std::max(-wz_rate_lim_, wz_rate));

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw);
  d_state(IDX::Y) = vx * sin(yaw);
  d_state(IDX::YAW) = wz;
  d_state(IDX::VX) = vx_rate;
  d_state(IDX::WZ) = wz_rate;

  return d_state;
};

/*
 *
 * SimModelTimeDelaySteer
 *
 */
SimModelTimeDelaySteer::SimModelTimeDelaySteer(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double vx_delay, double vx_time_constant, double steer_delay,
  double steer_time_constant)
: SimModelInterface(5 /* dim x */, 2 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  vx_delay_(vx_delay),
  vx_time_constant_(std::max(vx_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT))
{
  if (vx_time_constant < MIN_TIME_CONSTANT) {
    ROS_WARN("Settings vx_time_constant is too small, replace it by %f", MIN_TIME_CONSTANT);
  }
  if (steer_time_constant < MIN_TIME_CONSTANT) {
    ROS_WARN("Settings steer_time_constant is too small, replace it by %f", MIN_TIME_CONSTANT);
  }

  initializeInputQueue(dt);
};

double SimModelTimeDelaySteer::getX() { return state_(IDX::X); };
double SimModelTimeDelaySteer::getY() { return state_(IDX::Y); };
double SimModelTimeDelaySteer::getYaw() { return state_(IDX::YAW); };
double SimModelTimeDelaySteer::getVx() { return state_(IDX::VX); };
double SimModelTimeDelaySteer::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
};
double SimModelTimeDelaySteer::getSteer() { return state_(IDX::STEER); };
void SimModelTimeDelaySteer::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  vx_input_queue_.push_back(input_(IDX_U::VX_DES));
  delayed_input(IDX_U::VX_DES) = vx_input_queue_.front();
  vx_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  updateRungeKutta(dt, delayed_input);
};
void SimModelTimeDelaySteer::initializeInputQueue(const double & dt)
{
  size_t vx_input_queue_size = static_cast<size_t>(round(vx_delay_ / dt));
  for (size_t i = 0; i < vx_input_queue_size; i++) {
    vx_input_queue_.push_back(0.0);
  }
  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  for (size_t i = 0; i < steer_input_queue_size; i++) {
    steer_input_queue_.push_back(0.0);
  }
}

Eigen::VectorXd SimModelTimeDelaySteer::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double vel = state(IDX::VX);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);
  const double delay_input_vel = input(IDX_U::VX_DES);
  const double delay_input_steer = input(IDX_U::STEER_DES);
  const double delay_vx_des = std::max(std::min(delay_input_vel, vx_lim_), -vx_lim_);
  const double delay_steer_des = std::max(std::min(delay_input_steer, steer_lim_), -steer_lim_);
  double vx_rate = -(vel - delay_vx_des) / vx_time_constant_;
  double steer_rate = -(steer - delay_steer_des) / steer_time_constant_;
  vx_rate = std::min(vx_rate_lim_, std::max(-vx_rate_lim_, vx_rate));
  steer_rate = std::min(steer_rate_lim_, std::max(-steer_rate_lim_, steer_rate));

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vel * cos(yaw);
  d_state(IDX::Y) = vel * sin(yaw);
  d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
  d_state(IDX::VX) = vx_rate;
  d_state(IDX::STEER) = steer_rate;

  return d_state;
};

SimModelTimeDelaySteerAccel::SimModelTimeDelaySteerAccel(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant)
: SimModelInterface(6 /* dim x */, 3 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT))
{
  if (acc_time_constant < MIN_TIME_CONSTANT) {
    ROS_WARN("Settings acc_time_constant is too small, replace it by %f", MIN_TIME_CONSTANT);
  }
  if (steer_time_constant < MIN_TIME_CONSTANT) {
    ROS_WARN("Settings steer_time_constant is too small, replace it by %f", MIN_TIME_CONSTANT);
  }

  initializeInputQueue(dt);
};

double SimModelTimeDelaySteerAccel::getX() { return state_(IDX::X); };
double SimModelTimeDelaySteerAccel::getY() { return state_(IDX::Y); };
double SimModelTimeDelaySteerAccel::getYaw() { return state_(IDX::YAW); };
double SimModelTimeDelaySteerAccel::getVx() { return state_(IDX::VX); };
double SimModelTimeDelaySteerAccel::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
};
double SimModelTimeDelaySteerAccel::getSteer() { return state_(IDX::STEER); };
void SimModelTimeDelaySteerAccel::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  acc_input_queue_.push_back(input_(IDX_U::ACCX_DES));
  delayed_input(IDX_U::ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();
  delayed_input(IDX_U::DRIVE_SHIFT) = input_(IDX_U::DRIVE_SHIFT);

  updateRungeKutta(dt, delayed_input);
  // clip velocity and accel
  if (delayed_input(IDX_U::DRIVE_SHIFT) >= 0.0) {
    state_(IDX::VX) = std::max(0.0, std::min(state_(IDX::VX), vx_lim_));
    if (
      std::abs((state_(IDX::VX) - 0.0)) < 10e-9 || std::abs((state_(IDX::VX) - vx_lim_)) < 10e-9) {
      state_(IDX::ACCX) = 0.0;
    }
  } else {
    state_(IDX::VX) = std::min(0.0, std::max(state_(IDX::VX), -vx_lim_));
    if (
      std::abs((state_(IDX::VX) - 0.0)) < 10e-9 ||
      std::abs((state_(IDX::VX) - (-vx_lim_))) < 10e-9) {
      state_(IDX::ACCX) = 0.0;
    }
  }
}

void SimModelTimeDelaySteerAccel::initializeInputQueue(const double & dt)
{
  size_t vx_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  for (size_t i = 0; i < vx_input_queue_size; i++) {
    acc_input_queue_.push_back(0.0);
  }
  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  for (size_t i = 0; i < steer_input_queue_size; i++) {
    steer_input_queue_.push_back(0.0);
  }
}

Eigen::VectorXd SimModelTimeDelaySteerAccel::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  double vel = state(IDX::VX);
  double acc = state(IDX::ACCX);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);
  const double delay_input_acc = input(IDX_U::ACCX_DES);
  const double delay_input_steer = input(IDX_U::STEER_DES);
  const double drive_shift = input(IDX_U::DRIVE_SHIFT);
  double delay_acc_des = std::max(std::min(delay_input_acc, vx_rate_lim_), -vx_rate_lim_);
  if (!(drive_shift >= 0.0)) delay_acc_des *= -1;  // reverse front-back
  double delay_steer_des = std::max(std::min(delay_input_steer, steer_lim_), -steer_lim_);
  double accx_rate = -(acc - delay_acc_des) / acc_time_constant_;
  double steer_rate = -(steer - delay_steer_des) / steer_time_constant_;
  acc = std::min(vx_rate_lim_, std::max(-vx_rate_lim_, acc));
  steer_rate = std::min(steer_rate_lim_, std::max(-steer_rate_lim_, steer_rate));

  if (drive_shift >= 0.0) {
    vel = std::max(0.0, std::min(vel, vx_lim_));
  } else {
    vel = std::min(0.0, std::max(vel, -vx_lim_));
  }

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vel * cos(yaw);
  d_state(IDX::Y) = vel * sin(yaw);
  d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
  d_state(IDX::VX) = acc;
  d_state(IDX::STEER) = steer_rate;
  d_state(IDX::ACCX) = accx_rate;

  return d_state;
};
