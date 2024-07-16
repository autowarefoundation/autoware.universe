// Copyright 2024 The Autoware Foundation.
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

#include "simple_planning_simulator/vehicle_model/sim_model_actuation_cmd.hpp"

#include "autoware_vehicle_msgs/msg/gear_command.hpp"

#include <algorithm>

bool AccelMap::readAccelMapFromCSV(const std::string & csv_path, const bool validation)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    return false;
  }

  vel_index_ = CSVLoader::getRowIndex(table);
  throttle_index_ = CSVLoader::getColumnIndex(table);
  accel_map_ = CSVLoader::getMap(table);
  if (validation && !CSVLoader::validateMap(accel_map_, true)) {
    return false;
  }
  return true;
}

bool AccelMap::getThrottle(const double acc, double vel, double & throttle) const
{
  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_index_);
  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (std::vector<double> accelerations : accel_map_) {
    interpolated_acc_vec.push_back(interpolation::lerp(vel_index_, accelerations, clamped_vel));
  }
  // calculate throttle
  // When the desired acceleration is smaller than the throttle area, return false => brake sequence
  // When the desired acceleration is greater than the throttle area, return max throttle
  if (acc < interpolated_acc_vec.front()) {
    return false;
  } else if (interpolated_acc_vec.back() < acc) {
    throttle = throttle_index_.back();
    return true;
  }
  throttle = interpolation::lerp(interpolated_acc_vec, throttle_index_, acc);
  return true;
}

double AccelMap::getAcceleration(const double throttle, const double vel) const
{
  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_index_);

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (std::vector<double> accelerations : accel_map_) {
    interpolated_acc_vec.push_back(interpolation::lerp(vel_index_, accelerations, clamped_vel));
  }

  // calculate throttle
  // When the desired acceleration is smaller than the throttle area, return min acc
  // When the desired acceleration is greater than the throttle area, return max acc
  const double clamped_throttle = CSVLoader::clampValue(throttle, throttle_index_);
  return interpolation::lerp(throttle_index_, interpolated_acc_vec, clamped_throttle);
}

/**
 * @class BrakeMap
 * @brief class to handle brake map
 */
bool BrakeMap::readBrakeMapFromCSV(const std::string & csv_path, const bool validation)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    return false;
  }

  vel_index_ = CSVLoader::getRowIndex(table);
  brake_index_ = CSVLoader::getColumnIndex(table);
  brake_map_ = CSVLoader::getMap(table);
  brake_index_rev_ = brake_index_;
  if (validation && !CSVLoader::validateMap(brake_map_, false)) {
    return false;
  }
  std::reverse(std::begin(brake_index_rev_), std::end(brake_index_rev_));

  return true;
}

double BrakeMap::getBrake(const double acc, const double vel)
{
  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_index_);

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (std::vector<double> accelerations : brake_map_) {
    interpolated_acc_vec.push_back(interpolation::lerp(vel_index_, accelerations, clamped_vel));
  }

  // calculate brake
  // When the desired acceleration is smaller than the brake area, return max brake on the map
  // When the desired acceleration is greater than the brake area, return min brake on the map
  if (acc < interpolated_acc_vec.back()) {
    return brake_index_.back();
  } else if (interpolated_acc_vec.front() < acc) {
    return brake_index_.front();
  }

  std::reverse(std::begin(interpolated_acc_vec), std::end(interpolated_acc_vec));
  return interpolation::lerp(interpolated_acc_vec, brake_index_rev_, acc);
}

double BrakeMap::getAcceleration(const double brake, const double vel) const
{
  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_index_);

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (std::vector<double> accelerations : brake_map_) {
    interpolated_acc_vec.push_back(interpolation::lerp(vel_index_, accelerations, clamped_vel));
  }

  // calculate brake
  // When the desired acceleration is smaller than the brake area, return min acc
  // When the desired acceleration is greater than the brake area, return min acc
  const double clamped_brake = CSVLoader::clampValue(brake, brake_index_);
  return interpolation::lerp(brake_index_, interpolated_acc_vec, clamped_brake);
}

bool SteerMap::readSteerMapFromCSV(const std::string & csv_path, const bool validation)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    return false;
  }

  steer_index_ = CSVLoader::getRowIndex(table);
  steer_cmd_index_ = CSVLoader::getColumnIndex(table);
  steer_map_ = CSVLoader::getMap(table);
  if (validation && !CSVLoader::validateMap(steer_map_, true)) {
    return false;
  }
  return true;
}

double SteerMap::getSteerCmd(const double steer_rate, const double steer) const
{
  const double clamped_steer = CSVLoader::clampValue(steer, steer_index_);
  std::vector<double> steer_rate_interp = {};
  for (const auto & steer_rate_vec : steer_map_) {
    steer_rate_interp.push_back(interpolation::lerp(steer_index_, steer_rate_vec, clamped_steer));
  }

  const double clamped_steer_rate = CSVLoader::clampValue(steer_rate, steer_rate_interp);
  return interpolation::lerp(steer_rate_interp, steer_cmd_index_, clamped_steer_rate);
}

double SteerMap::getSteerRate(const double steer_cmd, const double steer) const
{
  const double clamped_steer = CSVLoader::clampValue(steer, steer_index_);
  std::vector<double> steer_rate_interp = {};
  for (const auto & steer_rate_vec : steer_map_) {
    steer_rate_interp.push_back(interpolation::lerp(steer_index_, steer_rate_vec, clamped_steer));
  }

  const double clamped_steer_cmd = CSVLoader::clampValue(steer_cmd, steer_cmd_index_);
  return interpolation::lerp(steer_cmd_index_, steer_rate_interp, clamped_steer_cmd);
}

SimModelActuationCmd::SimModelActuationCmd(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double accel_delay, double accel_time_constant, double brake_delay,
  double brake_time_constant, double steer_delay, double steer_time_constant, double steer_bias,
  bool convert_accel_cmd, bool convert_brake_cmd, bool convert_steer_cmd,
  std::string accel_map_path, std::string brake_map_path, std::string steer_map_path)
: SimModelInterface(6 /* dim x */, 5 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  accel_delay_(accel_delay),
  accel_time_constant_(std::max(accel_time_constant, MIN_TIME_CONSTANT)),
  brake_delay_(brake_delay),
  brake_time_constant_(std::max(brake_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_bias_(steer_bias)
{
  initializeInputQueue(dt);
  convert_accel_cmd_ = convert_accel_cmd && accel_map_.readAccelMapFromCSV(accel_map_path);
  convert_brake_cmd_ = convert_brake_cmd && brake_map_.readBrakeMapFromCSV(brake_map_path);
  convert_steer_cmd_ = convert_steer_cmd && steer_map_.readSteerMapFromCSV(steer_map_path);
}

double SimModelActuationCmd::getX()
{
  return state_(IDX::X);
}
double SimModelActuationCmd::getY()
{
  return state_(IDX::Y);
}
double SimModelActuationCmd::getYaw()
{
  return state_(IDX::YAW);
}
double SimModelActuationCmd::getVx()
{
  return state_(IDX::VX);
}
double SimModelActuationCmd::getVy()
{
  return 0.0;
}
double SimModelActuationCmd::getAx()
{
  return state_(IDX::ACCX);
}
double SimModelActuationCmd::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}
double SimModelActuationCmd::getSteer()
{
  return state_(IDX::STEER) + steer_bias_;
}
void SimModelActuationCmd::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  accel_input_queue_.push_back(input_(IDX_U::ACCEL_DES));
  delayed_input(IDX_U::ACCEL_DES) = accel_input_queue_.front();
  accel_input_queue_.pop_front();

  brake_input_queue_.push_back(input_(IDX_U::BRAKE_DES));
  delayed_input(IDX_U::BRAKE_DES) = brake_input_queue_.front();
  brake_input_queue_.pop_front();

  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  delayed_input(IDX_U::GEAR) = input_(IDX_U::GEAR);
  delayed_input(IDX_U::SLOPE_ACCX) = input_(IDX_U::SLOPE_ACCX);

  const auto prev_state = state_;
  updateRungeKutta(dt, delayed_input);

  // take velocity limit explicitly
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  // consider gear
  // update position and velocity first, and then acceleration is calculated naturally
  updateStateWithGear(state_, prev_state, gear_, dt);
}

void SimModelActuationCmd::initializeInputQueue(const double & dt)
{
  const size_t accel_input_queue_size = static_cast<size_t>(round(accel_delay_ / dt));
  accel_input_queue_.resize(accel_input_queue_size);
  std::fill(accel_input_queue_.begin(), accel_input_queue_.end(), 0.0);

  const size_t brake_input_queue_size = static_cast<size_t>(round(brake_delay_ / dt));
  brake_input_queue_.resize(brake_input_queue_size);
  std::fill(brake_input_queue_.begin(), brake_input_queue_.end(), 0.0);

  const size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}

Eigen::VectorXd SimModelActuationCmd::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  using autoware_vehicle_msgs::msg::GearCommand;

  const double vel = std::clamp(state(IDX::VX), -vx_lim_, vx_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);

  const double accel = input(IDX_U::ACCEL_DES);
  const double brake = input(IDX_U::BRAKE_DES);
  const auto gear = input(IDX_U::GEAR);

  // 1) calculate acceleration by accel and brake command
  const double acc_des = std::clamp(
    std::invoke([&]() -> double {
      // Select the non-zero value between accel and brake and calculate the acceleration
      if (convert_accel_cmd_ && accel > 0.0) {
        return accel_map_.getAcceleration(accel, vel);
      } else if (convert_brake_cmd_ && brake > 0.0) {
        return brake_map_.getAcceleration(brake, vel);
      } else {
        // if conversion is disabled, accel command is directly used as acceleration
        return accel;
      }
    }),
    -vx_rate_lim_, vx_rate_lim_);
  // add slope acceleration considering the gear state
  const double acc_by_slope = input(IDX_U::SLOPE_ACCX);
  const double acc = std::invoke([&]() -> double {
    if (gear == GearCommand::NONE || gear == GearCommand::PARK) {
      return 0.0;
    } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
      return -acc_des + acc_by_slope;
    }
    return acc_des + acc_by_slope;
  });
  const double acc_time_constant = accel > 0.0 ? accel_time_constant_ : brake_time_constant_;

  // 2) calculate steering rate by steer command
  const double steer_rate = std::clamp(
    std::invoke([&]() -> double {
      // if conversion is enabled, calculate steering rate from steer command
      if (convert_steer_cmd_) {
        return steer_map_.getSteerRate(input(IDX_U::STEER_DES), steer) / steer_time_constant_;
      }
      // otherwise, steer command is desired steering angle, so calculate steering rate from the
      // difference between the desired steering angle and the current steering angle.
      const double steer_des = std::clamp(input(IDX_U::STEER_DES), -steer_lim_, steer_lim_);
      return -(getSteer() - steer_des) / steer_time_constant_;
    }),
    -steer_rate_lim_, steer_rate_lim_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vel * cos(yaw);
  d_state(IDX::Y) = vel * sin(yaw);
  d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
  d_state(IDX::VX) = acc;
  d_state(IDX::STEER) = steer_rate;
  d_state(IDX::ACCX) = -(acc - acc_des) / acc_time_constant;

  return d_state;
}

void SimModelActuationCmd::updateStateWithGear(
  Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear, const double dt)
{
  using autoware_vehicle_msgs::msg::GearCommand;
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state(IDX::VX) < 0.0) {
      state(IDX::VX) = 0.0;
      state(IDX::X) = prev_state(IDX::X);
      state(IDX::Y) = prev_state(IDX::Y);
      state(IDX::YAW) = prev_state(IDX::YAW);
      state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
    }
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state(IDX::VX) > 0.0) {
      state(IDX::VX) = 0.0;
      state(IDX::X) = prev_state(IDX::X);
      state(IDX::Y) = prev_state(IDX::Y);
      state(IDX::YAW) = prev_state(IDX::YAW);
      state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
    }
  } else {  // including 'gear == GearCommand::PARK'
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
    state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
  }
}
