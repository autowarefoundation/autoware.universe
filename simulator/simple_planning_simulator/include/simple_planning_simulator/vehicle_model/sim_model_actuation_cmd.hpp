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

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_ACTUATION_CMD_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_ACTUATION_CMD_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "interpolation/linear_interpolation.hpp"
#include "simple_planning_simulator/utils/csv_loader.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"

#include <deque>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

/**
 * @class AccelMap
 * @brief class to handle acceleration map.
 * this class is copied from raw_vehicle_cmd_converter.
 */
class AccelMap
{
public:
  bool readAccelMapFromCSV(const std::string & csv_path, const bool validation = false);
  bool getThrottle(const double acc, const double vel, double & throttle) const;
  double getAcceleration(const double throttle, const double vel) const;
  std::vector<double> getVelIdx() const { return vel_index_; }
  std::vector<double> getThrottleIdx() const { return throttle_index_; }
  std::vector<std::vector<double>> getAccelMap() const { return accel_map_; }

private:
  std::vector<double> vel_index_;
  std::vector<double> throttle_index_;
  std::vector<std::vector<double>> accel_map_;
};

/**
 * @class BrakeMap
 * @brief class to handle brake map
 * this class is copied from raw_vehicle_cmd_converter.
 */
class BrakeMap
{
public:
  bool readBrakeMapFromCSV(const std::string & csv_path, const bool validation = false);
  double getBrake(const double acc, const double vel);
  double getAcceleration(const double brake, const double vel) const;
  std::vector<double> getVelIdx() const { return vel_index_; }
  std::vector<double> getBrakeIdx() const { return brake_index_; }
  std::vector<std::vector<double>> getBrakeMap() const { return brake_map_; }

private:
  std::vector<double> vel_index_;
  std::vector<double> brake_index_;
  std::vector<double> brake_index_rev_;
  std::vector<std::vector<double>> brake_map_;
};

/**
 * @class SteerMap
 * @brief class to handle steer map
 * this class is copied from raw_vehicle_cmd_converter.
 */
class SteerMap
{
public:
  bool readSteerMapFromCSV(const std::string & csv_path, const bool validation = false);
  double getSteerCmd(const double steer_rate, const double steer) const;
  double getSteerRate(const double steer_cmd, const double steer) const;

private:
  std::vector<double> steer_index_;
  std::vector<double> steer_cmd_index_;
  std::vector<std::vector<double>> steer_map_;
};

/**
 * @class SimModelActuationCmd
 * @brief class to handle vehicle model with actuation command
 */
class SimModelActuationCmd : public SimModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] steer_lim steering limit [rad]
   * @param [in] vx_rate_lim acceleration limit [m/ss]
   * @param [in] steer_rate_lim steering angular velocity limit [rad/ss]
   * @param [in] wheelbase vehicle wheelbase length [m]
   * @param [in] dt delta time information to set input buffer for delay
   * @param [in] accel_delay time delay for accel command [s]
   * @param [in] acc_time_constant time constant for 1D model of accel dynamics
   * @param [in] brake_delay time delay for brake command [s]
   * @param [in] brake_time_constant time constant for 1D model of brake dynamics
   * @param [in] steer_delay time delay for steering command [s]
   * @param [in] steer_time_constant time constant for 1D model of steering dynamics
   * @param [in] steer_bias steering bias [rad]
   * @param [in] convert_accel_cmd flag to convert accel command
   * @param [in] convert_brake_cmd flag to convert brake command
   * @param [in] convert_steer_cmd flag to convert steer command
   * @param [in] accel_map_path path to csv file for accel conversion map
   * @param [in] brake_map_path path to csv file for brake conversion map
   * @param [in] steer_map_path path to csv file for steer conversion map
   */
  SimModelActuationCmd(
    double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
    double dt, double accel_delay, double accel_time_constant, double brake_delay,
    double brake_time_constant, double steer_delay, double steer_time_constant, double steer_bias,
    bool convert_accel_cmd, bool convert_brake_cmd, bool convert_steer_cmd,
    std::string accel_map_path, std::string brake_map_path, std::string steer_map_path);

  /**
   * @brief default destructor
   */
  ~SimModelActuationCmd() = default;

  AccelMap accel_map_;
  BrakeMap brake_map_;
  SteerMap steer_map_;

  bool convert_accel_cmd_;
  bool convert_brake_cmd_;
  bool convert_steer_cmd_;

private:
  const double MIN_TIME_CONSTANT;  //!< @brief minimum time constant

  enum IDX {
    X = 0,
    Y,
    YAW,
    VX,
    STEER,
    ACCX,
  };
  enum IDX_U {
    ACCEL_DES = 0,
    BRAKE_DES,
    SLOPE_ACCX,
    STEER_DES,
    GEAR,
  };

  const double vx_lim_;          //!< @brief velocity limit [m/s]
  const double vx_rate_lim_;     //!< @brief acceleration limit [m/ss]
  const double steer_lim_;       //!< @brief steering limit [rad]
  const double steer_rate_lim_;  //!< @brief steering angular velocity limit [rad/s]
  const double wheelbase_;       //!< @brief vehicle wheelbase length [m]

  std::deque<double> accel_input_queue_;  //!< @brief buffer for accel command
  std::deque<double> brake_input_queue_;  //!< @brief buffer for brake command
  std::deque<double> steer_input_queue_;  //!< @brief buffer for steering command
  const double accel_delay_;              //!< @brief time delay for accel command [s]
  const double accel_time_constant_;      //!< @brief time constant for accel dynamics
  const double brake_delay_;              //!< @brief time delay for brake command [s]
  const double brake_time_constant_;      //!< @brief time constant for brake dynamics
  const double steer_delay_;              //!< @brief time delay for steering command [s]
  const double steer_time_constant_;      //!< @brief time constant for steering dynamics
  const double steer_bias_;               //!< @brief steering angle bias [rad]
  const std::string path_;                //!< @brief conversion map path

  /**
   * @brief set queue buffer for input command
   * @param [in] dt delta time
   */
  void initializeInputQueue(const double & dt);

  /**
   * @brief get vehicle position x
   */
  double getX() override;

  /**
   * @brief get vehicle position y
   */
  double getY() override;

  /**
   * @brief get vehicle angle yaw
   */
  double getYaw() override;

  /**
   * @brief get vehicle velocity vx
   */
  double getVx() override;

  /**
   * @brief get vehicle lateral velocity
   */
  double getVy() override;

  /**
   * @brief get vehicle longitudinal acceleration
   */
  double getAx() override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  double getWz() override;

  /**
   * @brief get vehicle steering angle
   */
  double getSteer() override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double & dt) override;

  /**
   * @brief calculate derivative of states with time delay steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;

  /**
   * @brief update state considering current gear
   * @param [in] state current state
   * @param [in] prev_state previous state
   * @param [in] gear current gear (defined in autoware_msgs/GearCommand)
   * @param [in] dt delta time to update state
   */
  void updateStateWithGear(
    Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear,
    const double dt);
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_ACTUATION_CMD_HPP_
