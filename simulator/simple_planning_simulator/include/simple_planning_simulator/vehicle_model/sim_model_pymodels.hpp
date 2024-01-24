// Copyright 2021 The Autoware Foundation.
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

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_PYMODELS_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_PYMODELS_HPP_

#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"
#include "pymodel_interconnected_model.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <deque>
#include <iostream>
#include <queue>

// #include <pybind11/embed.h>
// namespace py = pybind11;

/**
 * @class SimModelPymodels
 * @brief calculate delay steering dynamics
 */
class SimModelPymodels : public SimModelInterface /*delay steer velocity*/
{
public:
  /**
   * @brief constructor
   * @param [in] dt delta time information to set input buffer for delay
   */
  SimModelPymodels(double dt);

  /**
   * @brief destructor
   */
  ~SimModelPymodels() = default;

private:

  /* 
  Specify string names for states and inputs. So we can automatically map states and 
  inputs of this model to states and inputs of the python model.
  */

  std::vector<char*> state_names = {(char*)"POS_X", 
                                    (char*)"POS_Y", 
                                    (char*)"YAW",
                                    (char*)"YAW_RATE", 
                                    (char*)"VX",
                                    (char*)"VY", 
                                    (char*)"STEER"
                                    };

  std::vector<char*> input_names = {(char*)"VX_DES", 
                                    (char*)"STEER_DES"
                                    };

  enum IDX {
    X = 0,
    Y,
    YAW,
    YAW_RATE,
    VX,
    VY,
    STEER,
  };
  enum IDX_U {
    VX_DES = 0,
    STEER_DES,
  };

  InterconnectedModel vehicle;
  // py::scoped_interpreter guard{};

  double prev_vx_ = 0.0;
  double current_ax_ = 0.0;

  std::deque<double> vx_input_queue_;     //!< @brief buffer for velocity command
  std::deque<double> steer_input_queue_;  //!< @brief buffer for angular velocity command
  
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
   * @brief get vehicle longitudinal velocity
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
   * @brief not used for this model. calculate derivative of states with delay steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel([[maybe_unused]]const Eigen::VectorXd & state, [[maybe_unused]]const Eigen::VectorXd & input) override {return Eigen::VectorXd::Zero(dim_x_);}
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_PYMODELS_HPP_
