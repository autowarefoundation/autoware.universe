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

/**
 * @file sim_model_interface.hpp
 * @brief simple planning simulator model interface class
 * @author Takamasa Horibe
 * @date 2019.08.17
 */

#ifndef SIMPLE_PLANNING_SIMULATOR_SIM_MODEL_INTERFACE_H_
#define SIMPLE_PLANNING_SIMULATOR_SIM_MODEL_INTERFACE_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>

/**
 * @class simple_planning_simulator vehicle model class
 * @brief calculate vehicle dynamics
 */
class SimModelInterface
{
protected:
  const int dim_x_;        //!< @brief dimension of state x
  const int dim_u_;        //!< @brief dimension of input u
  Eigen::VectorXd state_;  //!< @brief vehicle state vector
  Eigen::VectorXd input_;  //!< @brief vehicle input vector

public:
  /**
   * @brief constructor
   * @param [in] dim_x dimension of state x
   * @param [in] dim_u dimension of input u
   */
  SimModelInterface(int dim_x, int dim_u);

  /**
   * @brief destructor
   */
  ~SimModelInterface() = default;

  /**
   * @brief get state vector of model
   * @param [out] state state vector
   */
  void getState(Eigen::VectorXd & state);

  /**
   * @brief get input vector of model
   * @param [out] input input vector
   */
  void getInput(Eigen::VectorXd & input);

  /**
   * @brief set state vector of model
   * @param [in] state state vector
   */
  void setState(const Eigen::VectorXd & state);

  /**
   * @brief set input vector of model
   * @param [in] input input vector
   */
  void setInput(const Eigen::VectorXd & input);

  /**
   * @brief update vehicle states with Runge-Kutta methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateRungeKutta(const double & dt, const Eigen::VectorXd & input);

  /**
   * @brief update vehicle states with Euler methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateEuler(const double & dt, const Eigen::VectorXd & input);

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  virtual void update(const double & dt) = 0;

  /**
   * @brief get vehicle position x
   */
  virtual double getX() = 0;

  /**
   * @brief get vehicle position y
   */
  virtual double getY() = 0;

  /**
   * @brief get vehicle angle yaw
   */
  virtual double getYaw() = 0;

  /**
   * @brief get vehicle velocity vx
   */
  virtual double getVx() = 0;

  /**
   * @brief get vehicle angular-velocity wz
   */
  virtual double getWz() = 0;

  /**
   * @brief get vehicle steering angle
   */
  virtual double getSteer() = 0;

  /**
   * @brief calculate derivative of states with vehicle model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  virtual Eigen::VectorXd calcModel(
    const Eigen::VectorXd & state, const Eigen::VectorXd & input) = 0;
};

#endif