// Copyright 2018-2021 The Autoware Foundation
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

/*
 *    Representation
 * k      : reference curvature (input)
 * e      : lateral error
 * th     : heading angle error
 * steer  : steering angle
 * steer_d: desired steering angle (input)
 * v      : velocity
 * W      : wheelbase length
 * tau    : time constant for steering dynamics
 *
 *    State & Input
 * x = [e, th, steer]^T
 * u = steer_d
 *
 *    Nonlinear model
 * dx1/dt = v * sin(x2)
 * dx2/dt = v * tan(x3) / W
 * dx3/dt = -(x3 - u) / tau
 *
 *    Linearized model around reference point (v = v_r, th = th_r, steer = steer_r)
 *         [0,  vr,       0]       [    0]       [                    0]
 * dx/dt = [0,   0,       B] * x + [    0] * u + [-vr*k + A - B*steer_r]
 *         [0,   0,  -1/tau]       [1/tau]       [                    0]
 *
 * where A = vr*tan(steer_r)/W
 *       B = vr/(W*cos(steer_r)^2) (partial derivative of A with respect to steer_r)
 */

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_HPP_

#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_interface.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <string>

namespace autoware::motion::control::mpc_lateral_controller
{

/**
 * Vehicle model class of bicycle kinematics
 * @brief calculate model-related values
 */
class KinematicsBicycleModel : public VehicleModelInterface
{
public:
  /**
   * @brief constructor with parameter initialization
   * @param [in] wheelbase wheelbase length [m]
   * @param [in] steer_lim steering angle limit [rad]
   * @param [in] steer_tau steering time constant for 1d-model [s]
   */
  KinematicsBicycleModel(const double wheelbase, const double steer_lim, const double steer_tau);

  /**
   * @brief destructor
   */
  ~KinematicsBicycleModel() = default;

  /**
   * @brief calculate discrete model matrix of x_k+1 = a_d * xk + b_d * uk + w_d, yk = c_d * xk
   * @param [out] a_d coefficient matrix
   * @param [out] b_d coefficient matrix
   * @param [out] c_d coefficient matrix
   * @param [out] w_d coefficient matrix
   * @param [in] dt Discretization time [s]
   */
  void calculateDiscreteMatrix(
    Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
    const double dt) override;

  /**
   * @brief calculate reference input
   * @param [out] u_ref input
   */
  void calculateReferenceInput(Eigen::MatrixXd & u_ref) override;

  std::string modelName() override { return "kinematics"; };

  MPCTrajectory calculatePredictedTrajectoryInWorldCoordinate(
    const Eigen::MatrixXd & a_d, const Eigen::MatrixXd & b_d, const Eigen::MatrixXd & c_d,
    const Eigen::MatrixXd & w_d, const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
    const MPCTrajectory & reference_trajectory, const double dt) const override;

  MPCTrajectory calculatePredictedTrajectoryInFrenetCoordinate(
    const Eigen::MatrixXd & a_d, const Eigen::MatrixXd & b_d, const Eigen::MatrixXd & c_d,
    const Eigen::MatrixXd & w_d, const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
    const MPCTrajectory & reference_trajectory, const double dt) const override;

private:
  double m_steer_lim;  //!< @brief steering angle limit [rad]
  double m_steer_tau;  //!< @brief steering time constant for 1d-model [s]
};
}  // namespace autoware::motion::control::mpc_lateral_controller
#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_HPP_
