/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
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

#include "vehicle_models/kinematic_vehicle_model.hpp"
#include <limits>

void ns_models::KinematicModelSingleTrackModel::updateParameters(const ParamsVehicle &params_vehicle)
{
  wheel_base_ = params_vehicle.wheel_base;
  lr_ = params_vehicle.lr;
  steering_tau_ = params_vehicle.steering_tau;
  speed_tau_ = params_vehicle.speed_tau;
  use_delay_models_ = params_vehicle.use_delay_model;
}

void ns_models::KinematicModelSingleTrackModel::systemEquations(const VehicleDynamicsBase::state_vector_ad_t &x,
                                                                const VehicleDynamicsBase::input_vector_ad_t &u,
                                                                const VehicleDynamicsBase::param_vector_ad_t &params,
                                                                VehicleDynamicsBase::state_vector_ad_t &f_xdot)
{
  // For guarding zero division.
  auto constexpr EPS = std::numeric_limits<double>::epsilon();

  // auto xw = x(0);  // Xw, Yw are the global coordinates.
  // auto yw = x(1);
  auto const &yaw_angle = x(2);  // heading angle.
  // auto s = x(3);              // distance travelled.
  auto const &ey = x(4);         // lateral error.
  auto const &e_yaw = x(5);      // heading error.
  auto const &v = x(6);          // longitudinal speed.
  auto const &delta = x(7);      // steering angle.

  // Set the reference curvature.
  auto const &kappa = params(0);        // curvature
  // auto const &vtarget = params(1);   //  target velocity

  auto const &tan_delta = tan(delta);
  auto const &beta = atan(tan_delta * lr_ / wheel_base_);

  // Unpack each of the controls
  auto const &ax_acc_brk_input = u(0);  // acceleration - brake input [m/s/s]
  auto const &steering_input = u(1);    // steering input [rad]

  // Kinematic equations.
  f_xdot(0) = v * cos(beta + yaw_angle);  // !<@brief xw_dot
  f_xdot(1) = v * sin(beta + yaw_angle);  // !<@brief yw_dot

  // f_xdot(2) = v * sin(beta) / lr_;                          // !<@brief psi_dot
  f_xdot(2) = v * tan(delta) / wheel_base_;                     // !<@brief psi_dot
  f_xdot(3) = v * cos(beta + e_yaw) / (1. - kappa * ey);        // !<@brief sdot
  f_xdot(4) = v * sin(beta + e_yaw);                            // !<@brief ey_dot

  f_xdot(5) = f_xdot(2) - kappa * f_xdot(3);  // !<@brief e_yaw_dot

  // Control states.
  if (use_delay_models_)
  {
    // f_xdot(6) = -(v - ax_acc_brk_input) / speed_tau_;

    f_xdot(6) = ax_acc_brk_input;  // !<@brief acceleration brake input
    f_xdot(7) = -(delta - steering_input) / steering_tau_;
    //f_xdot(8) = -(f_xdot(3) - vx_virtual_car) / speed_tau_;

  } else
  {
    f_xdot(6) = ax_acc_brk_input;  // !<@brief acceleration brake input
    f_xdot(7) = steering_input;

  }

  f_xdot(8) = f_xdot(2) * f_xdot(6) * cos(beta + e_yaw); // ay_dot = psi_dot x Vx
}

void ns_models::KinematicModelSingleTrackModel::testModel()
{
  ns_utils::print("\nTESTING the AUTO-DIFF FUNCTIONS .....");

  // Compute f(x, u) by codegen from the model.
  state_vector_t f_of_dx;
  f_of_dx.setZero();

  state_vector_t x;
  x.setRandom();

  input_vector_t u;
  u.setRandom();

  param_vector_t params;
  params.setZero();

  params(0) = 0.1;  // curvature
  params(1) = 3.0;  // virtual car speed.

  computeFx(x, u, params, f_of_dx);

  ns_utils::print("Forward differentiation test ... ");
  ns_eigen_utils::printEigenMat(f_of_dx);

  // Compute Jacobian --
  state_matrix_t A;
  A.setZero();

  control_matrix_t B;
  B.setZero();

  computeJacobians(x, u, params, A, B);

  ns_utils::print("Jacobian test: state matrix A and control matrix B \n");
  ns_eigen_utils::printEigenMat(A);
  ns_eigen_utils::printEigenMat(B);
}

double ns_models::KinematicModelSingleTrackModel::getLongSpeedDynamics_vdot(const double &current_long_speed,
                                                                            const double & /*current_speed_input*/) const
{
  auto const &v = current_long_speed;
  auto const &vx_acc_brk_input = current_long_speed;

  double vdot{};

  // Control states.
  if (use_delay_models_)
  {
    vdot = -(v - vx_acc_brk_input) / speed_tau_;

  } else
  {
    vdot = vx_acc_brk_input;  // !<@brief acceleration brake input
  }
  return vdot;
}

double ns_models::KinematicModelSingleTrackModel::getSteeringDynamics_deltadot(const double &current_steering,
                                                                               const double &current_steering_input) const
{
  auto &&delta = current_steering;
  auto &&steering_input = current_steering_input;

  double delta_dot{};
  // Control states.
  if (use_delay_models_)
  {
    delta_dot = -(delta - steering_input) / steering_tau_;

  } else
  {
    delta_dot = steering_input;
  }
  return delta_dot;
}
