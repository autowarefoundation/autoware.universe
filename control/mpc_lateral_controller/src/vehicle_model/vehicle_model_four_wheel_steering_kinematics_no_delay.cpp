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

#include "mpc_lateral_controller/vehicle_model/vehicle_model_four_wheel_steering_kinematics_no_delay.hpp"

#include <cmath>

namespace autoware::motion::control::mpc_lateral_controller
{
KinematicsFourWheelSteeringModelNoDelay::KinematicsFourWheelSteeringModelNoDelay(
  const double wheelbase, const double steer_lim)
: VehicleModelInterface(/* dim_x */ 2, /* dim_u */ 1, /* dim_y */ 2, wheelbase)
{
  m_steer_lim = steer_lim;
}

void KinematicsFourWheelSteeringModelNoDelay::calculateDiscreteMatrix(
  Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
  const double dt)
{
  auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

  /* Linearize delta around delta_r (reference delta) */
  double delta_r = atan(0.5 * m_wheelbase * m_curvature);
  if (std::abs(delta_r) >= m_steer_lim) {
    delta_r = m_steer_lim * static_cast<double>(sign(delta_r));
  }
  double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

  a_d << 0.0, m_velocity, 0.0, 0.0;

  b_d << 0.0, 2 * m_velocity / m_wheelbase * cos_delta_r_squared_inv;

  c_d << 1.0, 0.0, 0.0, 1.0;

  w_d << 0.0, -2 * m_velocity / m_wheelbase * delta_r * cos_delta_r_squared_inv;

  // bilinear discretization for ZOH system
  // no discretization is needed for Cd
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  const Eigen::MatrixXd i_dt2a_inv = (I - dt * 0.5 * a_d).inverse();
  a_d = i_dt2a_inv * (I + dt * 0.5 * a_d);
  b_d = i_dt2a_inv * b_d * dt;
  w_d = i_dt2a_inv * w_d * dt;
}

void KinematicsFourWheelSteeringModelNoDelay::calculateReferenceInput(Eigen::MatrixXd & u_ref)
{
  u_ref(0, 0) = std::atan(0.5 * m_wheelbase * m_curvature);
}
}  // namespace autoware::motion::control::mpc_lateral_controller
