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

#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"

#include <cmath>

namespace autoware::motion::control::mpc_lateral_controller
{
KinematicsBicycleModel::KinematicsBicycleModel(
  const double wheelbase, const double steer_lim, const double steer_tau)
: VehicleModelInterface(/* dim_x */ 3, /* dim_u */ 1, /* dim_y */ 2, wheelbase)
{
  m_steer_lim = steer_lim;
  m_steer_tau = steer_tau;
}

void KinematicsBicycleModel::calculateDiscreteMatrix(
  Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
  const double dt)
{
  auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

  /* Linearize delta around delta_r (reference delta) */
  double delta_r = atan(m_wheelbase * m_curvature);
  if (std::abs(delta_r) >= m_steer_lim) {
    delta_r = m_steer_lim * static_cast<double>(sign(delta_r));
  }
  double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));
  double velocity = m_velocity;
  if (std::abs(m_velocity) < 1e-04) {
    velocity = 1e-04 * (m_velocity >= 0 ? 1 : -1);
  }

  a_d << 0.0, velocity, 0.0, 0.0, 0.0, velocity / m_wheelbase * cos_delta_r_squared_inv, 0.0, 0.0,
    -1.0 / m_steer_tau;

  b_d << 0.0, 0.0, 1.0 / m_steer_tau;

  c_d << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  w_d << 0.0,
    -velocity * m_curvature +
      velocity / m_wheelbase * (tan(delta_r) - delta_r * cos_delta_r_squared_inv),
    0.0;

  // bilinear discretization for ZOH system
  // no discretization is needed for Cd
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  const Eigen::MatrixXd i_dt2a_inv = (I - dt * 0.5 * a_d).inverse();
  a_d = i_dt2a_inv * (I + dt * 0.5 * a_d);
  b_d = i_dt2a_inv * b_d * dt;
  w_d = i_dt2a_inv * w_d * dt;
}

void KinematicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd & u_ref)
{
  u_ref(0, 0) = std::atan(m_wheelbase * m_curvature);
}

MPCTrajectory KinematicsBicycleModel::calculatePredictedTrajectoryInWorldCoordinate(
  [[maybe_unused]] const Eigen::MatrixXd & a_d, [[maybe_unused]] const Eigen::MatrixXd & b_d,
  [[maybe_unused]] const Eigen::MatrixXd & c_d, [[maybe_unused]] const Eigen::MatrixXd & w_d,
  const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
  const MPCTrajectory & reference_trajectory, const double dt) const
{
  // Calculate predicted state in world coordinate since there is modeling errors in Frenet
  // Relative coordinate x = [lat_err, yaw_err, steer]
  // World coordinate x = [x, y, yaw, steer]

  const auto & t = reference_trajectory;

  // create initial state in the world coordinate
  Eigen::Vector3d state_w = [&]() {
    Eigen::Vector3d state = Eigen::Vector3d::Zero();
    const auto lateral_error_0 = x0(0);
    const auto yaw_error_0 = x0(1);
    state(0) = t.x.at(0) - std::sin(t.yaw.at(0)) * lateral_error_0;  // world-x
    state(1) = t.y.at(0) + std::cos(t.yaw.at(0)) * lateral_error_0;  // world-y
    state(2) = t.yaw.at(0) + yaw_error_0;                            // world-yaw
    return state;
  }();

  // update state in the world coordinate
  const auto updateState = [&](
                             const Eigen::Vector3d & state_w, const double & input, const double dt,
                             const double velocity) {
    const auto yaw = state_w(2);

    Eigen::Vector3d dstate = Eigen::Vector3d::Zero();
    dstate(0) = velocity * std::cos(yaw);
    dstate(1) = velocity * std::sin(yaw);
    dstate(2) = velocity * std::tan(input) / m_wheelbase;

    // Note: don't do "return state_w + dstate * dt", which does not work due to the lazy evaluation
    // in Eigen.
    const Eigen::Vector3d next_state = state_w + dstate * dt;
    return next_state;
  };

  MPCTrajectory mpc_predicted_trajectory;
  const auto DIM_U = getDimU();

  for (size_t i = 0; i < reference_trajectory.size(); ++i) {
    state_w = updateState(state_w, Uex(i * DIM_U, 0), dt, t.vx.at(i));
    mpc_predicted_trajectory.push_back(
      state_w(0), state_w(1), t.z.at(i), state_w(2), t.vx.at(i), t.k.at(i), t.smooth_k.at(i),
      t.relative_time.at(i));
  }
  return mpc_predicted_trajectory;
}

MPCTrajectory KinematicsBicycleModel::calculatePredictedTrajectoryInFrenetCoordinate(
  const Eigen::MatrixXd & a_d, const Eigen::MatrixXd & b_d,
  [[maybe_unused]] const Eigen::MatrixXd & c_d, const Eigen::MatrixXd & w_d,
  const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
  const MPCTrajectory & reference_trajectory, [[maybe_unused]] const double dt) const
{
  // Relative coordinate x = [lat_err, yaw_err, steer]

  Eigen::VectorXd Xex = a_d * x0 + b_d * Uex + w_d;
  MPCTrajectory mpc_predicted_trajectory;
  const auto DIM_X = getDimX();
  const auto & t = reference_trajectory;

  for (size_t i = 0; i < reference_trajectory.size(); ++i) {
    const auto lateral_error = Xex(i * DIM_X);  // model dependent
    const auto yaw_error = Xex(i * DIM_X + 1);  // model dependent
    const auto x = t.x.at(i) - std::sin(t.yaw.at(i)) * lateral_error;
    const auto y = t.y.at(i) + std::cos(t.yaw.at(i)) * lateral_error;
    const auto yaw = t.yaw.at(i) + yaw_error;
    mpc_predicted_trajectory.push_back(
      x, y, t.z.at(i), yaw, t.vx.at(i), t.k.at(i), t.smooth_k.at(i), t.relative_time.at(i));
  }
  return mpc_predicted_trajectory;
}
}  // namespace autoware::motion::control::mpc_lateral_controller
