
// Copyright 2022 The Autoware Foundation
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

#include "communication_delay_compensator_core.hpp"

#include <utility>

// -- HELPER METHODS --
observers::tf_t observers::get_nthOrderTF(
  const autoware::common::types::float64_t &w_cut_off_hz, const int &n)
{
  auto &&wc_rad_sec = 2.0 * M_PI * w_cut_off_hz;  // in [rad/sec]

  // float64_t time_constant_of_qfilter{};
  auto tau = 1.0 / wc_rad_sec;

  // --------------- Qfilter Construction --------------------------------------
  // Create nth order qfilter transfer function for the steering system. 1 /( tau*s + 1)&^n
  // Calculate the transfer function.
  ns_control_toolbox::tf_factor denominator{std::vector<double>{tau, 1.}};  // (tau*s+1)

  // Take power of the denominator.
  denominator.power(static_cast<unsigned int>(n));

  // Create the transfer function from a numerator an denominator.
  auto const &&q_tf = ns_control_toolbox::tf{std::vector<double>{1.}, denominator()};

  return q_tf;
}

observers::tf_t observers::get_nthOrderTFwithDampedPoles(const autoware::common::types::float64_t &w_cut_off_hz,
                                                         const int &remaining_order,
                                                         const autoware::common::types::float64_t &damping_val)
{
  auto &&wc_rad_sec = 2.0 * M_PI * w_cut_off_hz;  // in [rad/sec]

  // float64_t time_constant_of_qfilter{};
  auto tau = 1.0 / wc_rad_sec;

  // A second order damped transfer function.
  auto q_tf_damped = tf_t({1.}, {tau * tau, 2 * damping_val * tau, 1.});

  if (remaining_order > 0)
  {
    auto q_remaining_tf = get_nthOrderTF(w_cut_off_hz, remaining_order);
    q_tf_damped = q_tf_damped * q_remaining_tf;
  }

  return q_tf_damped;
}

// ------------------- Communication Delay Compensator using Forward Dynamics. ---------

observers::LateralCommunicationDelayCompensator::LateralCommunicationDelayCompensator(
  obs_model_ptr_t observer_vehicle_model,
  model_ptr_t vehicle_model,
  const observers::tf_t &qfilter_lateral,
  sLyapMatrixVecs const &lyap_matsXY,
  const float64_t &dt)
  : observer_vehicle_model_ptr_(std::move(observer_vehicle_model)),
    vehicle_model_ptr_(std::move(vehicle_model)),
    tf_qfilter_lat_{qfilter_lateral},
    vXs_{lyap_matsXY.vXs},
    vYs_{lyap_matsXY.vYs},
    dt_{dt},
    qfilter_order_{qfilter_lateral.order()}
{
  // Compute the state-space model of QGinv(s)
  ss_qfilter_lat_ = ss_t(tf_qfilter_lat_, dt_);  // Do not forget to enter the time step dt.

  xu0_ = Eigen::MatrixXd(qfilter_order_, 1);
  xd0_ = Eigen::MatrixXd(qfilter_order_, 1);

  xu0_.setZero();
  xd0_.setZero();
}

void observers::LateralCommunicationDelayCompensator::printQfilterTFs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("Transfer function of qfilter of lateral error : \n");
  tf_qfilter_lat_.print();
}

void observers::LateralCommunicationDelayCompensator::printQfilterSSs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("State-Space Matrices of qfilter of lateral error : \n");
  ss_qfilter_lat_.print();
}

void observers::LateralCommunicationDelayCompensator::printLyapMatrices() const
{
  ns_utils::print("Lyapunov X matrices : ");
  if (!vXs_.empty())
  {
    for (auto const &item : vXs_)
    {
      ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
    }
  }

  ns_utils::print("Lyapunov Y matrices : ");
  if (!vYs_.empty())
  {
    for (auto const &item : vYs_)
    {
      ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
    }
  }
}

void observers::LateralCommunicationDelayCompensator::setInitialStates()
{
  if (!is_observer_model_initial_states_set_ && observer_vehicle_model_ptr_->areInitialStatesSet())
  {

    auto const vehicle_states = observer_vehicle_model_ptr_->getInitialStates();

    xhat0_prev_ << vehicle_states(0), vehicle_states(1), vehicle_states(2), 0.;
    is_observer_model_initial_states_set_ = true;

  }
}

void observers::LateralCommunicationDelayCompensator::resetInitialState()
{

  xhat0_prev_.setZero();
  is_observer_model_initial_states_set_ = false;
}

void
observers::LateralCommunicationDelayCompensator::simulateOneStep(const state_vector_vehicle_t &current_measurements,
                                                                 float64_t const &prev_steering_control_cmd,
                                                                 float64_t const &current_steering_cmd,
                                                                 std::shared_ptr<DelayCompensatatorMsg> const
                                                                 &msg_compensation_results,
                                                                 std::shared_ptr<DelayCompensatorDebugMsg>
                                                                 const &msg_debug_results)
{
  // If the initial states of the state observer are not set, sets it.
  setInitialStates();

  // Compute the observer gain matrix for the current operating conditions.
  computeObserverGains(current_measurements);

  // Filter the current input and store it in the as the filtered previous input.
  qfilterControlCommand(current_steering_cmd);

  // Run the state observer to estimate the current state.
  estimateVehicleStates(current_measurements, prev_steering_control_cmd, current_steering_cmd);

  // Final assignment steps.
  msg_debug_results->lat_uf = static_cast<double>(static_cast<float>(current_qfiltered_control_cmd_));
  msg_debug_results->lat_ey_hat = static_cast<double>(static_cast<float>(prev_yobs_(0)));
  msg_debug_results->lat_eyaw_hat = static_cast<double>(static_cast<float>(prev_yobs_(1)));
  msg_debug_results->lat_steering_hat = static_cast<double>(static_cast<float>(prev_yobs_(2)));
  msg_debug_results->lat_duf = static_cast<double>(static_cast<float>(df_d0_));

  msg_compensation_results->lateral_deviation_error_compensation_ref = yv_d0_(0);
  msg_compensation_results->heading_angle_error_compensation_ref = yv_d0_(1);
  msg_compensation_results->steering_compensation_ref = yv_d0_(2);

  // Debug
  // end of debug
}

void observers::LateralCommunicationDelayCompensator::computeObserverGains(
  const state_vector_vehicle_t &current_measurements)
{
  // Get the current nonlinear Lyapunov parameters.
  theta_params_.setZero();
  observer_vehicle_model_ptr_->evaluateNonlinearTermsForLyap(theta_params_, current_measurements);

  // Compute the parametric lyapunov matrices.
  auto Xc = vXs_.back();  // X0, Y0 are stored at the end.
  auto Yc = vYs_.back();

  // P(th) = P0 + th1*P1 + ...
  for (size_t k = 0; k < vXs_.size() - 1; ++k)
  {
    Xc += theta_params_(static_cast<Eigen::Index>(k)) * vXs_[k];
    Yc += theta_params_(static_cast<Eigen::Index>(k)) * vYs_[k];
  }

  Lobs_ = Yc * Xc.inverse();

  // DEBUG
  //  ns_utils::print("Current Thetas : ");
  //  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(theta_params_));
  //
  //  ns_utils::print("Current Observer Gains : ");
  //  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Lobs_.transpose()));
}

void observers::LateralCommunicationDelayCompensator::qfilterControlCommand(const autoware::common::types::float64_t &current_control_cmd)
{
  // First give the output, then update the states.
  prev_qfiltered_control_cmd_ = current_qfiltered_control_cmd_;
  current_qfiltered_control_cmd_ = ss_qfilter_lat_.simulateOneStep(xu0_, current_control_cmd);
}

/**
 * @brief Two-steps estimation method is used as we need the current state estimate. To do this,
 * we use the previous values for the vehicle model. Upon estimating the current states, we
 * predict the next states and broadcast it.
 * */
void observers::LateralCommunicationDelayCompensator
::estimateVehicleStates(state_vector_vehicle_t const &current_measurements,
                        float64_t const &,/*prev_steering_control_cmd,*/
                        float64_t const &/*current_steering_cmd*/)
{

  /**
  * xbar = A @ x0_hat + B * u_prev + Bwd
  * ybar = C @ xbar + D * uk_qf
  * */
  // FIRST STEP: propagate the previous states.
  xbar_temp_ = xhat0_prev_.eval();
  observer_vehicle_model_ptr_->simulateOneStep(prev_yobs_, xbar_temp_, prev_qfiltered_control_cmd_);

  xhat0_prev_ = xbar_temp_ + Lobs_.transpose() * (prev_yobs_ - current_measurements); // # xhat_k

  // Before updating the observer states, use xhat0 current estimate to simulate the disturbance input.
  // Apply the q-filter to the disturbance state
  auto dist_state = xhat0_prev_.eval().bottomRows<1>()(0);

  // UPDATE the OBSERVER STATE: Second step: simulate the current states and controls.
  observer_vehicle_model_ptr_->simulateOneStep(current_yobs_, xhat0_prev_, current_qfiltered_control_cmd_);


  /**
    * d = (u - ue^{-sT})
    * uf - dfilt = ue^{-sT}
    * We filter this state because in the current_yobs_ C row is zero, we cannot observe it from this output.
    * */
  df_d0_ = current_qfiltered_control_cmd_ - ss_qfilter_lat_.simulateOneStep(xd0_, dist_state);

  // Send the qfiltered disturbance input to the vehicle model to get the response.

  xv_d0_ = current_measurements.eval();
  vehicle_model_ptr_->simulateOneStepZeroState(yv_d0_, xv_d0_, df_d0_);

}

observers::LateralDisturbanceCompensator::LateralDisturbanceCompensator(observers::LateralDisturbanceCompensator::obs_model_ptr_t observer_vehicle_model,
                                                                        const observers::tf_t &qfilter_lateral,
                                                                        const observers::sLyapMatrixVecs &lyap_matsXY,
                                                                        const autoware::common::types::float64_t &dt)
  : observer_vehicle_model_ptr_(std::move(observer_vehicle_model)),
    tf_qfilter_lat_{qfilter_lateral},
    vXs_{lyap_matsXY.vXs},
    vYs_{lyap_matsXY.vYs},
    dt_{dt},
    qfilter_order_{qfilter_lateral.order()}
{
  // Compute the state-space model of QGinv(s)
  ss_qfilter_lat_ = ss_t(tf_qfilter_lat_, dt_);  // Do not forget to enter the time step dt.

  xu0_ = Eigen::MatrixXd(qfilter_order_, 1);
  xd0_ = Eigen::MatrixXd(qfilter_order_, 1);  // {state_vector_qfilter<qfilter_lateral.order()>::Zero()}

  xu0_.setZero();
  xd0_.setZero();
}

void observers::LateralDisturbanceCompensator::printQfilterTFs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("Transfer function of qfilter of lateral error : \n");
  tf_qfilter_lat_.print();
}

void observers::LateralDisturbanceCompensator::printQfilterSSs() const
{
  ns_utils::print(" --------- DELAY COMPENSATOR SUMMARY -------------  \n");
  ns_utils::print("State-Space Matrices of qfilter of lateral error : \n");
  ss_qfilter_lat_.print();
}

void observers::LateralDisturbanceCompensator::printLyapMatrices() const
{
  ns_utils::print("Lyapunov X matrices : ");
  if (!vXs_.empty())
  {
    for (auto const &item : vXs_)
    {
      ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
    }
  }

  ns_utils::print("Lyapunov Y matrices : ");
  if (!vYs_.empty())
  {
    for (auto const &item : vYs_)
    {
      ns_eigen_utils::printEigenMat(Eigen::MatrixXd(item));
    }
  }

}

void observers::LateralDisturbanceCompensator::setInitialStates()
{
  if (!is_vehicle_initial_states_set_ && observer_vehicle_model_ptr_->areInitialStatesSet())
  {

    auto const vehicle_states = observer_vehicle_model_ptr_->getInitialStates();

    xhat0_prev_ << vehicle_states(0), vehicle_states(1), vehicle_states(2), 0.;
    is_vehicle_initial_states_set_ = true;

  }
}

void observers::LateralDisturbanceCompensator::computeObserverGains(const observers::state_vector_vehicle_t &current_measurements)
{
  // Get the current nonlinear Lyapunov parameters.
  theta_params_.setZero();
  observer_vehicle_model_ptr_->evaluateNonlinearTermsForLyap(theta_params_, current_measurements);

  // Compute the parametric lyapunov matrices.
  auto Xc = vXs_.back();  // X0, Y0 are stored at the end.
  auto Yc = vYs_.back();

  // P(th) = P0 + th1*P1 + ...
  for (size_t k = 0; k < vXs_.size() - 1; ++k)
  {
    Xc += theta_params_(static_cast<Eigen::Index>(k)) * vXs_[k];
    Yc += theta_params_(static_cast<Eigen::Index>(k)) * vYs_[k];
  }

  Lobs_ = Yc * Xc.inverse();
}

void
observers::LateralDisturbanceCompensator::simulateOneStep(const observers::state_vector_vehicle_t &current_measurements,
                                                          const autoware::common::types::float64_t &prev_steering_control_cmd,
                                                          const autoware::common::types::float64_t &current_steering_cmd,
                                                          std::shared_ptr<DelayCompensatatorMsg> const &msg_compensation_results)
{

  setInitialStates();

  // Compute the observer gain matrix for the current operating conditions.
  computeObserverGains(current_measurements);

  // Filter the current input and store it in the as the filtered previous input.
  qfilterControlCommand(current_steering_cmd);

  // Run the state observer to estimate the current state.
  estimateVehicleStates(current_measurements, prev_steering_control_cmd, current_steering_cmd);

  // Final assignment steps.
  msg_compensation_results->steering_dob = -dist_input_;

}

void observers::LateralDisturbanceCompensator::qfilterControlCommand(const float64_t &current_control_cmd)
{
  // First give the output, then update the states.
  current_qfiltered_control_cmd_ = ss_qfilter_lat_.simulateOneStep(xu0_, current_control_cmd);
}

void observers::LateralDisturbanceCompensator::estimateVehicleStates(const observers::state_vector_vehicle_t &current_measurements,
                                                                     const float64_t &prev_steering_control_cmd,
                                                                     const float64_t &current_steering_cmd)
{
  /**
  *   xbar = A @ x0_hat + B * u_prev + Bwd + Lobs*(yhat - ytime_delay_compensator)
  *   ybar = C @ xbar + D * uk_qf
  *
  * x and y belong to the VEHICLE state observer.
  * */

  /**
  * Qfilter the time-delay state estimator output ytime_delay = y(without delay) - dcurvature
  * dcurvature is an output disturbance, we want to find out an input causing this output.
  * */


  // FIRST STEP: propagate the previous states.
  xbar_temp_ = xhat0_prev_.eval();
  observer_vehicle_model_ptr_->simulateOneStep(ybar_temp_, xbar_temp_, prev_steering_control_cmd);

  /**
  * Here using yv_d0_ is the point of DDOB. The last row of xhat is the disturbance input estimated for compensation.
  * */
  xhat0_prev_ = xbar_temp_ + Lobs_.transpose() * (ybar_temp_ - current_measurements); // # xhat_k
  dist_input_ = ss_qfilter_lat_.simulateOneStep(xd0_, xhat0_prev_.eval().bottomRows<1>()(0));

  /**
   * UPDATE the OBSERVER STATE: Second step: simulate the current states and controls.
   * */
  observer_vehicle_model_ptr_->simulateOneStep(current_yobs_, xhat0_prev_, current_steering_cmd);

}

void observers::LateralDisturbanceCompensator::resetInitialState()
{
  xhat0_prev_.setZero(); //.bottomRows<1>()(0) = 0; // .setZero();
  is_vehicle_initial_states_set_ = false;
}

