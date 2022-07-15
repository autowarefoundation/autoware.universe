/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NONLINEAR_MPC_CORE__DATA_AND_PARAMETER_CONTAINER_HPP_
#define NONLINEAR_MPC_CORE__DATA_AND_PARAMETER_CONTAINER_HPP_

#include <vector>
#include "Eigen/StdVector"  // must be before the vector but clang re-orders.
#include "active_model.hpp"
#include "nonlinear_mpc_core/nmpc_data_discretization.hpp"
#include "nonlinear_mpc_core/nmpc_data_trajectory.hpp"

namespace ns_data
{
struct ParamsNMPCNode
{
  // Control implementation parameters
  double control_frequency{};
  double control_period{};

  double input_delay_time{};             // !<@brief input time delay.
  size_t input_delay_discrete_nsteps{};  // !<@brief discrete time delay steps.

  // Stop state parameters.
  double stop_state_entry_ego_speed{};
  double stop_state_entry_target_speed{};
  double stop_state_keep_stopping_dist{};
  double will_stop_state_dist{};

  // Simulation model for planning simulator.
  bool use_delay_sim_model{};
  bool predict_initial_states{};

  // For interpolator used in the prediction.
  bool use_acceleration_inputs{}; // parameters whether to use NMPC computed acc: Obsolete to be removed.
  bool use_kalman{false};

  // For trajectory initialization. Linear or LPV feedback trajectory initialization.
  bool use_linear_trajectory_initialization{};

  // Use mpc controller or the feedback controllers.
  bool use_mpc_controller{};
  size_t number_of_sqp_iterations{};

  // Vehicle parameters to keep in the node
  double lr{};

  // CDOB DOB parameters.
  bool use_cdob{false};
  bool use_dob{false};

};

struct ParamsFilters
{
  ParamsFilters()
  {
    Vsqrt.setZero();
    Wsqrt.setZero();
    Psqrt.setZero();
  }

  // ~ParamsFilters() = default;
  Model::state_diag_mat_t Vsqrt;  // !<@brief model process uncertainty covariance sqrt.
  Model::state_diag_mat_t Wsqrt;  // !<@brief measurement uncertainty covariance sqrt.
  Model::state_diag_mat_t Psqrt;  // !<@brief Kalman covariance matrix sqrt.

  // UKF specific parameters.
  double ukf_alpha{0.95};
  double ukf_beta{2.0};
  double ukf_kappa{0.5};
};

struct ParamsOptimization
{
  ParamsOptimization()
  {

    // Prepare the weight matrices.
    Q.setIdentity();
    QN.setIdentity();
    R.setIdentity();
    Rj.setIdentity();

    // Prepare the scaling matrix and vectors.
    Sx.setIdentity();  // prepare state scaling diagonal matrix.
    Su.setIdentity();  // prepare control scaling diagonal matrix.

    // Prepare inverse scaling matrices.
    Sx_inv.setIdentity();
    Su_inv.setIdentity();
  }

  //~ParamsOptimization() = default;

  /**
   * @brief State and control upper and lower bounds for the optimization algorithms.
   * Current states [xw, yw, psi, s, ey, e_yaw, v, delta] and controls [vx, steering]_inputs
   * */
  Model::state_vector_t xupper{Model::state_vector_t::Zero()};
  Model::state_vector_t xlower{Model::state_vector_t::Zero()};
  Model::state_vector_t xupper_scaled{Model::state_vector_t::Zero()};
  Model::state_vector_t xlower_scaled{Model::state_vector_t::Zero()};

  Model::input_vector_t uupper{Model::input_vector_t::Zero()};
  Model::input_vector_t ulower{Model::input_vector_t::Zero()};
  Model::input_vector_t uupper_scaled{Model::input_vector_t::Zero()};
  Model::input_vector_t ulower_scaled{Model::input_vector_t::Zero()};

  /** @brief State and control scaling min, max values. */
  std::vector<double>
    scaling_range{std::vector<double>(2, 0.0)};  // the range the variables scaled into [-1, 1] or any.
  Model::state_vector_t xmin_for_scaling{Model::state_vector_t::Zero()};
  Model::state_vector_t xmax_for_scaling{Model::state_vector_t::Zero()};
  Model::input_vector_t umin_for_scaling{Model::input_vector_t::Zero()};
  Model::input_vector_t umax_for_scaling{Model::input_vector_t::Zero()};

  /** @brief State Q and control R weights for quadratic optimization. */
  Model::state_diag_mat_t Q;     // !<@brief State weights [0, ... N-1]
  Model::state_diag_mat_t QN;    // !<@brief State terminal weights [N]
  Model::control_diag_mat_t R;   // !<@brief Control weights.
  Model::control_diag_mat_t Rj;  // !<@brief Jerk weights

  /**
   * Scaling Matrix and and centering vectors.
   * x = S_x * xhat + cx where xhat is scaled between [1, 1]
   * u = S_x * uhat + cu where u is scaled between [-1, 1]
   * */
  Model::state_diag_mat_t Sx;  // Scaling state matrix
  Model::state_vector_t Cx{Model::state_vector_t::Zero()};

  Model::control_diag_mat_t Su;
  Model::input_vector_t Cu{Model::input_vector_t::Zero()};

  /** @brief For inverse scaling, xhat = Sxinv * (x-csclx) and control similarly. */
  Model::state_diag_mat_t Sx_inv;
  Model::control_diag_mat_t Su_inv;

  // OSQP.
  bool osqp_warm_start{};
  bool osqp_polishing{};
  bool osqp_scaling{};
  long int osqp_max_iters{};
  long int osqp_polish_iters{};
  double osqp_time_limit{};
  double osqp_eps_abs{};
  double osqp_eps_rel{};
  bool osqp_verbose{};
  bool osqp_scaled_termination{};
};

/**
 * @brief Parameter container for the LPV initializer.
 * @tparam numOfNonlinearTerms number of nonlinear terms in the state transition matrix.
 * */
template<size_t numOfNonlinearTerms>
struct ParamsLPVFeedback
{

  ParamsLPVFeedback()
  {
    lpvXcontainer.reserve(numOfNonlinearTerms);  // !<@brief including X0, Y0
    lpvYcontainer.reserve(numOfNonlinearTerms);
  }

  // ~ParamsLPVFeedback() = default;

  // LPV parameters
  size_t num_of_nonlinearities{numOfNonlinearTerms};

  Model::lpv_X_matrix_v_t lpvXcontainer;  // !<@brief keeps

  // Lyapunov matrices X's.
  Model::lpv_Y_matrix_v_t lpvYcontainer;  // !<@brief keeps Lyapunov matrices Y's.
};

/**
* @brief Parameter container for the NMPC core class and its members.
* @tparam mpc_numPredictionStepsK number of time step in the prediction horizon,
* */
template<size_t mpc_numPredictionStepsK>
struct DataNMPCcore
{

  DataNMPCcore() = default;

  explicit DataNMPCcore(double const &mpc_dt) : mpc_prediction_dt{mpc_dt}
  {
    trajectory_data.initializeTrajectory(mpc_numPredictionStepsK, mpc_dt);
    target_reference_states_and_controls.initializeTrajectory(mpc_numPredictionStepsK, mpc_dt);
    discretization_data.initializeDiscretizationMatrices(mpc_numPredictionStepsK, mpc_dt);

    auto u0 = trajectory_data.U.at(0);
    u0.setZero();
  }
  // ~DataNMPCcore() = default;

  // Data members.
  double wheel_base{};
  double mpc_prediction_dt{};
  double input_delay_time{};  // !<@brief to use in the predictions.

  // !<-@brief reference speed scaling for feedforward cont.
  double feedforward_speed_set_point_scale{};

  trajectory_data_t trajectory_data;
  trajectory_data_t target_reference_states_and_controls;
  discretization_data_t discretization_data;
};

using param_lpv_type_t = ParamsLPVFeedback<ns_nmpc_interface::NUM_LPV_NONLINEARITIES>;
using data_nmpc_core_type_t = DataNMPCcore<ns_nmpc_interface::MPC_NUM_OF_PRED_STEPS>;

}  // namespace ns_data
#endif  // NONLINEAR_MPC_CORE__DATA_AND_PARAMETER_CONTAINER_HPP_
