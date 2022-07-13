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

#ifndef NONLINEAR_MPC_CORE__ACTIVE_MODEL_HPP_
#define NONLINEAR_MPC_CORE__ACTIVE_MODEL_HPP_

#include "eigen3/Eigen/Core"
#include "vehicle_models/kinematic_vehicle_model.hpp"

// #define kInfinity std::numeric_limits<double>::infinity()  // !<-@brief OSQP infinity.
constexpr double kInfinity = std::numeric_limits<double>::infinity();

// !<-@brief Model used throughout the node.
using Model = ns_models::KinematicModelSingleTrackModel;
using trajectory_data_t = Model::trajectory_data_t;  // !<-@brief trajectory data class
using discretization_data_t = Model::discretization_data_t;

/**
 * @brief Nonlinear MPC and LPV definitions for the templates used.
 * */
namespace ns_nmpc_interface
{
constexpr size_t MPC_NUM_OF_PRED_STEPS = 50;
constexpr size_t NUM_LPV_NONLINEARITIES = 7;
}  // namespace ns_nmpc_interface

namespace ns_nmpc_splines
{
constexpr size_t MPC_MAP_SMOOTHER_IN = 100;   // !<-@brief Autoware points are re-sampled into.
constexpr size_t MPC_MAP_SMOOTHER_OUT = 80;  // !<-@brief Re-sampled points are smoothed out.
}  // namespace ns_nmpc_splines

/**
 * @brief Number of sigma points in the UKF implementation.
 * */
namespace ns_filters
{
constexpr size_t UKF_NUM_OF_SIGMAS = 2 * Model::state_dim + 1;
}

/**
 * @brief  Map matrix types. The Autoware trajectory is first re-sampled to a fixed number of
 * points which enter the to algorithms. Then these points are sent to the smoothers [out].
 *   - The raw reference map consists of the columns [s, x, y, z]
*/
using map_matrix_in_t = Eigen::Matrix<double, ns_nmpc_splines::MPC_MAP_SMOOTHER_IN, 4>;
// using map_matrix_out_t = Eigen::Matrix<double, ns_nmpc_interface::MPC_MAP_SMOOTHER_OUT, 6>;

// Using Map vector types
using map_vector_in_t = Eigen::Matrix<double, ns_nmpc_splines::MPC_MAP_SMOOTHER_IN, 1>;
using map_vector_out_t = Eigen::Matrix<double, ns_nmpc_splines::MPC_MAP_SMOOTHER_OUT, 1>;

#endif  // NONLINEAR_MPC_CORE__ACTIVE_MODEL_HPP_
