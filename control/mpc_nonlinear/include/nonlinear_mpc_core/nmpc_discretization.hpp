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

#ifndef NONLINEAR_MPC_CORE__NMPC_DISCRETIZATION_HPP_
#define NONLINEAR_MPC_CORE__NMPC_DISCRETIZATION_HPP_

#include <Eigen/StdVector>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include "active_model.hpp"
#include "external/eigenIntegration.hpp"
#include "splines/interpolating_spline_pcg.hpp"

/**
 * @brief  The discrete matrices A, B0, B1 are prepared by these modules. The continuous Ac, Bc are put in the 1D vector
 *  form and integrated for a given interval within the Fundamental Matrix framework.
 *
 *  First Order Hold integration is applied to the inputs. In FOH, the inputs to the model u0, u1 and curvature 0, 1
 *  are interpolated within the RK integration term.
 *
 *  Constructor parameters:
 *  @param  model; shared pointer to the model.
 *  @param u0_u1; two-column input vector keeping u0 and u1 in the columns.
 *  @param kappa0_kappa1; curvature input similar to the intput vector.
 *
 * */

namespace ns_discretization
{
/**
 * @brief ODE class for integrating system matrices for the Boost integration methods.
 * */
class ODEfoh
{
 public:
	/**
	 * @brief ODEzoh matrix for A, B0, B1, Residuals z
	 *  All the matrix and vectors have the same size of rows.  V = ode_matrix_type, columns are reserved for (x0, A, B0, B1, z)
	 *
	 * */

	/**
	 * @brief If FOH. V = [x0, A, B0, B1, z]
	 *  using ode_matrix_t = typename Eigen::Matrix<double, Model::state_dim,
	 *		 + Model::state_dim + Model::input_dim +
	 *		 Model::input_dim + 1>;
	 */
	// If ZOH. V = [x0, A, B0, z]
	using ode_matrix_t = typename Eigen::Matrix<double, Model::state_dim, 1 + Model::state_dim + Model::input_dim + 1>;

	explicit ODEfoh(Model::model_ptr_t model,
									const Model::input_vector_t &u0,
									const Model::input_vector_t &u1,
									Model::param_vector_t const &params0,
									Model::param_vector_t const &params1, double dt);

	/**
	* @brief operator () that boost integration library integration function signature f(x, dxdt, t) requires.
	* */
	void operator()(const ode_matrix_t &V, ode_matrix_t &dVdt, double t);

 private:
	Model::model_ptr_t model_;       // !<-@brief pointer to the vehicle model.
	Model::input_vector_t u0_;       // !<-@brief control input vector at the beginning of integation.
	Model::input_vector_t u1_;       // !<-@brief control input vector at the end of integration.
	Model::param_vector_t params0_;  // !<-@brief path curvature at the beginning of integration
	Model::param_vector_t params1_;  // !<-@brief curvature at the end.
	double dt_;                      // !<-@brief time step.
};

/**
 * @brief zero order ODE class for system matrix integration.
 * */
class ODEzoh
{
 public:
	using ode_matrix_t = typename Eigen::Matrix<double, Model::state_dim, 1 + Model::state_dim + Model::input_dim + 1>;

	/**
	 * @brief ODEzoh matrix for A, B0, B1, Residuals z * All the matrix and vectors have
	 * the same size of rows. V = ode_matrix_type, columns are reserved for (x0, A, B0, B1, z)
	 * */

	/**
	 * @brief If FOH. V = [x0, A, B0, B1, z]
	 * using ode_matrix_t = typename Eigen::Matrix<double, Model::state_dim,
	 * 1 + Model::state_dim + Model::input_dim
	 * + Model::input_dim + 1>;
	 * */

	// If ZOH. V = [x0, A, B0, z] using ode_matrix_t = typename Eigen::Matrix<double,
	// Model::state_dim, 1 + Model::state_dim + Model::input_dim + 1>;
	explicit ODEzoh(Model::model_ptr_t model, const Model::input_vector_t &u0,
									Model::param_vector_t const &params0, double dt);

	/**
	 * @brief operator () that boost integration library integration function signature
	 * f(x, dxdt, t) requires.
	 */
	void operator()(const ode_matrix_t &V, ode_matrix_t &dVdt, double t);

 private:
	Model::model_ptr_t model_;      // !<-@brief pointer to the vehicle model.
	Model::input_vector_t u0_;      // !<-@brief control input vector at the beginning of integration.
	Model::param_vector_t params0_;  //  !<-@brief path curvature at the beginning of integration.
	double dt_;                     //  !<-@brief time step.
};

/**
 *  @brief First Order Hold Implementation. The input is treated as a line during the integration interval.
 *  Gets the reference state and input trajectories X and U and compute the Jacobians.
 *  The Jacobians are stored in the trajectory_data_ object.
 *  @param model_ptr_   : shared pointer to the vehicle model.
 *  @param td           : trajectory data object that stores the reference states and controls over the
 *  prediction horizon.
 *  @param ns_splines   : InterpolatingSplinePCG &piecewise_interpolator:  reference map columns :
 *  [s, x, y, psi, kappa=curvature]
 *  @param dt           : time step [seconds]
 *  @param dd           : discretization data vector that keeps the system matrices A, B, and residuals z.
 * */
bool multipleShootingTrajectory(Model::model_ptr_t const &model_ptr, trajectory_data_t const &trajectory_data,
																trajectory_data_t const &target_states,
																ns_splines::InterpolatingSplinePCG const &piecewise_interpolator, double const &dt,
																discretization_data_t &dd);

bool bilinearTransformation(Model::model_ptr_t const &model_ptr,
														trajectory_data_t const &trajectory_data,
														trajectory_data_t const &target_state,
														ns_splines::InterpolatingSplinePCG const &piecewise_interpolator,
														double const &ts,
														discretization_data_t &dd);

/**
 * @brief Discretisize one-step Ad, Bd system matrices.
 * */
bool multipleShootingSingleStep(Model::model_ptr_t const &model_ptr, Model::state_vector_t const &x0,
																Model::input_vector_t const &u0, Model::param_vector_t const &params0, double const &dt,
																Model::state_matrix_t &Ad, Model::control_matrix_t &Bd);

bool bilinearTransformationOneStep(Model::model_ptr_t const &model_ptr,
																	 Model::state_vector_t const &x0,
																	 Model::input_vector_t const &u0,
																	 Model::param_vector_t const &params0,
																	 double const &dt,
																	 Model::state_matrix_t &Ad,
																	 Model::control_matrix_t &Bd);

}  // namespace ns_discretization
#endif  // NONLINEAR_MPC_CORE__NMPC_DISCRETIZATION_HPP_
