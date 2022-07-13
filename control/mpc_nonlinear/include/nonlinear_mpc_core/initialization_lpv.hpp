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

#ifndef NONLINEAR_MPC_CORE__INITIALIZATION_LPV_HPP_
#define NONLINEAR_MPC_CORE__INITIALIZATION_LPV_HPP_

#include <Eigen/StdVector>
#include <vector>
#include "nonlinear_mpc_core/active_model.hpp"
#include "nonlinear_mpc_core/data_and_parameter_container.hpp"
#include "nonlinear_mpc_core/nmpc_simulation.hpp"
#include "splines/interpolating_spline_pcg.hpp"
#include "vehicle_models/kinematic_model_definitions.hpp"

class LPVinitializer
{
 public:
	LPVinitializer() = default;

	explicit LPVinitializer(size_t const &number_of_nonlinear_terms_in_A)
		: ntheta_(number_of_nonlinear_terms_in_A)
	{
		thetas_ = std::vector<double>(ntheta_);
	}

	// Copy  constructors.
	LPVinitializer(LPVinitializer const &other);

	LPVinitializer &operator=(LPVinitializer const &other);

	//  // Move constructors.
	//    LPVinitializer(LPVinitializer &&other) noexcept;
	//
	//    LPVinitializer &operator=(LPVinitializer &&other) noexcept;
	//
	// ~LPVinitializer() = default;

	// Methods.
	/**
	 * @brief Computes X(k), U(k) by feedback control, using the model equations and the parameters.
	 * @param model_ptr pointer to the vehicle model.
	 * @param piecewise_interpolator piecewise interpolator for the curvature.
		* */
	bool simulateWithFeedback(Model::model_ptr_t const &model_ptr,
														ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator,
														ns_data::param_lpv_type_t const &params_lpv, ns_data::ParamsOptimization const &param_opt,
														ns_data::data_nmpc_core_type_t &nmpc_data);

	bool computeSingleFeedbackControls(Model::model_ptr_t const &model_ptr,
																		 ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator,
																		 ns_data::param_lpv_type_t const &params_lpv,
																		 ns_data::ParamsOptimization const &params_opt,
																		 ns_data::data_nmpc_core_type_t &nmpc_data,
																		 double const &dt);

 private:
	size_t ntheta_{};  // <- @brief Number of nonlinear terms in the state transition matrix A(theta).
	std::vector<double> thetas_{};  // <-@brief Computed non-linearities are stored in this vector.
};

#endif  // NONLINEAR_MPC_CORE__INITIALIZATION_LPV_HPP_
