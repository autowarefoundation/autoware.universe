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

#ifndef NONLINEAR_MPC_CORE__NMPC_SIMULATION_HPP_
#define NONLINEAR_MPC_CORE__NMPC_SIMULATION_HPP_

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include "nonlinear_mpc_core/active_model.hpp"
#include "utils/nmpc_utils.hpp"
#include "utils/nmpc_utils_eigen.hpp"

namespace ns_sim
{

/**
 *  @brief  ODEzoh class is a functor for using boost integration library for zero order hold control signals.
 *  @param      Model::shared_ptr, differential equation for the model,
 *  @param      dt is time step,
 *  @param	u is the input,
 *  @param	x is the state of the integration. xdot = f(x, u).
 */
class ODEzoh
{
 public:
	ODEzoh() = default;

	explicit ODEzoh(Model::model_ptr_t const &model, const Model::input_vector_t &u,
									Model::param_vector_t const &params, double dt);

	ODEzoh(ODEzoh const &) = default;

	ODEzoh &operator=(ODEzoh const &) = default;

	ODEzoh(ODEzoh &&) noexcept = default;

	ODEzoh &operator=(ODEzoh &&) noexcept = default;

	~ODEzoh() = default;

	/**
	 * @brief Boost integration library integration function signature. f(x, dxdt, t)
	 *
	 * */
	void operator()(const Model::state_vector_t &x, Model::state_vector_t &dfdt, double t);

 private:
	Model::model_ptr_t model_;
	Model::input_vector_t u_{Model::input_vector_t::Zero()};
	Model::param_vector_t params_{Model::param_vector_t::Zero()};
	double dt_;
};

/**
 * @brief Ordinary differential equation class for first order hold integration.
 * */
class ODEfoh
{
 public:
	ODEfoh() = default;

	explicit ODEfoh(Model::model_ptr_t const &model, const Model::input_vector_t &u0,
									const Model::input_vector_t &u1, Model::param_vector_t const &params0,
									Model::param_vector_t const &params1, double dt);

	ODEfoh(ODEfoh const &) = default;

	ODEfoh &operator=(ODEfoh const &) = default;

	ODEfoh(ODEfoh &&) noexcept = default;

	ODEfoh &operator=(ODEfoh &&) noexcept = default;

	~ODEfoh() = default;

	/**
	 * @brief Boost integration library integration function signature. f(x, dxdt, t)
	 * */
	void operator()(const Model::state_vector_t &x, Model::state_vector_t &dxdt, double t);

 private:
	Model::model_ptr_t model_;  // !<-@brief pointer to the vehicle model.

	// !<-@brief control input vector at the beginning of integration.
	Model::input_vector_t u0_{Model::input_vector_t::Zero()};

	// !<-@brief control input vector at the end of integration.
	Model::input_vector_t u1_{Model::input_vector_t::Zero()};

	// !<-@brief [curvature and vx_target] at the beginning of integration
	Model::param_vector_t params0_{Model::param_vector_t::Zero()};

	// !<-@brief [curvature and vx_target]  at the end.
	Model::param_vector_t params1_{Model::param_vector_t::Zero()};
	double dt_{};                      // !<-@brief time step.
};

/**
 *  @brief Ordinary differential equation class for variable speed integration. We use this class when the NMPC
 *  does not control the vehicle longitudinal motion. Instead of using the NMPC acceleration control in the simulations,
 *  we use trajectory speed.
 * */
class ODEvariableSpeed
{
 public:

	ODEvariableSpeed() = default;
	explicit ODEvariableSpeed(Model::model_ptr_t const &model, Model::input_vector_t const &u0,
														Model::param_vector_t const &params0, double const &v0, double const &v1, double dt);

	ODEvariableSpeed(ODEvariableSpeed const &) = default;

	ODEvariableSpeed &operator=(ODEvariableSpeed const &) = default;

	ODEvariableSpeed(ODEvariableSpeed &&) noexcept = default;

	ODEvariableSpeed &operator=(ODEvariableSpeed &&) noexcept = default;

	~ODEvariableSpeed() = default;

	/**
	* @brief Boost integration library integration function signature. f(x, dxdt, t)
	* */
	void operator()(Model::state_vector_t &x, Model::state_vector_t &dxdt, double t);

 private:
	Model::model_ptr_t model_;       // pointer to the vehicle model.
	Model::input_vector_t u0_;       // control input vector at the beginning of integration.
	Model::param_vector_t params0_;  // path curvature at the beginning of integration
	double v0_;
	double v1;
	double dt_;  // time step.
};

/**
*  @brief Given the initial conditions and the control to be applied, integrates the model one-step.
*  @param model_ptr: Pointer to the vehicle model,
*  @param u0; the control signal to be applied constant during the integration.
*  @param dt; time step.
*  @param x; initial state. The final state will be stored in it.
* */
void simulateNonlinearModel_zoh(Model::model_ptr_t model,
																Model::input_vector_t const &u0,
																Model::param_vector_t const &params,
																const double &dt,
																Model::state_vector_t &x);

void simulateNonlinearModel_foh(Model::model_ptr_t model,
																Model::input_vector_t const &u0,
																Model::input_vector_t const &u1,
																Model::param_vector_t const &params0,
																Model::param_vector_t const &params1,
																const double &dt,
																Model::state_vector_t &x);

void simulateNonlinearModel_variableSpeed(Model::model_ptr_t model,
																					Model::input_vector_t const &u0,
																					Model::param_vector_t const &params0,
																					double const &v0,
																					double const &v1,
																					const double &dt,
																					Model::state_vector_t &x);

}  // namespace ns_sim

#endif  // NONLINEAR_MPC_CORE__NMPC_SIMULATION_HPP_
